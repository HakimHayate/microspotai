import numpy as np
import math
import threading

import time
from clean_icp import icp as clean_icp

from graph import Graph
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from se2 import v2t, t2v, relative, wrap_angle
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
from optimizer import magic_optimizer
from map_manager import OccupancyGridMap

global_yaw_prev = None
global_yaw_current = 0.0

global_pts = None
global_c = 0

map = OccupancyGridMap()
data_ready_event = threading.Event()

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        self.sub_imu_yaw = self.create_subscription(
            Float64,
            '/imu/yaw',
            self.yaw_callback,
            10)
        
        self.sub_laser = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.map_pub_ = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )

    def yaw_callback(self, msg):
        global global_yaw_current
        global_yaw_current = msg.data

    def scan_callback(self, msg):
        global global_pts
        global global_c

        ranges = msg.ranges
        pts = []
        for i in range(len(ranges)):
            if msg.range_max >= ranges[i] >= msg.range_min:
                angle = msg.angle_min + i * msg.angle_increment
                x = math.cos(angle) * ranges[i]
                y = math.sin(angle) * ranges[i]
                pts.append((x,y))
        global_pts = np.array(pts)
        global_c += 1
        data_ready_event.set()

def pub_path_msg(node, pose_):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        
        pose.pose.position.x = float(pose_[0])
        pose.pose.position.y = float(pose_[1])
        pose.pose.position.z = 0.0
        
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(pose_[2] / 2.0)
        pose.pose.orientation.w = math.cos(pose_[2] / 2.0)

        node.path_msg.poses.append(pose)
        node.path_msg.header.stamp = node.get_clock().now().to_msg()
        node.path_pub.publish(node.path_msg)

def compress(data):
    ros_data = np.full(data.shape, -1, dtype=np.int8)
    
    ros_data[data < -0.50] = 0    
    ros_data[data > 0.50] = 100   
    
    return ros_data.tolist()

def pub_map(node):
    global map 
    
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.header.stamp = node.get_clock().now().to_msg()
    
    grid.info.resolution = map.resolution_
    grid.info.width = map.width_cells_
    grid.info.height = map.height_cells_
    
    grid.info.origin.position.x = float(-(grid.info.width * grid.info.resolution) / 2.0)
    grid.info.origin.position.y = float(-(grid.info.height * grid.info.resolution) / 2.0)
    grid.info.origin.position.z = 0.0
    
    grid.info.origin.orientation.x = 0.0
    grid.info.origin.orientation.y = 0.0
    grid.info.origin.orientation.z = 0.0
    grid.info.origin.orientation.w = 1.0

    flat_data = map.map_.flatten()
    grid.data = compress(flat_data)
    
    node.map_pub_.publish(grid)

def rebuild_and_publish_path(node, graph):
    node.path_msg.poses.clear()
    
    sorted_node_ids = sorted(graph.nodes_dict_.keys())
    
    for node_id in sorted_node_ids:
        pose_ = graph.nodes_dict_[node_id].pose_
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        
        pose.pose.position.x = float(pose_[0])
        pose.pose.position.y = float(pose_[1])
        pose.pose.position.z = 0.0
        
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(pose_[2] / 2.0)
        pose.pose.orientation.w = math.cos(pose_[2] / 2.0)
        
        node.path_msg.poses.append(pose)
        
    node.path_msg.header.stamp = node.get_clock().now().to_msg()
    node.path_pub.publish(node.path_msg)

def mapping_worker(node):
    global global_pts
    global global_yaw_current
    global global_c

    nb_used_scans = 0

    current_yaw = None
    prev_yaw = None

    current_pts = None
    prev_pts = None

    T_global_prev = np.eye(3)
    T_global_current = np.eye(3)
    
    graph = Graph()
    prev_node_id = None
    current_node_id = None

    while True:
        run_optimizer = False
        
        data_ready_event.wait()
        
        if global_pts is None or global_yaw_current is None:
            time.sleep(0.01)
            continue
        
        current_pts = global_pts
        nb_used_scans += 1

        current_yaw = global_yaw_current

        if prev_yaw is None:
            prev_yaw = current_yaw
        
        yaw = wrap_angle(current_yaw - prev_yaw)

        if prev_pts is None:
            prev_node_id = graph.add_node(t2v(np.eye(3)), current_pts)
            prev_pts = current_pts
            continue

        m = min(prev_pts.shape[0], current_pts.shape[0])
       
        res = clean_icp(prev_pts, current_pts, np.array([0, 0, yaw]))
        if res is None: # ICP didnt converge
            continue

        T_prev_current, error = res
        measurement = t2v(T_prev_current)
        if np.linalg.norm(measurement[:2]) > 0.5:
            print("ICP Diverged! Ignoring frame.")
            continue
        #print(f'relative pose = {measurement}, error {error}\n')

        if np.linalg.norm(measurement[:2]) < 1e-3 and abs(measurement[2]) < 0.0174533 * 1: # Robot didnt move
            continue

        T_global_current = T_global_prev @ T_prev_current
        current_node_id = graph.add_node(t2v(T_global_current), current_pts)
        graph.add_edge(prev_node_id, current_node_id, measurement)
        map.update(t2v(T_global_current), current_pts)
        pub_map(node)

        pub_path_msg(node, t2v(T_global_current))
        
        #print(f'global coordinate : {t2v(T_global_current)}')
        
        prev_pts = current_pts
        prev_yaw = current_yaw
        T_global_prev = T_global_current
        prev_node_id = current_node_id

        condidates = graph.check_loop_closure(current_node_id)
        if len(condidates) > 0:
            for id in condidates:
                res = clean_icp(graph.nodes_dict_[id].pts_, current_pts, relative(graph.nodes_dict_[id].pose_, graph.nodes_dict_[current_node_id].pose_))
                if res is not None: # ICP converged
                    T, error = res
                    run_optimizer = True
                    graph.add_edge(current_node_id, id, t2v(T))

        if run_optimizer:
            magic_optimizer(graph)
            rebuild_and_publish_path(node, graph)
            map.rebuild(graph)
            pub_map(node)
            T_global_prev = v2t(graph.nodes_dict_[current_node_id].pose_)

        print(f'ration scans effeciency {nb_used_scans/global_c:.2f}')
        

def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    worker_thread = threading.Thread(target=mapping_worker, daemon=True, args=(node,))
    
    worker_thread.start()

    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()