import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from optimizer import magic_optimizer
from map_manager import OccupancyGridMap
from graph import Graph
from se2 import *
import math
from pathfinding import PathFinding
from icp_point2line import icp_point2line

class SLAMBackendNode(Node):
    def __init__(self):
        super().__init__('slam_backend_node')
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.path = []
        self.graph = Graph()
        self.global_map = OccupancyGridMap()
        self.last_node_id = None
        self.latest_raw_pose = None
        self.pathfinder = PathFinding(self.global_map)

        self.keyframe_sub = self.create_subscription(
            Float32MultiArray,
            '/slam/keyframe',
            self.keyframe_callback,
            10
        )

        self.data_planning_pub = self.create_publisher(Float32MultiArray, '/current_pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.planned_path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose', 
            self.goal_callback,
            10
        )



        self.get_logger().info("SLAM Backend Initialized.")

    def goal_callback(self, msg):
        dest_x = msg.pose.position.x
        dest_y = msg.pose.position.y
        dst_pose_map = self.global_map.world_to_grid(dest_x, dest_y)
        current_pose_map = self.global_map.world_to_grid(self.latest_raw_pose[0],self.latest_raw_pose[1])
        path_map = self.pathfinder.get_path(current_pose_map, dst_pose_map)
        path_world = self.global_map.grid_to_world(path_map)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        planned_poses = []
        
        for pose in path_world:
            
            p = PoseStamped()
            p.header.stamp = path_msg.header.stamp
            p.header.frame_id = 'map'
            
            p.pose.position.x = float(pose[0])
            p.pose.position.y = float(pose[1])
            p.pose.position.z = 0.0
            
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0
            planned_poses.append(p)

        path_msg.poses = planned_poses
        self.planned_path_pub.publish(path_msg)

    def compress(self, data):
        ros_data = np.full(data.shape, -1, dtype=np.int8)
        
        ros_data[data < -0.50] = 0    
        ros_data[data > 0.50] = 100   
        
        return ros_data.tolist()
    
    def pub_map(self):
        
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = self.get_clock().now().to_msg()
        
        grid.info.resolution = self.global_map.resolution_
        grid.info.width = self.global_map.width_cells_
        grid.info.height = self.global_map.height_cells_
        
        grid.info.origin.position.x = float(-(grid.info.width * grid.info.resolution) / 2.0)
        grid.info.origin.position.y = float(-(grid.info.height * grid.info.resolution) / 2.0)
        grid.info.origin.position.z = 0.0
        
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0

        flat_data = self.global_map.map_.T.flatten()
        grid.data = self.compress(flat_data)
        
        self.map_pub.publish(grid)

    def keyframe_callback(self, msg):
        raw_data = np.array(msg.data, dtype=np.float32)
        current_raw_pose = raw_data[:3].tolist()

        msg = Float32MultiArray()
        msg.data = current_raw_pose
        self.data_planning_pub.publish(msg)

        current_pts = raw_data[3:].reshape(-1, 2)


        if self.last_node_id is None:
            node_id = self.graph.add_node(current_raw_pose, current_pts)
            global_pts = project(v2t(current_raw_pose), current_pts)
            pts_map = self.global_map.world_to_grid_vectorized(global_pts)
            robot_pos_map = self.global_map.world_to_grid(current_raw_pose[0], current_raw_pose[1])
            self.global_map.update(pts_map, robot_pos_map)
        else:
            delta_mvt = relative(self.latest_raw_pose, current_raw_pose)
            pose_optimized = t2v(v2t(self.graph.nodes_dict_[self.last_node_id].pose_) @ v2t(delta_mvt)) 
            print(f'current pose {pose_optimized}')
            node_id = self.graph.add_node(pose_optimized, current_pts)
            
            self.graph.add_edge(self.last_node_id, node_id, delta_mvt)

            global_pts = project(v2t(pose_optimized), current_pts)
            pts_map = self.global_map.world_to_grid_vectorized(global_pts)
            robot_pos_map = self.global_map.world_to_grid(pose_optimized[0], pose_optimized[1])
            self.global_map.update(pts_map, robot_pos_map)
            self.pub_path(pose_optimized)
            

        #self.process_loop_closures(node_id)

        self.latest_raw_pose = current_raw_pose
        self.last_node_id = node_id
        self.publish_global_state()

        

    def process_loop_closures(self, new_node_id):
        candidates = self.graph.check_loop_closure(new_node_id)
        
        if len(candidates) > 0:
            for id in candidates:
                temp_map = OccupancyGridMap()
                window_size = 5
                start_idx = max(0, id - window_size)
                end_idx = min(len(self.graph.nodes_dict_), id + window_size + 1)
                
                for window_id in range(start_idx, end_idx):
                    pose = self.graph.nodes_dict_[window_id].pose_
                    pts = self.graph.nodes_dict_[window_id].pts_
                    
                    projected_pts = project(v2t(pose), pts)
                    temp_map.update(
                        temp_map.world_to_grid_vectorized(projected_pts), 
                        temp_map.world_to_grid(pose[0], pose[1])
                    )
                measurement, score, _, _ = temp_map.match(
                    relative(self.graph.nodes_dict_[id].pose_, self.graph.nodes_dict_[new_node_id].pose_), 
                    self.graph.nodes_dict_[new_node_id].pts_, x_range_pixels=10, y_range_pixels=10
                )
                self.graph.add_edge(id, new_node_id, measurement, is_loop_closure=True)

            magic_optimizer(self.graph)
            self.global_map.rebuild(self.graph)
            self.rebuild_path()
            
    def rebuild_path(self):
        self.path = []
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        sorted_node_ids = sorted(self.graph.nodes_dict_.keys(), key=int)
        
        for n_id in sorted_node_ids:
            pose_ = self.graph.nodes_dict_[n_id].pose_
            
            p = PoseStamped()
            p.header.frame_id = 'map'
            
            p.pose.position.x = float(pose_[0])
            p.pose.position.y = float(pose_[1])
            p.pose.position.z = 0.0
            
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = math.sin(pose_[2] / 2.0)
            p.pose.orientation.w = math.cos(pose_[2] / 2.0)
            
            self.path.append(p)

        path_msg.poses = self.path
        self.path_pub.publish(path_msg)

    def pub_path(self, pose):
        p = PoseStamped()
        p.header.frame_id = 'map'
        
        p.pose.position.x = float(pose[0])
        p.pose.position.y = float(pose[1])
        p.pose.position.z = 0.0
        
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = math.sin(pose[2] / 2.0)
        p.pose.orientation.w = math.cos(pose[2] / 2.0)
        self.path.append(p)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        path_msg.poses = self.path
        self.path_pub.publish(path_msg)

    def publish_global_state(self):
        self.pub_map()
        
        if self.last_node_id is not None:
            optimized_pose = self.graph.nodes_dict_[self.last_node_id].pose_

            T_map_base = v2t(optimized_pose)
            T_odom_base = v2t(self.latest_raw_pose)
            T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
            
            map_odom_vec = t2v(T_map_odom)
            
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'map'
            tf_msg.child_frame_id = 'odom'
            
            tf_msg.transform.translation.x = float(map_odom_vec[0])
            tf_msg.transform.translation.y = float(map_odom_vec[1])
            tf_msg.transform.translation.z = 0.0
            
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = math.sin(map_odom_vec[2] / 2.0)
            tf_msg.transform.rotation.w = math.cos(map_odom_vec[2] / 2.0)
            
            self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SLAMBackendNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()