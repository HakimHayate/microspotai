from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
from nearestNeighbor import Tree
from utils import polar_to_cartezian
import math
import threading
import queue
import time
from clean_icp import icp as clean_icp
from map_manager import OccupancyGridMap

from graph import Graph
from optimizer import magic_optimizer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

global_yaw = 0.0

class YawSubscriber(Node):
    def __init__(self):
        super().__init__('yaw_subscriber')
        self.subscription = self.create_subscription(
            Float64,
            '/imu/yaw',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global global_yaw
        global_yaw = msg.data

        
map = OccupancyGridMap()

data_queue = queue.Queue(maxsize=1000)

plot_queue = queue.Queue(maxsize=2)

def producer():
    PORT_NAME = "COM6"
    lidar = RPLidar(PORT_NAME, timeout=3, baudrate=115200)
    
    try:
        print('LIDAR Started. Gathering one scan every 3 seconds...')
        while True:
            
            for scan in lidar.iter_scans():
                if data_queue.full():
                    try: data_queue.get_nowait()
                    except queue.Empty: pass

                data_queue.put(scan)
                
    except Exception as e:
        print(f"LIDAR Error: {e}")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("LIDAR disconnected cleanly.")

def get_prob_map():
        return 1 / (1 + np.exp(-map.map_)).T

from se2 import t2v, v2t, relative

def consumer():
    translation = []
    rotation = []

    tol_trans = 100#mm
    tol_rotation = 0.5

    graph = Graph()
    scan = None
    prev_pts = None
    T_global_prev = np.eye(3)
    T_global_prev[0, -1] = 0 #map.width_cells_ * map.resolution_ / 2
    T_global_prev[1, -1] = 0 #map.height_cells_ * map.resolution_ / 2
    init = False
    src_id = graph.add_node(t2v(T_global_prev))
    j = 0
    while True:
        j += 1
        while True:
            scan = data_queue.get()
            try:
                scan = data_queue.get_nowait() 
            except queue.Empty:
                break
        
        if not init:
            init = True
            continue

        current_pts = polar_to_cartezian(scan)

        if prev_pts is None:
            prev_pts = current_pts
            continue

        m = min(prev_pts.shape[0], current_pts.shape[0])
        

        T_prev_current = clean_icp(prev_pts[:m, :], current_pts[:m, :], global_yaw)
        print(f'yaw = {global_yaw}')
        measurement = t2v(T_prev_current)

        if np.linalg.norm(measurement[:2]) < tol_trans or (measurement[-1]) < tol_rotation:
            continue

        T_global_current = T_global_prev @ T_prev_current
        dst_id = graph.add_node(t2v(T_global_current))

        

        graph.add_edge(src_id, dst_id, measurement)

        T_global_prev = T_global_current
        src_id = dst_id

        
        pose_theta = math.atan2(T_global_current[1, 0], T_global_current[0, 0])
        pose_x, pose_y = T_global_current[:-1, -1]
        print(f'pose = {measurement}')
        print(f'global coordinate : {pose_x, pose_y, pose_theta}')
        #map.update(pose_x, pose_y, pose_theta, scan)

        

        if plot_queue.full():
            try: plot_queue.get_nowait()
            except queue.Empty: pass
            
        # plot_queue.put((current_pts, prev_pts))
        prev_pts = current_pts
        #graph.draw_graph()

        plt.clf() 
        print(len(current_pts))
        plt.plot(current_pts[:, 0], current_pts[:, 1], 'r.', markersize=2)
        
        plt.axis('equal') 
        plt.grid(True)
        
        plt.draw()
        plt.pause(0.01)
        
shutdown_event = threading.Event()

def main(args=None):
    producer_thread = threading.Thread(target=producer, daemon=True)
    consumer_thread = threading.Thread(target=consumer, daemon=True)
    
    producer_thread.start()
    consumer_thread.start()
    
    rclpy.init(args=args)
    yaw_subscriber = YawSubscriber()
    
    try:
        rclpy.spin(yaw_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        yaw_subscriber.destroy_node()
        rclpy.shutdown()
        shutdown_event.set()
    

if __name__ == '__main__':
    main()