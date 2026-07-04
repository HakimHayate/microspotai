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
    tol = 0.1
    graph = Graph()
    scan = None
    prev_pts = None
    T_global_prev = np.eye(3)
    T_global_prev[0, -1] = map.width_cells_ * map.resolution_ / 2
    T_global_prev[1, -1] = map.height_cells_ * map.resolution_ / 2
    init = False
    src_id = graph.add_node(t2v(T_global_prev))
    while True:

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
        

        T_prev_current, error = clean_icp(prev_pts[:m, :], current_pts[:m, :])
        measurement = t2v(T_prev_current)
        if np.linalg.norm(measurement) < tol:
            continue

        T_global_current = T_global_prev @ T_prev_current
        dst_id = graph.add_node(t2v(T_global_current))

        

        graph.add_edge(src_id, dst_id, measurement)

        T_global_prev = T_global_current
        src_id = dst_id

        
        pose_theta = math.atan2(T_global_current[1, 0], T_global_current[0, 0])
        pose_x, pose_y = T_global_current[:-1, -1]
        print(f'pose_theta = {pose_theta}')
        print(error)
        map.update(pose_x, pose_y, pose_theta, scan)

        

        if plot_queue.full():
            try: plot_queue.get_nowait()
            except queue.Empty: pass
            
        # plot_queue.put((current_pts, prev_pts))
        prev_pts = current_pts
        #graph.draw_graph()
        
shutdown_event = threading.Event()

def main():
    producer_thread = threading.Thread(target=producer, daemon=True)
    consumer_thread = threading.Thread(target=consumer, daemon=True)
    plt.ion()
    producer_thread.start()
    consumer_thread.start()
    try:
        while producer_thread.is_alive() or consumer_thread.is_alive():
            producer_thread.join(timeout=0.1)
            consumer_thread.join(timeout=0.1)
            
    except KeyboardInterrupt:
        shutdown_event.set()
    

if __name__ == '__main__':
    main()