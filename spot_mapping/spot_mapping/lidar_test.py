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

data_queue = queue.Queue(maxsize=1000)

plot_queue = queue.Queue(maxsize=100) 
shutdown_event = threading.Event()

def producer():
    PORT_NAME = "COM6"
    lidar = RPLidar(PORT_NAME, timeout=3, baudrate=115200)
    
    try:
        print('LIDAR Started...')
        while not shutdown_event.is_set():
            for scan in lidar.iter_scans():
                if data_queue.full():
                    try: data_queue.get_nowait()
                    except queue.Empty: pass

                data_queue.put(scan)
                if shutdown_event.is_set():
                    break
    except Exception as e:
        print(f"LIDAR Error: {e}")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("LIDAR disconnected.")

def consumer():
    scan_history = []
    while not shutdown_event.is_set():
        while not data_queue.empty():
            try:
                scan_history.append(data_queue.get_nowait()) 
            except queue.Empty:
                break
        pts = []
        if len(scan_history) >= 1:
            scan_history = scan_history[-1:]
            
            angle_min = np.min(np.array(scan_history[0])[:, 1])
            angle_max = np.max(np.array(scan_history[0])[:, 1])
            for scan in scan_history:
                print(scan)
                current_pts = polar_to_cartezian(scan)
                pts.extend(current_pts)
            print(angle_min, angle_max)
            if plot_queue.full():
                try: plot_queue.get_nowait()
                except queue.Empty: pass
                
            plot_queue.put(np.array(pts))
        
        else:
            time.sleep(0.02)


def main():
    producer_thread = threading.Thread(target=producer, daemon=True)
    consumer_thread = threading.Thread(target=consumer, daemon=True)
    
    producer_thread.start()
    consumer_thread.start()
    
    plt.ion()
    fig, ax = plt.subplots()
    
    try:
        while True:
            pts = None
            
            while not plot_queue.empty():
                try:
                    pts = plot_queue.get_nowait()
                except queue.Empty:
                    break
            
            if pts is not None:
                ax.clear()
                ax.scatter(pts[:, 0], pts[:, 1], s=5, c='blue', alpha=0.5)
                ax.axis('equal') 
                
                plt.pause(0.01) 
            else:
                time.sleep(0.05)
                
    except KeyboardInterrupt:
        print("Stopping threads...")
    finally:
        shutdown_event.set()
        plt.ioff()
        plt.close()

if __name__ == '__main__':
    main()