from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
from nearestNeighbor import Tree
from utils import polar_to_cartezian
import math
from utils import icp_version2
import threading
import queue
import time
from clean_icp import icp as clean_icp

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

def consumer():
    prev_pts = None

    while True:
        while True:
            scan = data_queue.get()
            try:
                scan = data_queue.get_nowait() 
            except queue.Empty:
                break
        current_pts = polar_to_cartezian(scan)

        if prev_pts is None:
            prev_pts = current_pts
            continue

        m = min(prev_pts.shape[0], current_pts.shape[0])
        
        
        T2 = clean_icp(prev_pts[:m, :], current_pts[:m, :])
        
        
        theta2 = math.degrees(math.atan2(T2[1, 0], T2[0, 0]))
        t2 = T2[:-1, -1]
        if plot_queue.full():
            try: plot_queue.get_nowait()
            except queue.Empty: pass
            
        plot_queue.put((current_pts, prev_pts, theta2, t2))
        prev_pts = current_pts
        time.sleep(3)



producer_thread = threading.Thread(target=producer, daemon=True)
consumer_thread = threading.Thread(target=consumer, daemon=True)

producer_thread.start()
consumer_thread.start()


plt.ion()
fig, ax = plt.subplots()

try:
    while True:
        try:
            
            current_pts, prev_pts, theta,  t = plot_queue.get(timeout=0.1)
            
            print(f'theta = {theta:.2f}')
            print(f't = {t}')
            ax.cla()
            ax.set_aspect('equal')
            ax.set_xlim(-5000, 5000)
            ax.set_ylim(-5000, 5000)

            ax.scatter(current_pts[:,0], current_pts[:,1], c='red', s=1, label='current')
            ax.scatter(prev_pts[:,0], prev_pts[:,1], c='black', s=1, label='prev')

            ax.scatter([1000.0], [0.0], c='r', s=1.5)
            ax.scatter([0.0], [1000.0], c='b', s=1.5)
            ax.scatter([0.0], [0.0], c='black', s=1)
            ax.legend()
            plt.draw()
            
        except queue.Empty:
            pass 
            
        # Refreshes the window backend GUI frame
        plt.pause(0.05)

except KeyboardInterrupt:
    print("Stopping application...")