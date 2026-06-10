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


def consumer():
    scan = None
    prev_pts = None
    T_global_current = np.eye(3)
    T_global_current[0, -1] = map.width_cells_ * map.resolution_ / 2
    T_global_current[1, -1] = map.height_cells_ * map.resolution_ / 2
    init = False
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
        
        
        T_current_prev = clean_icp(prev_pts[:m, :], current_pts[:m, :])
        T_global_current = T_global_current @ np.linalg.inv(T_current_prev)
        
        pose_theta = math.atan2(T_global_current[1, 0], T_global_current[0, 0])
        pose_x, pose_y = T_global_current[:-1, -1]
        print(f'pose_theta = {pose_theta}')
        print('calling map update')
        map.update(pose_x, pose_y, pose_theta, scan)



        if plot_queue.full():
            try: plot_queue.get_nowait()
            except queue.Empty: pass
            
        # plot_queue.put((current_pts, prev_pts))
        prev_pts = current_pts
        time.sleep(0.5)


def main():
    producer_thread = threading.Thread(target=producer, daemon=True)
    consumer_thread = threading.Thread(target=consumer, daemon=True)

    producer_thread.start()
    consumer_thread.start()


    plt.ion()
    fig, ax = plt.subplots()

    img = ax.imshow(
        get_prob_map(),
        cmap="gray",
        vmin=0,
        vmax=1,
        origin="lower"
    )

    plt.show()

    try:
        while True:

            img.set_data(get_prob_map())

            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.01)

    except KeyboardInterrupt:
        print("Stopping application...")

if __name__ == '__main__':
    main()