import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import threading
from rplidar import RPLidar
import math

shutdown_event = threading.Event()

def lidar_worker(node):
    PORT_NAME = "/ttyUSB0"
    lidar = RPLidar(PORT_NAME, timeout=3, baudrate=115200)
    
    try:
        node.get_logger().info('LIDAR Started...')
        for scan in lidar.iter_scans():
            node.publish_scan(scan)

    except Exception as e:
        node.get_logger().error(f"LIDAR Error: {e}")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("LIDAR disconnected.")
        rclpy.shutdown()

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        timer_period = 0.1  
        self.num_bins = 360
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def publish_scan(self, scan):
        msg = LaserScan()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        
        msg.angle_min = 0             
        msg.angle_max = 2.0 * math.pi  
        
        msg.angle_increment = (msg.angle_max - msg.angle_min) / self.num_bins
        
        msg.range_min = 0.10               
        msg.range_max = 15.0  

        msg.ranges = [float('inf')] * self.num_bins
        msg.intensities = [0.0] * self.num_bins  

        if scan is None:
            return
        
        for quality, angle_deg, distance_mm in scan:
            distance_m = distance_mm / 1000.0
            
            if distance_m < msg.range_min or distance_m > msg.range_max:
                continue
    
            index = int(round(angle_deg))
            
            index = index % self.num_bins
            
            msg.ranges[index] = distance_m
            msg.intensities[index] = float(quality)
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    lidar_publisher = LidarPublisher()

    worker_thread = threading.Thread(target=lidar_worker, args=(lidar_publisher,),  daemon=True)
    worker_thread.start()

    
    try:
        rclpy.spin(lidar_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            lidar_publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()