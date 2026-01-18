#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.sub = self.create_subscription(Imu, '/imu', self.cb, 10)
        self.pub = self.create_publisher(Imu, '/imu_custom', 10)
        
        self.prev_accel_z = 0.0
        self.alpha = 0.2 # 필터링
        
    def cb(self, msg):
        #노이즈 제거
        filtered_accel_z = (self.alpha*msg.linear_acceleration.z) + ((1.0 - self.alpha) * self.prev_accel_z)
        self.prev_accel_z = filtered_accel_z
        
        q = msg.orientation
        sin_cosp = 2* (q.w * q.z + q.x * q.y)
        cos_cosp = 1 - 2*(q.y * q.y + q.z * q.z)
        yaw = math.atan2(sin_cosp, cos_cosp)
        yaw_deg = math.degrees(yaw)
        
        # 2초마다 현재 바라보는 각도 출력
        #self.get_logger().info(f'현재 로봇 방향(Yaw): {yaw_deg:.2f}도', throttle_duration_sec=2.0)
        
        new_msg = msg
        new_msg.linear_acceleration.z = filtered_accel_z
        self.pub.publish(new_msg)
        
def main():
    rclpy.init()
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()