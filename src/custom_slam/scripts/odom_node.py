#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy # QoS 설정 추가
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        
        # [수정] QoS 프로파일 설정: 원본 데이터가 'Best Effort'일 경우를 대비
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=10)

        # qos_profile을 적용하여 구독
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, qos_profile)
            
        self.pub = self.create_publisher(Odometry, '/odom_custom', 10)
        
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.alpha = 0.8
        
        #self.get_logger().info('OdomNode: QoS 설정 완료 및 데이터 대기 중...')

    def cb(self, msg):
        # 데이터가 들어오면 무조건 터미널에 수신 알림을 찍음
        # self.get_logger().info("데이터 수신됨!") 

        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        
        filtered_x = (self.alpha * raw_x) + ((1.0 - self.alpha) * self.prev_x)
        filtered_y = (self.alpha * raw_y) + ((1.0 - self.alpha) * self.prev_y)
        
        self.prev_x = filtered_x
        self.prev_y = filtered_y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        linear_vel = msg.twist.twist.linear.x

        # 1초마다 출력 (확인을 위해 주기를 당김)
        #self.get_logger().info(
        #    f"Pos:({filtered_x:.2f}, {filtered_y:.2f}) | Yaw:{yaw:6.1f}° | Vel:{linear_vel:.2e}", 
        #    throttle_duration_sec=1.0
        #)

        msg.pose.pose.position.x = filtered_x
        msg.pose.pose.position.y = filtered_y
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()