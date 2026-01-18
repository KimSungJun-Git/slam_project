#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import os

class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')
        self.obstacles = []
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.create_subscription(Float64MultiArray, '/lidar_custom', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom_custom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu_custom', self.imu_cb, 10)
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Master Controller: 장애물 데이터 수신 및 출력 시작')

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Master Controller: 시스템 정상 가동 중')
        
    def scan_cb(self, msg):
        self.obstacles = msg.data
        
    def odom_cb(self, msg): 
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
    
    def imu_cb(self, msg):
        q = msg.orientation
        sin_cosp = 2 * (q.w * q.z + q.x * q.y)
        cos_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.degrees(math.atan2(sin_cosp, cos_cosp))
        
    def control_loop(self):
        """
        msg = Twist()
        msg = Twist()
        is_obstacle_ahead = False
        num_obs = len(self.obstacles) // 3
        
        for i in range(num_obs):
            ox = self.obstacles[i*3]
            oy = self.obstacles[i*3 + 1]
            dist = math.sqrt(ox**2 + oy**2)
            
            if 0 < ox < 0.4 and abs(oy) < 0.3:
                is_obstacle_ahead = True
                break

        if is_obstacle_ahead:
            msg.linear.x = 0.0
            msg.angular.z = 0.5 
            self.get_logger().warn("장애물 감지! 회전 중...", throttle_duration_sec=1.0)
        else:
            msg.linear.x = 0.2
            msg.angular.z = 0.0
            self.get_logger().info("전방 클리어: 전진 중", throttle_duration_sec=1.0)
            
        self.pub_cmd.publish(msg)
        """
        msg = Twist()
    
        # 설정값 (로봇에 맞춰 조절 가능)
        base_speed = 0.2       # 기본 전진 속도
        min_speed = 0.05       # 장애물 감지 시 최소 전진 속도 (멈추지 않음)
        avoid_turn_speed = 0.6 # 회피 시 회전 강도
        detect_range = 0.6     # 장애물 감지 거리 (60cm)
        
        closest_dist = 999.9
        obstacle_y_pos = 0.0
        is_obstacle_detected = False
    
        # 1. 장애물 위치 파악
        num_obs = len(self.obstacles) // 3
        for i in range(num_obs):
            ox = self.obstacles[i*3]
            oy = self.obstacles[i*3 + 1]
            
            # 전방 감지 영역 (전방 0.6m, 좌우 0.3m 폭)
            if 0 < ox < detect_range and abs(oy) < 0.3:
                dist = ox
                if dist < closest_dist:
                    closest_dist = dist
                    obstacle_y_pos = oy # 장애물이 왼쪽(+)인지 오른쪽(-)인지 저장
                    is_obstacle_detected = True
    
        # 2. 회피 주행 로직
        if is_obstacle_detected:
            # 거리에 따라 전진 속도 조절 (가까울수록 느리게, 하지만 최소 min_speed 유지)
            msg.linear.x = max(min_speed, base_speed * (closest_dist / detect_range))
            
            # 장애물 위치의 반대 방향으로 회전
            # oy > 0 (장애물이 왼쪽) -> 우회전 (angular.z < 0)
            # oy < 0 (장애물이 오른쪽) -> 좌회전 (angular.z > 0)
            if obstacle_y_pos > 0:
                msg.angular.z = -avoid_turn_speed
            else:
                msg.angular.z = avoid_turn_speed
                
            self.get_logger().info(f"회피 중: 거리 {closest_dist:.2f}m, 조향 방향: {'우' if obstacle_y_pos > 0 else '좌'}")
        else:
            # 장애물 없으면 시원하게 직진
            msg.linear.x = base_speed
            msg.angular.z = 0.0
            self.get_logger().info("길이 열렸습니다: 직진 중")
    
        # 3. 명령 발행
        self.pub_cmd.publish(msg)

def main():
    rclpy.init()
    node = MasterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 노드가 종료됩니다. 로봇을 정지합니다...')
        pass
    finally:
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        node.pub_cmd.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()