import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.cluster_pub = self.create_publisher(Float64MultiArray, '/lidar_custom', 10)

        self.get_logger().info('LidarNode started and logging ready')

    def cb(self, msg: LaserScan):
        # 1. Scan 데이터 필터링 (NaN 및 Inf 처리)
        ranges = np.array(msg.ranges)
        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max

        # 2. XY 포인트 변환 및 전방 필터링
        points = []
        for i, r in enumerate(ranges):
            if not (0.05 < r < msg.range_max):
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = (math.degrees(angle) + 360.0) % 360.0

            # 전방: 300 ~ 60도 (로봇 정면 기준)
            if 60.0 < angle_deg < 300.0:
                continue
            
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([x, y])
            
        if len(points) < 2:
            return

        points = np.array(points)

        # 3. 클러스터링
        jump_threshold = 0.5
        clusters = []
        current_cluster = [0]

        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i - 1])
            if dist < jump_threshold:
                current_cluster.append(i)
            else:
                clusters.append(current_cluster)
                current_cluster = [i]
        clusters.append(current_cluster)
        
        # 4. 장애물 데이터 계산 및 로그 메시지 준비
        obstacle_data = [] 
        log_msg = "\n" + "="*50 + "\n" # 초기화 필요!
        obstacle_count = 0             # 초기화 필요!

        for idx, cluster in enumerate(clusters):
            if len(cluster) < 3: 
                continue
            
            cluster_points = points[cluster]
            center = np.mean(cluster_points, axis=0)
            
            # 반지름 계산
            distances = np.linalg.norm(cluster_points - center, axis=1)
            radius = np.max(distances)
            
            # 데이터 추가 (타입 안정성을 위해 float() 사용)
            obstacle_data.extend([float(center[0]), float(center[1]), float(radius)])
            
            # 로그 조립 (들여쓰기 주의)
            obstacle_count += 1
            log_msg += f"[{obstacle_count}] 위치: ({center[0]:5.2f}, {center[1]:5.2f}) | 반경: {radius:4.2f}m\n"

        log_msg += "="*50

        # 5. 출력 및 퍼블리시
        if obstacle_count > 0:
            self.get_logger().info(log_msg)

        msg_out = Float64MultiArray()
        msg_out.data = obstacle_data
        self.cluster_pub.publish(msg_out)

def main():
    rclpy.init()
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()