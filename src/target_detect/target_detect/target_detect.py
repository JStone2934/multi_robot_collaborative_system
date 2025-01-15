import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import math

class MultiRobotMapUpdater(Node):
    def __init__(self):
        super().__init__('multi_robot_map_updater')
        
        # 订阅两个机器人的激光雷达数据
        self.scan_sub_robot1 = self.create_subscription(
            LaserScan,
            '/tb3_0/scan',
            self.scan_callback_robot1,
            10)
        self.scan_sub_robot2 = self.create_subscription(
            LaserScan,
            '/tb3_1/scan',
            self.scan_callback_robot2,
            10)
        
        # 订阅合并地图
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/merge_map',
            self.map_callback,
            10)
        
        # 发布更新后的地图
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/target_map',
            10)
        
        self.current_map = None
        self.target_angle = 45  # 指定角度

    def map_callback(self, msg):
        # 保存原始地图
        self.current_map = msg

    def scan_callback_robot1(self, scan_msg):
        if self.current_map is None:
            self.get_logger().warn("Map not received yet.")
            return
        self.process_scan(scan_msg, robot_id="robot1")

    def scan_callback_robot2(self, scan_msg):
        if self.current_map is None:
            self.get_logger().warn("Map not received yet.")
            return
        self.process_scan(scan_msg, robot_id="robot2")

    def process_scan(self, scan_msg, robot_id):
        # 计算指定角度上物体的位置
        angle_index = int((self.target_angle - scan_msg.angle_min) / scan_msg.angle_increment)
        if 0 <= angle_index < len(scan_msg.ranges):
            distance = scan_msg.ranges[angle_index]
            # 将激光雷达数据转换为地图坐标
            x = distance * math.cos(math.radians(self.target_angle))
            y = distance * math.sin(math.radians(self.target_angle))
            
            # 在地图上更新物体位置
            self.update_map(x, y, robot_id)

    def update_map(self, x, y, robot_id):
        # 获取地图的分辨率和原点
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        # 计算物体位置对应的栅格坐标
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        index = grid_y * self.current_map.info.width + grid_x

        if 0 <= index < len(self.current_map.data):
            # 标记物体位置（设置为障碍物）
            self.current_map.data[index] = 100  # 设置为障碍物
            
            # 可以根据机器人ID进行不同的标记或区分
            if robot_id == "robot1":
                self.current_map.data[index] = 100  # 例如标记为机器人1检测到的物体
            elif robot_id == "robot2":
                self.current_map.data[index] = 50  # 例如标记为机器人2检测到的物体（不同值）

            # 发布更新后的地图
            self.map_pub.publish(self.current_map)

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
