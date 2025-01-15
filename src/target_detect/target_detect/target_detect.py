import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# 设置 QoS 为 BEST_EFFORT，与发布者一致
best_effort_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

class MultiRobotMapUpdater(Node):
    def __init__(self):
        super().__init__('multi_robot_map_updater')
        
        # 订阅两个机器人的激光雷达数据
        self.scan_sub_robot1 = self.create_subscription(
            LaserScan,
            '/tb3_0/scan',
            self.scan_callback_robot1,
            best_effort_qos
            )
        self.scan_sub_robot2 = self.create_subscription(
            LaserScan,
            '/tb3_1/scan',
            self.scan_callback_robot2,
            best_effort_qos
            )
        
        # 订阅合并地图
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/merge_map',
            self.map_callback,
            best_effort_qos
            )
        
        # 发布更新后的地图
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/target_map',
            best_effort_qos
            )
        
        self.current_map = None
        self.target_angle = 1  # 指定角度

    def map_callback(self, msg):
        #self.get_logger().info("Map data received.")
        self.current_map = msg
        

    def scan_callback_robot1(self, scan_msg):
        if self.current_map is None:
            self.get_logger().warn("Map not received yet.")
            return
        #self.get_logger().info(f"Received scan for robot1 at angle {self.target_angle}.")
        self.process_scan(scan_msg, robot_id="robot1")

    def scan_callback_robot2(self, scan_msg):
        if self.current_map is None:
            self.get_logger().warn("Map not received yet.")
            return
        #self.get_logger().info(f"Received scan for robot1 at angle {self.target_angle}.")
        self.process_scan(scan_msg, robot_id="robot2")

    def process_scan(self, scan_msg, robot_id):
        self.get_logger().info(f"Processing scan for {robot_id}.")
        # 计算指定角度上物体的位置
        angle_index = int((self.target_angle - scan_msg.angle_min) / scan_msg.angle_increment)
        self.get_logger().info(f" angle index:{angle_index}")
        self.get_logger().info(f"len angle: {len(scan_msg.ranges)}")
        if 0 <= angle_index < len(scan_msg.ranges):
            distance = scan_msg.ranges[angle_index]
            # 将激光雷达数据转换为地图坐标
            x = distance * math.cos(math.radians(self.target_angle))
            y = distance * math.sin(math.radians(self.target_angle))
            self.get_logger().info(f"Updating map")
            # 在地图上更新物体位置
            self.update_map(x, y, robot_id)

    def update_map(self, x, y, robot_id):
        self.get_logger().info(f"Updating map with position: ({x}, {y}), robot_id: {robot_id}")
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        index = grid_y * self.current_map.info.width + grid_x

        self.get_logger().info(f"Calculated grid position: ({grid_x}, {grid_y}), index: {index}")

        if 0 <= index < len(self.current_map.data):
            self.get_logger().info(f"Index {index} is valid, updating map.")
            
            # 创建新的地图数据
            updated_map = OccupancyGrid()
            updated_map.header = self.current_map.header
            updated_map.info = self.current_map.info
            updated_map.data = list(self.current_map.data)

            # 根据机器人ID标记数据
            if robot_id == "robot1":
                updated_map.data[index] = 100  # 机器人1标记
            elif robot_id == "robot2":
                updated_map.data[index] = 50   # 机器人2标记

            # 发布更新后的地图
            self.map_pub.publish(updated_map)
            self.get_logger().info("Updated map published.")
        else:
            self.get_logger().warn(f"Invalid index {index}, skipping map update.")

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
