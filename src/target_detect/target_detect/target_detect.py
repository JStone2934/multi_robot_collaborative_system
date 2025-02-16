import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf_transformations import euler_from_quaternion  
import tf2_ros
from vision_msgs.msg import Detection2DArray, Detection2D
# 设置 QoS 为 BEST_EFFORT，与发布者一致
best_effort_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
class MultiRobotMapUpdater(Node):
    def __init__(self):
        super().__init__('multi_robot_map_updater')
        
        # 为两个机器人分别设置不同的目标角度
        self.target_angle_robot1 = math.radians(0)   # 机器人1的目标角度
        self.target_angle_robot2 = math.radians(0)   # 机器人2的目标角度
        
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
        
        # 订阅两个机器人的位姿（Odometry）
        self.odom_sub_robot1 = self.create_subscription(
            Odometry,
            '/tb3_0/odom',
            self.odom_callback_robot1,
            best_effort_qos
            )
        self.odom_sub_robot2 = self.create_subscription(
            Odometry,
            '/tb3_1/odom',
            self.odom_callback_robot2,
            best_effort_qos
            )
        
        # 订阅每个机器人的目标物体（识别结果）
        self.detection_sub_robot1 = self.create_subscription(
            Detection2DArray,
            '/yolo_result0',  # 机器人1检测到的物体
            self.detection_callback_robot1,
            best_effort_qos
        )
        self.detection_sub_robot2 = self.create_subscription(
            Detection2DArray,
            '/yolo_result1',  # 机器人2检测到的物体
            self.detection_callback_robot2,
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
        
        # 发布标记消息
        self.marker_pub = self.create_publisher(Marker, '/map_markers', best_effort_qos)

        self.current_map = None
        # 存储已标记的物体位置
        self.marked_positions = []

        # 机器人位姿
        self.robot1_pose = None
        self.robot2_pose = None

        #机器人yolo识别结果
        self.robot1_detection = None
        self.robot2_detection = None

    def map_callback(self, msg):
        self.current_map = msg

    def odom_callback_robot1(self, msg):
        # 获取第一个机器人的位姿
        self.robot1_pose = msg.pose.pose
        #self.get_logger().info(f"Received Odometry for robot1: {self.robot1_pose}")

    def odom_callback_robot2(self, msg):
        # 获取第二个机器人的位姿
        self.robot2_pose = msg.pose.pose
        #self.get_logger().info(f"Received Odometry for robot2: {self.robot2_pose}")

    def scan_callback_robot1(self, scan_msg):
        if self.current_map is None or self.robot1_pose is None:
            self.get_logger().warn("Map or Odometry not received yet.")
            return
        self.process_scan(scan_msg, self.robot1_detection,robot_id="robot1", robot_pose=self.robot1_pose, target_angle=self.target_angle_robot1)

    def scan_callback_robot2(self, scan_msg):
        if self.current_map is None or self.robot2_pose is None:
            self.get_logger().warn("Map or Odometry not received yet.")
            return
        self.process_scan(scan_msg, self.robot2_detection,robot_id="robot2", robot_pose=self.robot2_pose, target_angle=self.target_angle_robot2)

    def detection_callback_robot1(self, cam_detection):
        if cam_detection is None:
            self.get_logger().warn("No detection from R1")
            return
        self.robot1_detection = cam_detection
        # 获取目标物体的坐标 (bbox.center.x 和 bbox.center.y)
        for detection in cam_detection.detections:
            if detection.results[0].hypothesis.class_id == "cone":  # 检查物体标签是否为 "cone"
                # 获取目标物体的坐标 (bbox.center.x 和 bbox.center.y)
                object_x = detection.bbox.center.x
        self.target_angle_robot1 = self.camera_to_lidar_angle(object_x)

    def detection_callback_robot2(self, cam_detection):
        if cam_detection is None:
            self.get_logger().warn("No detection from R2")
            return
        self.robot2_detection = cam_detection
        for detection in cam_detection.detections:
            if detection.results[0].hypothesis.class_id == "cone":  # 检查物体标签是否为 "cone"
                # 获取目标物体的坐标 (bbox.center.x 和 bbox.center.y)
                object_x = detection.bbox.center.x
        self.target_angle_robot2 = self.camera_to_lidar_angle(object_x)

    def camera_to_lidar_angle(self, object_x, fov_camera=1.02974, resolution_width=1920):
        # 计算每个像素的视角宽度（弧度）
        fov_per_pixel = fov_camera / resolution_width
        # 计算图像中心位置
        image_center = resolution_width / 2
        # 计算激光雷达的角度（弧度）
        angle = (object_x - image_center) * fov_per_pixel
        if angle < 0:
            laser_angle = 6.280000/2 + angle
        else :
            laser_angle = angle
        return laser_angle

    def process_scan(self, scan_msg, cam_detection, robot_id, robot_pose, target_angle):
        # 计算指定角度上物体的位置
        angle_index = int((target_angle - scan_msg.angle_min) / scan_msg.angle_increment)
        #self.get_logger().info(f"Angle min: {scan_msg.angle_min}")
        #self.get_logger().warn(f"process_scan")
        if 0 <= angle_index < len(scan_msg.ranges):
            distance = scan_msg.ranges[angle_index]
            
            # 过滤掉无效距离值
            if distance == float('inf') or distance > scan_msg.range_max:
                return
            
            # 将激光雷达数据转换为局部坐标
            local_x = distance * math.cos(math.radians(target_angle))
            local_y = distance * math.sin(math.radians(target_angle))
            
            # 转换为全局坐标
            robot_x = robot_pose.position.x
            robot_y = robot_pose.position.y
            robot_theta = euler_from_quaternion([
                robot_pose.orientation.x,
                robot_pose.orientation.y,
                robot_pose.orientation.z,
                robot_pose.orientation.w
            ])[2]  # 偏航角
            
            global_x = robot_x + local_x * math.cos(robot_theta) - local_y * math.sin(robot_theta)
            global_y = robot_y + local_x * math.sin(robot_theta) + local_y * math.cos(robot_theta)
            
            # 更新地图
            self.update_map(global_x, global_y, robot_id, cam_detection)
            
    def update_map(self, x, y, robot_id, cam_detection):
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        index = grid_y * self.current_map.info.width + grid_x

        if 0 <= index < len(self.current_map.data):
            # 创建新的地图数据
            updated_map = OccupancyGrid()
            updated_map.header = self.current_map.header
            updated_map.header.frame_id = 'merge_map'
            updated_map.info = self.current_map.info
            updated_map.data = list(self.current_map.data)

            # 根据机器人ID标记数据
            if robot_id == "robot1":
                updated_map.data[index] = 100  # 机器人1标记
            elif robot_id == "robot2":
                updated_map.data[index] = 50   # 机器人2标记
            if cam_detection is None:
                pass
            else:
                for detection in cam_detection.detections:
                    if detection.results[0].hypothesis.class_id == "cone":  # 检查物体标签是否为 "cone"
                        # 如果这个位置已经被标记过，保留原标记
                        if (grid_x, grid_y) not in self.marked_positions:
                            self.marked_positions.append((grid_x, grid_y))
                            self.get_logger().warn(f"marked_positions {grid_x},{grid_y}")
                        self.robot1_detection = None
                        self.robot2_detection = None

            # 发布更新后的地图
            self.map_pub.publish(updated_map)
            
            # 发布标记信息
            self.publish_markers()

        else:
            self.get_logger().warn(f"Invalid index {index}, skipping map update.")
    
    def publish_markers(self):
        # 创建标记消息
        marker = Marker()
        marker.header.frame_id = 'merge_map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_markers'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # 点的大小
        marker.scale.y = 0.1
        marker.color.r = 1.0  # 红色
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for pos in self.marked_positions:
            point = Point()
            point.x = pos[0] * self.current_map.info.resolution + self.current_map.info.origin.position.x
            point.y = pos[1] * self.current_map.info.resolution + self.current_map.info.origin.position.y
            marker.points.append(point)

        # 发布标记
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
