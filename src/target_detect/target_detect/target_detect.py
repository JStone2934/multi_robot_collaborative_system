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
        self.detect_target = "beer"
        self.target_angle_robot1 = math.radians(0)  # 机器人1的目标角度
        self.target_angle_robot2 = math.radians(0)  # 机器人2的目标角度
        self.target_in_laser = False
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
        self.marker_pub_robot1 = self.create_publisher(Marker, '/map_markers_robot1', best_effort_qos)
        self.marker_pub_robot2 = self.create_publisher(Marker, '/map_markers_robot2', best_effort_qos)

        self.current_map = None
        self.marked_positions_robot1 = []
        self.marked_positions_robot2 = []

        # 机器人位姿
        self.robot1_pose = None
        self.robot2_pose = None

        # 机器人YOLO识别结果
        self.robot1_detection = None
        self.robot2_detection = None

        self.scan_msg_robot1 = None
        self.scan_time_robot1 = None
        self.scan_msg_robot2 = None
        self.scan_time_robot2 = None

        self.detection_msg_robot1 = None
        self.detection_time_robot1 = None
        self.detection_msg_robot2 = None
        self.detection_time_robot2 = None
        self.start_time_R1 = None
        self.start_time_R2 = None

    def map_callback(self, msg):
        self.current_map = msg

    def odom_callback_robot1(self, msg):
        self.robot1_pose = msg.pose.pose

    def odom_callback_robot2(self, msg):
        self.robot2_pose = msg.pose.pose

    def scan_callback_robot1(self, scan_msg):
        self.scan_msg_robot1 = scan_msg
        self.scan_time_robot1 = self.get_clock().now()  # 使用仿真时间
        self.try_process_robot1()

    def scan_callback_robot2(self, scan_msg):
        self.scan_msg_robot2 = scan_msg
        self.scan_time_robot2 = self.get_clock().now()  # 使用仿真时间
        self.try_process_robot2()

    def detection_callback_robot1(self, cam_detection):
        self.detection_msg_robot1 = cam_detection
        self.detection_time_robot1 = self.get_clock().now()  # 使用仿真时间
        self.try_process_robot1()

    def detection_callback_robot2(self, cam_detection):
        self.detection_msg_robot2 = cam_detection
        self.detection_time_robot2 = self.get_clock().now()  # 使用仿真时间
        self.try_process_robot2()

    def try_process_robot1(self):
        if self.scan_msg_robot1 is not None and self.detection_msg_robot1 is not None:
            # 获取 scan 和 detection 的时间戳（仿真时间）
            scan_time_sec = self.scan_time_robot1.nanoseconds / 1e9
            detection_time_sec = self.detection_time_robot1.nanoseconds / 1e9

            # 计算时间差（秒）
            time_diff = abs(detection_time_sec - scan_time_sec)
            self.get_logger().warn(f"Robot1 Time Diff: {time_diff} seconds")  # 输出时间差调试信息
            
            if time_diff < 0.1:  # 允许的时间差（0.1秒以内）
                self.process_robot1()

    def try_process_robot2(self):
        if self.scan_msg_robot2 is not None and self.detection_msg_robot2 is not None:
            # 获取 scan 和 detection 的时间戳（仿真时间）
            scan_time_sec = self.scan_time_robot2.nanoseconds / 1e9
            detection_time_sec = self.detection_time_robot2.nanoseconds / 1e9
            # 计算时间差（秒）
            time_diff = abs(detection_time_sec - scan_time_sec)
            self.get_logger().warn(f"Robot2 Time Diff: {time_diff} seconds")  # 输出时间差调试信息
            
            if time_diff < 0.1:  # 允许的时间差（0.1秒以内）
                self.process_robot2()


    def process_robot1(self):
        self.get_logger().warn("NR2")
        scan_msg = self.scan_msg_robot1
        cam_detection = self.detection_msg_robot1
        if scan_msg is None and cam_detection is None:
            self.get_logger().warn("No detection from R1")
            return
        self.robot1_detection = cam_detection
        highest_confidence = 0.0
        best_detection = None
        for detection in cam_detection.detections:
            if detection.results[0].hypothesis.class_id == "beer":
                object_x = detection.bbox.center.x
                object_y = detection.bbox.center.y
                confidence = detection.results[0].hypothesis.score
                if confidence >= 0.90 and confidence > highest_confidence:
                    highest_confidence = confidence
                    best_detection = (object_x, object_y, confidence)
        if best_detection:
            self.target_angle_robot1 = self.camera_to_lidar_angle(object_x)
            self.process_scan(scan_msg, cam_detection, robot_id="robot1", robot_pose=self.robot1_pose, target_angle=self.target_angle_robot1)
        
        # 清空缓存
        self.scan_msg_robot1 = None
        self.detection_msg_robot1 = None

    def process_robot2(self):
        self.get_logger().warn("NR2")
        scan_msg = self.scan_msg_robot2
        cam_detection = self.detection_msg_robot2
        if scan_msg is None and cam_detection is None:
            self.get_logger().warn("No detection from R2")
            return
        self.robot2_detection = cam_detection
        highest_confidence = 0.0
        best_detection = None
        for detection in cam_detection.detections:
            if detection.results[0].hypothesis.class_id == "beer":
                object_x = detection.bbox.center.x
                object_y = detection.bbox.center.y
                confidence = detection.results[0].hypothesis.score
                if confidence >= 0.90 and confidence > highest_confidence:
                    highest_confidence = confidence
                    best_detection = (object_x, object_y, confidence)
        if best_detection:
            self.target_angle_robot2 = self.camera_to_lidar_angle(object_x)
            self.process_scan(scan_msg, cam_detection, robot_id="robot2", robot_pose=self.robot2_pose, target_angle=self.target_angle_robot2)
        
        # 清空缓存
        self.scan_msg_robot2 = None
        self.detection_msg_robot2 = None

    def camera_to_lidar_angle(self, object_x, fov_camera=1.02974, resolution_width=1920):
        # 计算每个像素的视角宽度（弧度）
        fov_per_pixel = fov_camera / resolution_width
        # 计算图像中心位置
        image_center = resolution_width / 2
        # 计算激光雷达的角度（弧度）
        angle = (object_x - image_center) * fov_per_pixel
        if angle < 0:
            laser_angle = 6.280000/2 + angle
        else:
            laser_angle = angle
        return laser_angle

    # 更新地图并更新标记
    def process_scan(self, scan_msg, cam_detection, robot_id, robot_pose, target_angle):
        
        angle_index = int((target_angle - scan_msg.angle_min) / scan_msg.angle_increment)
        if 0 <= angle_index < len(scan_msg.ranges):
            distance = scan_msg.ranges[angle_index]
            if distance != float('inf') and distance < scan_msg.range_max:
                self.target_in_laser = True
            else:
                return
            local_x = distance * math.cos(math.radians(target_angle))
            local_y = distance * math.sin(math.radians(target_angle))
            
            robot_x = robot_pose.position.x
            robot_y = robot_pose.position.y
            robot_theta = euler_from_quaternion([
                robot_pose.orientation.x,
                robot_pose.orientation.y,
                robot_pose.orientation.z,
                robot_pose.orientation.w
            ])[2]
            
            global_x = robot_x + local_x * math.cos(robot_theta) - local_y * math.sin(robot_theta)
            global_y = robot_y + local_x * math.sin(robot_theta) + local_y * math.cos(robot_theta)
            
            self.update_map(global_x, global_y, robot_id, cam_detection)
    
    # 更新地图
    def update_map(self, x, y, robot_id, cam_detection):
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        index = grid_y * self.current_map.info.width + grid_x

        if 0 <= index < len(self.current_map.data):
            updated_map = OccupancyGrid()
            updated_map.header = self.current_map.header
            updated_map.header.frame_id = 'merge_map'
            updated_map.info = self.current_map.info
            updated_map.data = list(self.current_map.data)

            if robot_id == "robot1":
                updated_map.data[index] = 100
            elif robot_id == "robot2":
                updated_map.data[index] = 50

            if cam_detection is not None:
                if self.target_in_laser == True:
                    for detection in cam_detection.detections:
                        if detection.results[0].hypothesis.class_id == self.detect_target:
                            if robot_id == "robot1" and (grid_x, grid_y) not in self.marked_positions_robot1:
                                self.marked_positions_robot1.append((grid_x, grid_y))
                                cam_detection = None
                                self.target_in_laser = False
                            elif robot_id == "robot2" and (grid_x, grid_y) not in self.marked_positions_robot2:
                                self.marked_positions_robot2.append((grid_x, grid_y))
                                cam_detection = None
                                self.target_in_laser = False

            self.map_pub.publish(updated_map)
            self.publish_markers()

    # 发布标记
    def publish_markers(self):
        marker_robot1 = Marker()
        marker_robot1.header.frame_id = 'merge_map'
        marker_robot1.header.stamp = self.get_clock().now().to_msg()
        marker_robot1.ns = 'robot1_markers'
        marker_robot1.id = 0
        marker_robot1.type = Marker.POINTS
        marker_robot1.action = Marker.ADD
        marker_robot1.pose.orientation.w = 1.0
        marker_robot1.scale.x = 0.1
        marker_robot1.scale.y = 0.1
        marker_robot1.color.r = 1.0
        marker_robot1.color.g = 0.0
        marker_robot1.color.b = 0.0
        marker_robot1.color.a = 1.0

        marker_robot2 = Marker()
        marker_robot2.header.frame_id = 'merge_map'
        marker_robot2.header.stamp = self.get_clock().now().to_msg()
        marker_robot2.ns = 'robot2_markers'
        marker_robot2.id = 0
        marker_robot2.type = Marker.POINTS
        marker_robot2.action = Marker.ADD
        marker_robot2.pose.orientation.w = 1.0
        marker_robot2.scale.x = 0.1
        marker_robot2.scale.y = 0.1
        marker_robot2.color.r = 0.0
        marker_robot2.color.g = 1.0
        marker_robot2.color.b = 0.0
        marker_robot2.color.a = 1.0

        for pos in self.marked_positions_robot1:
            point = Point()
            point.x = pos[0] * self.current_map.info.resolution + self.current_map.info.origin.position.x
            point.y = pos[1] * self.current_map.info.resolution + self.current_map.info.origin.position.y
            marker_robot1.points.append(point)

        for pos in self.marked_positions_robot2:
            point = Point()
            point.x = pos[0] * self.current_map.info.resolution + self.current_map.info.origin.position.x
            point.y = pos[1] * self.current_map.info.resolution + self.current_map.info.origin.position.y
            marker_robot2.points.append(point)

        self.marker_pub_robot1.publish(marker_robot1)
        self.marker_pub_robot2.publish(marker_robot2)



def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
