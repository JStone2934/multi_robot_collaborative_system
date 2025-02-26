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
from collections import deque
from .marker_fix import fix_points
TIME_DIFF = 0.02 #激光雷达与视觉识别的同步
# 设置 QoS 为 BEST_EFFORT，与发布者一致
best_effort_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
class MultiRobotMapUpdater(Node):
    def __init__(self):
        super().__init__('multi_robot_map_updater')

        self.map_pub = self.create_publisher(OccupancyGrid, 'updated_map', best_effort_qos)
        self.detect_target = "beer"
        self.target_angle_robot1 = None
        self.target_angle_robot2 = None
        self.target_in_laser = False
        # 初始化 current_map
        self.current_map = OccupancyGrid()
        # 队列缓存，最大缓存长度为100条
        self.scan_queue_robot1 = deque(maxlen=100)
        self.scan_queue_robot2 = deque(maxlen=100)
        self.detection_queue_robot1 = deque(maxlen=100)
        self.detection_queue_robot2 = deque(maxlen=100)
        self.odom_queue_robot1 = deque(maxlen=100)
        self.odom_queue_robot2 = deque(maxlen=100)
        self.marked_positions_robot1 = []
        self.marked_positions_robot2 = []
        # 添加scan_msg和detection_msg初始化
        self.scan_msg_robot1 = None
        self.scan_msg_robot2 = None
        self.detection_msg_robot1 = None
        self.detection_msg_robot2 = None

        self.robot1_pose = None
        self.robot2_pose = None

        self.fix_positions = []

        # 发布者，用于发布机器人1的标记点
        self.marker_pub_robot1 = self.create_publisher(Marker, 'map_markers_robot1', best_effort_qos)

        # 发布者，用于发布机器人2的标记点
        self.marker_pub_robot2 = self.create_publisher(Marker, 'map_markers_robot2', best_effort_qos)

        # 发布者，用于发布修正后的标记点
        self.marker_pub_fix = self.create_publisher(Marker, 'map_markers_fix', best_effort_qos)

        # 订阅数据
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
        self.detection_sub_robot1 = self.create_subscription(
            Detection2DArray,
            '/yolo_result0',
            self.detection_callback_robot1,
            best_effort_qos
        )
        self.detection_sub_robot2 = self.create_subscription(
            Detection2DArray,
            '/yolo_result1',
            self.detection_callback_robot2,
            best_effort_qos
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/merge_map',  # 订阅地图话题
            self.map_callback,
            best_effort_qos
        )
    def scan_callback_robot1(self, scan_msg):
        self.scan_queue_robot1.append(scan_msg)
        self.scan_msg_robot1 = scan_msg  # 保存 scan_msg_robot1
        self.try_process_robot1()

    def scan_callback_robot2(self, scan_msg):
        self.scan_queue_robot2.append(scan_msg)
        self.scan_msg_robot2 = scan_msg  # 保存 scan_msg_robot2
        self.try_process_robot2()

    def odom_callback_robot1(self, msg):
        self.odom_queue_robot1.append(msg)
        self.robot1_pose = msg.pose.pose  # Save the pose data for robot 1
        self.try_process_robot1()

    def odom_callback_robot2(self, msg):
        self.odom_queue_robot2.append(msg)
        self.robot2_pose = msg.pose.pose  # Save the pose data for robot 2
        self.try_process_robot2()

    def detection_callback_robot1(self, cam_detection):
        self.detection_queue_robot1.append(cam_detection)
        self.detection_msg_robot1 = cam_detection  # 保存 detection_msg_robot1
        self.try_process_robot1()

    def detection_callback_robot2(self, cam_detection):
        self.detection_queue_robot2.append(cam_detection)
        self.detection_msg_robot2 = cam_detection  # 保存 detection_msg_robot2
        self.try_process_robot2()
    def map_callback(self, map_msg):
        # 存储接收到的地图数据
        self.current_map = map_msg
    def try_process_robot1(self):
        if not self.scan_queue_robot1 or not self.detection_queue_robot1 or not self.odom_queue_robot1:
            return  # 缓存不完整，无法同步
        
        # 获取YOLO识别数据
        cam_detection = self.detection_msg_robot1  # 使用已保存的detection_msg_robot1
        if not cam_detection:
            return  # 如果没有YOLO数据，跳过
        
        # 获取与YOLO数据时间戳接近的位姿数据
        odom_msg = self.get_closest_message_within_time(cam_detection.header.stamp, self.odom_queue_robot1)
        
        # 获取与YOLO数据时间戳接近的激光雷达数据
        scan_msg = self.get_closest_message_within_time(cam_detection.header.stamp, self.scan_queue_robot1)

        if scan_msg and cam_detection and odom_msg:
            self.process_robot1(scan_msg, cam_detection, odom_msg)

    def try_process_robot2(self):
        if not self.scan_queue_robot2 or not self.detection_queue_robot2 or not self.odom_queue_robot2:
            return  # 缓存不完整，无法同步
        
        # 获取YOLO识别数据
        cam_detection = self.detection_msg_robot2  # 使用已保存的detection_msg_robot2
        if not cam_detection:
            return  # 如果没有YOLO数据，跳过
        
        # 获取与YOLO数据时间戳接近的位姿数据
        odom_msg = self.get_closest_message_within_time(cam_detection.header.stamp, self.odom_queue_robot2)
        
        # 获取与YOLO数据时间戳接近的激光雷达数据
        scan_msg = self.get_closest_message_within_time(cam_detection.header.stamp, self.scan_queue_robot2)

        if scan_msg and cam_detection and odom_msg:
            self.process_robot2(scan_msg, cam_detection, odom_msg)

    def get_closest_message_within_time(self, detection_stamp, message_queue, max_time_diff=TIME_DIFF):
        """
        获取与检测数据时间戳（detection_stamp）相差小于 max_time_diff 的消息。
        
        :param detection_stamp: YOLO检测数据的时间戳
        :param message_queue: 消息队列（可以是位姿数据队列或激光雷达数据队列）
        :param max_time_diff: 最大时间差阈值，单位为秒
        :return: 时间戳接近检测数据的消息，若无符合条件的消息返回 None
        """
        if len(message_queue) == 0:
            return None

        closest_msg = None
        min_time_diff = float('inf')

        for msg in message_queue:
            # 获取消息的时间戳（秒）
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            # 获取YOLO检测数据的时间戳（秒）
            detection_time = detection_stamp.sec + detection_stamp.nanosec / 1e9
            
            # 计算时间差
            time_diff = abs(msg_time - detection_time)
            #print(f"Message Time: {msg_time}, Detection Time: {detection_time}, Time Diff: {time_diff}")

            # 如果时间差小于最大时间差，认为符合条件
            if time_diff <= max_time_diff:
                # 如果这是第一次符合条件的消息，或者当前消息比已找到的更接近检测数据的时间
                if closest_msg is None or time_diff < min_time_diff:
                    closest_msg = msg
                    min_time_diff = time_diff

        return closest_msg


    def process_robot1(self, scan_msg, cam_detection, odom_msg):
        #self.get_logger().warn("NR2")
        #scan_msg = self.scan_msg_robot1
        #cam_detection = self.detection_msg_robot1
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
            print(self.target_angle_robot1)
            self.process_scan(scan_msg, cam_detection, robot_id="robot1", robot_pose=self.robot1_pose, target_angle=self.target_angle_robot1)
        
        # 清空缓存
        self.scan_msg_robot1 = None
        self.detection_msg_robot1 = None

    def process_robot2(self, scan_msg, cam_detection, odom_msg):
        #self.get_logger().warn("NR2")
        #scan_msg = self.scan_msg_robot2
        #cam_detection = self.detection_msg_robot2
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
            print(self.target_angle_robot2)
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
            laser_angle = 6.280000 + angle
        else:
            laser_angle = angle
        return laser_angle

    # 更新地图并更新标记
    def process_scan(self, scan_msg, cam_detection, robot_id, robot_pose, target_angle):
        try:
            print("process")
            angle_index = int((target_angle - scan_msg.angle_min) / scan_msg.angle_increment)
            if 0 <= angle_index < len(scan_msg.ranges):
                distance = scan_msg.ranges[angle_index]
                if distance != float('inf') and distance < 2.8:
                    print(distance)
                    self.target_in_laser = True
                else:
                    self.target_in_laser = False
                    return
                local_x = distance * math.cos(target_angle)
                local_y = distance * math.sin(target_angle)
                
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
        except:
            print("process scan ERROR")
            pass
    
    # 更新地图
    def update_map(self, x, y, robot_id, cam_detection):
        # 直接使用机器人检测到的目标位置（在全局坐标系下）
        # 不需要将标记点转换为网格坐标（grid_x, grid_y）
        if cam_detection is not None and self.target_in_laser:
            for detection in cam_detection.detections:
                if detection.results[0].hypothesis.class_id == self.detect_target:
                    # 在目标首次检测到时，将标记点位置添加到已存储的列表
                    if robot_id == "robot1" and (x, y) not in self.marked_positions_robot1:
                        self.marked_positions_robot1.append((x, y))
                        cam_detection = None
                        self.target_in_laser = False
                    elif robot_id == "robot2" and (x, y) not in self.marked_positions_robot2:
                        self.marked_positions_robot2.append((x, y))
                        cam_detection = None
                        self.target_in_laser = False
        merge_points = self.marked_positions_robot1+self.marked_positions_robot2
        self.fix_positions = fix_points(merge_points)
        # 发布更新后的地图
        self.map_pub.publish(self.current_map)
        self.publish_markers()  # 只根据标记点的位置发布

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
        marker_robot1.color.a = 0.6
        marker_robot1.frame_locked = False

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
        marker_robot2.color.a = 0.6
        marker_robot2.frame_locked = False

        marker_fix = Marker()
        marker_fix.header.frame_id = 'merge_map'
        marker_fix.header.stamp = self.get_clock().now().to_msg()
        marker_fix.ns = 'robot2_markers'
        marker_fix.id = 0
        marker_fix.type = Marker.POINTS
        marker_fix.action = Marker.ADD
        marker_fix.pose.orientation.w = 1.0
        marker_fix.scale.x = 0.3
        marker_fix.scale.y = 0.3
        marker_fix.color.r = 1.0
        marker_fix.color.g = 1.0
        marker_fix.color.b = 0.0
        marker_fix.color.a = 1.0
        marker_fix.frame_locked = False

        # 发布标记点，使用全局坐标而不是基于地图原点的坐标
        for pos in self.marked_positions_robot1:
            point = Point()
            point.x = pos[0]  # 使用存储的全局坐标
            point.y = pos[1]  # 使用存储的全局坐标
            marker_robot1.points.append(point)

        for pos in self.marked_positions_robot2:
            point = Point()
            point.x = pos[0]  # 使用存储的全局坐标
            point.y = pos[1]  # 使用存储的全局坐标
            marker_robot2.points.append(point)

        for pos in self.fix_positions:
            point = Point()
            point.x = pos[0]  # 使用存储的全局坐标
            point.y = pos[1]  # 使用存储的全局坐标
            marker_fix.points.append(point)

        # 发布标记点
        self.marker_pub_robot1.publish(marker_robot1)
        self.marker_pub_robot2.publish(marker_robot2)
        self.marker_pub_fix.publish(marker_fix)




def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
