import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# 设置 QoS 为 BEST_EFFORT，与发布者一致
best_effort_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

class TestYolo(Node):
    def __init__(self):
        super().__init__('test_olo')

        # 创建发布者，用于发布识别到的 "cone" 的坐标
        self.cone_publisher = self.create_publisher(Point, 'detected_cone_position', best_effort_qos)

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

    def detection_callback_robot1(self, detection_msg):
        self.handle_detection(detection_msg, robot_id="robot1")

    def detection_callback_robot2(self, detection_msg):
        self.handle_detection(detection_msg, robot_id="robot2")

    def handle_detection(self, detection_msg, robot_id):
        highest_confidence = 0.0
        best_detection = None

        # 遍历所有检测到的目标
        for detection in detection_msg.detections:
            if detection.results[0].hypothesis.class_id == "beer":  # 检查物体标签是否为 "cone"
                # 获取目标物体的坐标 (bbox.center.x 和 bbox.center.y)
                object_x = detection.bbox.center.x
                object_y = detection.bbox.center.y

                # 获取当前目标的置信度
                confidence = detection.results[0].hypothesis.score

                # 如果置信度大于0.90并且是当前最高的置信度，则更新
                if confidence >= 0.90 and confidence > highest_confidence:
                    highest_confidence = confidence
                    best_detection = (object_x, object_y, confidence)

        # 如果找到了满足条件的目标，打印结果并发布位置
        if best_detection:
            object_x, object_y, confidence = best_detection
            self.get_logger().info(f"Detected cone at ({object_x}, {object_y}) by {robot_id} with highest confidence {confidence:.2f}")

            # 创建并发布一个 Point 消息，表示 "cone" 的位置
            cone_position = Point()
            cone_position.x = object_x
            cone_position.y = object_y
            cone_position.z = 0.0  # 可以设置为0，表示平面坐标

            # 发布识别到的 "cone" 的位置
            self.cone_publisher.publish(cone_position)
            self.get_logger().info(f"Published cone position: ({object_x}, {object_y}) with confidence {confidence:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = TestYolo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
