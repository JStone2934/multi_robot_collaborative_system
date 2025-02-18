from math import frexp
from traceback import print_tb
from torch import imag
from yolov5 import YOLOv5
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose, Detection2D
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
from yolov5_ros2.cv_tool import px2xy
import os

# Get the ROS distribution version and set the shared directory for YoloV5 configuration files.
ros_distribution = os.environ.get("ROS_DISTRO")
package_share_directory = get_package_share_directory('yolov5_ros2')

# Create a ROS 2 Node class YoloV5Ros2.
class YoloV5Ros2(Node):
    def __init__(self):
        super().__init__('yolov5_ros2')
        self.get_logger().info(f"Current ROS 2 distribution: {ros_distribution}")

        # Declare ROS parameters.
        self.declare_parameter("device", "cuda", ParameterDescriptor(
            name="device", description="Compute device selection, default: cpu, options: cuda:0"))

        self.declare_parameter("model", "gazebo_beer", ParameterDescriptor(
            name="model", description="Default model selection: cone"))

        self.declare_parameter("image_topic_0", "/tb3_0/camera/image_raw", ParameterDescriptor(
            name="image_topic_0", description="Image topic for tb3_0, default: /tb3_0/camera/image_raw"))

        self.declare_parameter("image_topic_1", "/tb3_1/camera/image_raw", ParameterDescriptor(
            name="image_topic_1", description="Image topic for tb3_1, default: /tb3_1/camera/image_raw"))

        self.declare_parameter("result_img_topic_0", "/result_img0", ParameterDescriptor(
            name="result_img_topic_0", description="Result image topic for tb3_0, default: /result_img0"))

        self.declare_parameter("result_img_topic_1", "/result_img1", ParameterDescriptor(
            name="result_img_topic_1", description="Result image topic for tb3_1, default: /result_img1"))

        self.declare_parameter("yolo_result_topic_0", "/yolo_result0", ParameterDescriptor(
            name="yolo_result_topic_0", description="YOLO result topic for tb3_0, default: /yolo_result0"))

        self.declare_parameter("yolo_result_topic_1", "/yolo_result1", ParameterDescriptor(
            name="yolo_result_topic_1", description="YOLO result topic for tb3_1, default: /yolo_result1"))

        # Default to displaying detection results.
        self.declare_parameter("show_result", False, ParameterDescriptor(
            name="show_result", description="Whether to display detection results, default: False"))

        self.declare_parameter("pub_result_img", True, ParameterDescriptor(
            name="pub_result_img", description="Whether to publish detection result images, default: False"))

        # 1. Load the model.
        model_path = package_share_directory + "/config/" + self.get_parameter('model').value + ".pt"
        device = self.get_parameter('device').value
        self.yolov5 = YOLOv5(model_path=model_path, device=device)

        # 2. Create publishers for tb3_0.
        self.yolo_result_pub_0 = self.create_publisher(
            Detection2DArray, self.get_parameter('yolo_result_topic_0').value, 10)
        self.result_img_pub_0 = self.create_publisher(
            Image, self.get_parameter('result_img_topic_0').value, 10)

        # 2. Create publishers for tb3_1.
        self.yolo_result_pub_1 = self.create_publisher(
            Detection2DArray, self.get_parameter('yolo_result_topic_1').value, 10)
        self.result_img_pub_1 = self.create_publisher(
            Image, self.get_parameter('result_img_topic_1').value, 10)

        # 3. Create image subscribers.
        qos_profile = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
        self.image_sub_0 = self.create_subscription(
            Image, self.get_parameter('image_topic_0').value, lambda msg: self.image_callback(msg, 0), qos_profile)
        self.image_sub_1 = self.create_subscription(
            Image, self.get_parameter('image_topic_1').value, lambda msg: self.image_callback(msg, 1), qos_profile)

        # 4. Image format conversion (using cv_bridge).
        self.bridge = CvBridge()
        self.show_result = self.get_parameter('show_result').value
        self.pub_result_img = self.get_parameter('pub_result_img').value

    def image_callback(self, msg: Image, robot_id: int):
        # Detect and publish results.
        image_rgb = self.bridge.imgmsg_to_cv2(msg)

        # Convert the image from BGR to RGB.
        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        # Perform YOLOv5 detection on the RGB image.
        detect_result = self.yolov5.predict(image_rgb)
        self.get_logger().info(f"Robot {robot_id} detection result: {str(detect_result)}")

        result_msg = Detection2DArray()
        result_msg.detections.clear()
        result_msg.header.frame_id = f"camera_{robot_id}"
        result_msg.header.stamp = self.get_clock().now().to_msg()

        # Parse the result.
        predictions = detect_result.pred[0]
        boxes = predictions[:, :4]  # x1, y1, x2, y2
        scores = predictions[:, 4]
        categories = predictions[:, 5]

        for index in range(len(categories)):
            name = detect_result.names[int(categories[index])]
            detection2d = Detection2D()
            detection2d.id = name
            x1, y1, x2, y2 = boxes[index]
            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)
            center_x = (x1+x2)/2.0
            center_y = (y1+y2)/2.0

            if ros_distribution == 'galactic':
                detection2d.bbox.center.x = center_x
                detection2d.bbox.center.y = center_y
            else:
                detection2d.bbox.center.position.x = center_x
                detection2d.bbox.center.position.y = center_y

            detection2d.bbox.size_x = float(x2-x1)
            detection2d.bbox.size_y = float(y2-y1)

            obj_pose = ObjectHypothesisWithPose()
            obj_pose.hypothesis.class_id = name
            obj_pose.hypothesis.score = float(scores[index])

            detection2d.results.append(obj_pose)
            result_msg.detections.append(detection2d)

            # Draw results on the original BGR image.
            if self.show_result or self.pub_result_img:
                cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(image_bgr, f"{name}", (x1, y1),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Display results if needed.
        if self.show_result:
            cv2.imshow(f'result_{robot_id}', image_bgr)
            cv2.waitKey(1)

        # Publish result images if needed.
        if self.pub_result_img:
            result_img_msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
            result_img_msg.header = msg.header
            if robot_id == 0:
                self.result_img_pub_0.publish(result_img_msg)
            else:
                self.result_img_pub_1.publish(result_img_msg)

        if len(categories) > 0:
            if robot_id == 0:
                self.yolo_result_pub_0.publish(result_msg)
            else:
                self.yolo_result_pub_1.publish(result_msg)

def main():
    rclpy.init()
    rclpy.spin(YoloV5Ros2())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
