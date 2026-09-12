import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sub_vision_interfaces.msg import DetectionArray


class AnnotatedNode(Node):
    def __init__(self):
        super().__init__("sub_vision_annotater")
        self._bridge = CvBridge()

        self._latest_image = None
        self._latest_header = None
        self._latest_detections = None
        self._pub = self.create_publisher(
            Image,
            "vision/debug_image",
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Image,
            "front_camera/image_color",
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DetectionArray,
            "vision/detections",
            self.detection_callback,
            qos_profile_sensor_data,
        )

    def image_callback(self, msg):
        self._latest_header = msg.header
        self._latest_image = self._bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8",
        )

        self.publish_debug()

    def detection_callback(self, msg):
        self._latest_detections = msg
        self.publish_debug()

    def publish_debug(self):

        if self._latest_image is None or self._latest_detections is None:
            return

        annotated = self._latest_image.copy()

        if self._latest_detections is not None:
            for det in self._latest_detections.detections:
                bbox = det.detection.bbox

                cx = bbox.center.position.x
                cy = bbox.center.position.y

                w = bbox.size_x
                h = bbox.size_y

                x1 = int(cx - w / 2)
                y1 = int(cy - h / 2)
                x2 = int(cx + w / 2)
                y2 = int(cy + h / 2)

                result = det.detection.results[0]

                cls = result.hypothesis.class_id
                score = result.hypothesis.score

                cv2.rectangle(
                    annotated,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    annotated,
                    f"{cls} {score:.2f}",
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

            msg = self._bridge.cv2_to_imgmsg(
                annotated,
                encoding="bgr8",
            )
            msg.header = self._latest_header
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AnnotatedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
