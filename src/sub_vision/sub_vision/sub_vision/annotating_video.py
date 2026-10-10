import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from sub_vision_interfaces.msg import DetectionArray


class AnnotatedNode(Node):
    def __init__(self):
        super().__init__("sub_vision_annotater")
        self._bridge = CvBridge()

        self._latest_image = None
        self._latest_detections = None
        self._pub = self.create_publisher(
            Image,
            "vision/debug_image",
            qos_profile_sensor_data,
        )
        # JPEG copy for viewers across the network; each is only drawn and sent with subscribers.
        self._pub_compressed = self.create_publisher(
            CompressedImage,
            "vision/debug_image/compressed",
            qos_profile_sensor_data,
        )

        # As in sub_vision: "compressed" reads <image>/compressed, for frames over the network
        self.declare_parameter("image_transport", "raw")
        transport = self.get_parameter("image_transport").value
        if transport == "compressed":
            self.create_subscription(
                CompressedImage,
                "front_camera/image_color/compressed",
                self.image_compressed_callback,
                qos_profile_sensor_data,
            )
        elif transport == "raw":
            self.create_subscription(
                Image,
                "front_camera/image_color",
                self.image_callback,
                qos_profile_sensor_data,
            )
        else:
            raise ValueError(f"unsupported image_transport '{transport}' (raw|compressed)")
        self.create_subscription(
            DetectionArray,
            "vision/detections",
            self.detection_callback,
            qos_profile_sensor_data,
        )

    # Frames are kept as messages and only decoded when something subscribes, so
    # an annotator nobody is watching costs next to nothing.
    def image_callback(self, msg):
        self._latest_image = msg
        self.publish_debug()

    def image_compressed_callback(self, msg):
        self._latest_image = msg
        self.publish_debug()

    def detection_callback(self, msg):
        self._latest_detections = msg
        self.publish_debug()

    def publish_debug(self):

        if self._latest_image is None or self._latest_detections is None:
            return
        send_raw = self._pub.get_subscription_count() > 0
        send_compressed = self._pub_compressed.get_subscription_count() > 0
        if not send_raw and not send_compressed:
            return

        if isinstance(self._latest_image, CompressedImage):
            annotated = self._bridge.compressed_imgmsg_to_cv2(
                self._latest_image, desired_encoding="bgr8"
            )
        else:
            # A copy: the conversion may return a read-only view of the message.
            annotated = self._bridge.imgmsg_to_cv2(
                self._latest_image, desired_encoding="bgr8"
            ).copy()

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

            cv2.rectangle(
                annotated,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )

            # A detection need not carry a hypothesis; draw its box unlabelled then
            if not det.detection.results:
                continue
            result = det.detection.results[0]

            cls = result.hypothesis.class_id
            score = result.hypothesis.score

            cv2.putText(
                annotated,
                f"{cls} {score:.2f}",
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        if send_raw:
            msg = self._bridge.cv2_to_imgmsg(
                annotated,
                encoding="bgr8",
            )
            msg.header = self._latest_image.header
            self._pub.publish(msg)

        if send_compressed:
            msg = self._bridge.cv2_to_compressed_imgmsg(
                annotated,
                dst_format="jpg",
            )
            msg.header = self._latest_image.header
            self._pub_compressed.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AnnotatedNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
