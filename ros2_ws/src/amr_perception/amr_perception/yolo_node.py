import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose, BoundingBox2D

try:
    from ultralytics import YOLO
except Exception as exc:
    YOLO = None
    _YOLO_IMPORT_ERROR = exc

class YoloPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('yolo_perception')
        
        # YOLO26 is NMS-free, so conf_threshold is the primary filter
        self.declare_parameter('model_path', 'yolo26n.pt')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('annotated_image_topic', '/perception/annotated_image')
        self.declare_parameter('conf_threshold', 0.25)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        annotated_image_topic = self.get_parameter('annotated_image_topic').get_parameter_value().string_value

        if YOLO is None:
            self.get_logger().error(f'Ultralytics error: {_YOLO_IMPORT_ERROR}')
            raise RuntimeError('Ultralytics YOLO is required')

        # Load YOLO26 Model
        self._model = YOLO(model_path)
        self._bridge = CvBridge()

        self._image_sub = self.create_subscription(Image, image_topic, self._on_image, 10)
        self._detections_pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self._annotated_image_pub = self.create_publisher(Image, annotated_image_topic, 10)

        self.get_logger().info(f'YOLO26 Node Started. Model: {model_path}')

    def _on_image(self, msg: Image) -> None:
        conf_thres = self.get_parameter('conf_threshold').get_parameter_value().double_value

        # Convert ROS Image to OpenCV
        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        annotated_image = cv_image.copy()

        # YOLO26 Inference
        # We don't need 'iou' because YOLO26 is natively NMS-free
        results = self._model.predict(
            source=cv_image,
            conf=conf_thres,
            verbose=False,
            half=True  # Optional: Uses FP16 for ~2x speed on supported GPUs
        )

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        if not results or len(results[0].boxes) == 0:
            self._detections_pub.publish(detection_array)
            annotated_msg = self._bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self._annotated_image_pub.publish(annotated_msg)
            return

        result = results[0]
        
        for box in result.boxes:
            # YOLO26 'boxes' objects handle coordinates directly
            # xywh[0] contains [center_x, center_y, width, height]
            cx, cy, w, h = box.xywh[0].tolist()
            cls_id = int(box.cls[0].item())
            score = float(box.conf[0].item())
            
            # Look up the human-readable name (e.g., 'person')
            class_name = result.names[cls_id]

            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.x = cx
            detection.bbox.center.y = cy
            detection.bbox.size_x = w
            detection.bbox.size_y = h

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = score
            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

            x1 = int(cx - (w / 2.0))
            y1 = int(cy - (h / 2.0))
            x2 = int(cx + (w / 2.0))
            y2 = int(cy + (h / 2.0))
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{class_name} {score:.2f}"
            cv2.putText(
                annotated_image,
                label,
                (x1, max(0, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        self._detections_pub.publish(detection_array)
        annotated_msg = self._bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        annotated_msg.header = msg.header
        self._annotated_image_pub.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()