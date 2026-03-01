import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray


class YoloReactiveController(Node):
    """Simple safety controller that stops the robot when YOLO detects a person.

    This node subscribes to a Detection2DArray topic (from the YOLO node) and,
    whenever a detection with class_id == "person" (or a configurable class)
    above a minimum confidence is observed, it publishes a zero Twist on the
    configured cmd_vel topic for a short hold time.

    It does not modify Nav2; it simply overrides its velocity commands by
    publishing zero velocity when a person is detected.
    """

    def __init__(self) -> None:
        super().__init__("yolo_reactive_controller")

        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("person_class_id", "person")
        self.declare_parameter("min_score", 0.5)
        self.declare_parameter("hold_time", 0.5)  # seconds to keep stopping

        detections_topic = (
            self.get_parameter("detections_topic").get_parameter_value().string_value
        )
        cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self._person_class_id = (
            self.get_parameter("person_class_id").get_parameter_value().string_value
        )
        self._min_score = (
            self.get_parameter("min_score").get_parameter_value().double_value
        )
        self._hold_time = (
            self.get_parameter("hold_time").get_parameter_value().double_value
        )

        self._last_person_time = None

        self._detections_sub = self.create_subscription(
            Detection2DArray, detections_topic, self._on_detections, 10
        )
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Timer at 10 Hz to publish stop commands while a person is detected
        self._timer = self.create_timer(0.1, self._on_timer)

        self.get_logger().info(
            "YOLO Reactive Controller started. Listening on '%s', publishing stop "
            "commands on '%s' when '%s' detected with score >= %.2f"
            % (detections_topic, cmd_vel_topic, self._person_class_id, self._min_score)
        )

    def _on_detections(self, msg: Detection2DArray) -> None:
        """Callback for YOLO detections.

        If any detection matches the configured person class and score
        threshold, record the current time so the timer callback will publish
        a stop command.
        """

        found_person = False
        for detection in msg.detections:
            for result in detection.results:
                class_id = result.hypothesis.class_id
                score = float(result.hypothesis.score)
                if class_id == self._person_class_id and score >= self._min_score:
                    found_person = True
                    self.get_logger().info(
                        f"[DETECTION] Person detected! class={class_id}, score={score:.2f}"
                    )
                    break
            if found_person:
                break

        if found_person:
            self._last_person_time = self.get_clock().now()

    def _on_timer(self) -> None:
        """Periodically publish zero Twist while a person is in frame.
        
        Once person leaves frame, continue stopping for hold_time seconds.
        """

        if self._last_person_time is None:
            return

        now = self.get_clock().now()
        dt = (now - self._last_person_time).nanoseconds / 1e9

        if dt <= self._hold_time:
            # Publish a zero Twist to stop the robot
            stop_cmd = Twist()
            self._cmd_pub.publish(stop_cmd)
            if dt < 0.05:
                # Person is actively in frame (dt is very small)
                self.get_logger().info("[STOP] Person in frame - robot stopped")
            else:
                # Person left frame but within hold_time grace period
                self.get_logger().debug(f"[HOLDING] Waiting for person to fully leave (hold_time in {self._hold_time - dt:.2f}s)")
        else:
            # detection is gone and hold_time has expired; resume
            self.get_logger().info(f"[RESUME] Person absent for {dt:.2f}s - robot can move")
            self._last_person_time = None


def main(args=None) -> None:
    rclpy.init(=args)
    node = YoloReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
