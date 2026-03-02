import rclpy
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
        self.declare_parameter("min_person_bbox_height_px", 0.0)
        self.declare_parameter("reaction_mode", "stop")  # stop | yield
        self.declare_parameter("yield_linear_x", 0.08)
        self.declare_parameter("yield_angular_z", 0.6)
        self.declare_parameter("yield_direction", "left")  # left | right
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
        self._min_person_bbox_height_px = (
            self.get_parameter("min_person_bbox_height_px")
            .get_parameter_value()
            .double_value
        )
        self._reaction_mode = (
            self.get_parameter("reaction_mode").get_parameter_value().string_value
        ).strip().lower()
        self._yield_linear_x = (
            self.get_parameter("yield_linear_x").get_parameter_value().double_value
        )
        self._yield_angular_z = (
            self.get_parameter("yield_angular_z").get_parameter_value().double_value
        )
        self._yield_direction = (
            self.get_parameter("yield_direction").get_parameter_value().string_value
        ).strip().lower()
        self._hold_time = (
            self.get_parameter("hold_time").get_parameter_value().double_value
        )

        if self._reaction_mode not in {"stop", "yield"}:
            self.get_logger().warn(
                f"Invalid reaction_mode='{self._reaction_mode}', falling back to 'stop'"
            )
            self._reaction_mode = "stop"

        if self._yield_direction not in {"left", "right"}:
            self.get_logger().warn(
                f"Invalid yield_direction='{self._yield_direction}', falling back to 'left'"
            )
            self._yield_direction = "left"

        self._last_person_time = None

        self._detections_sub = self.create_subscription(
            Detection2DArray, detections_topic, self._on_detections, 10
        )
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Timer at 10 Hz to publish stop commands while a person is detected
        self._timer = self.create_timer(0.1, self._on_timer)

        self.get_logger().info(
            "YOLO Reactive Controller started. Listening on '%s', publishing stop "
            "or yield commands on '%s' when '%s' detected with score >= %.2f, "
            "bbox_height >= %.1f px, mode=%s"
            % (
                detections_topic,
                cmd_vel_topic,
                self._person_class_id,
                self._min_score,
                self._min_person_bbox_height_px,
                self._reaction_mode,
            )
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
                bbox_height_px = float(detection.bbox.size_y)
                passes_distance_gate = (
                    self._min_person_bbox_height_px <= 0.0
                    or bbox_height_px >= self._min_person_bbox_height_px
                )
                if (
                    class_id == self._person_class_id
                    and score >= self._min_score
                    and passes_distance_gate
                ):
                    found_person = True
                    self.get_logger().info(
                        f"[DETECTION] Person detected! class={class_id}, "
                        f"score={score:.2f}, bbox_height={bbox_height_px:.1f}px"
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
            cmd = Twist()

            if self._reaction_mode == "yield":
                turn_sign = 1.0 if self._yield_direction == "left" else -1.0
                cmd.linear.x = float(self._yield_linear_x)
                cmd.angular.z = turn_sign * float(self._yield_angular_z)
            else:
                # stop mode
                cmd = Twist()

            self._cmd_pub.publish(cmd)

            if dt < 0.05:
                if self._reaction_mode == "yield":
                    self.get_logger().info(
                        f"[YIELD] Person in frame - moving aside "
                        f"(vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f})"
                    )
                else:
                    self.get_logger().info("[STOP] Person in frame - robot stopped")
            else:
                self.get_logger().debug(f"[HOLDING] Waiting for person to fully leave (hold_time in {self._hold_time - dt:.2f}s)")
        else:
            # detection is gone and hold_time has expired; resume
            self.get_logger().info(f"[RESUME] Person absent for {dt:.2f}s - robot can move")
            self._last_person_time = None


def main(args=None) -> None:
    rclpy.init(args=args)
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
