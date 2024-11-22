#!/usr/bin/env python3
import rclpy
import math
from enum import Enum
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from twist_mux_msgs.action import JoyTurbo
from visualization_msgs.msg import Marker, MarkerArray


class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop_node")
        self.declare_parameter("danger_distance", 0.2)
        self.declare_parameter("warning_distance", 0.6)
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("safety_stop_topic", "safety_stop")
        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value
        self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value

        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, 10) # subscribes to laser topic
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10) # publishes safety topics
        self.zones_pub = self.create_publisher(MarkerArray, "zones", 10)

        self.decrease_speed_client = ActionClient(self, JoyTurbo, "joy_turbo_decrease")
        self.increase_speed_client = ActionClient(self, JoyTurbo, "joy_turbo_increase")

        while not self.decrease_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_decrease not available. Waiting.....")
        while not self.increase_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_increase not available. Waiting.....")
        
        self._zones = MarkerArray()
        warningZone = Marker()
        warningZone.id = 0
        warningZone.action = Marker.ADD
        warningZone.type = Marker.CYLINDER
        warningZone.scale.z = 0.001
        warningZone.scale.x = self.warning_distance * 2
        warningZone.scale.y = self.warning_distance * 2
        warningZone.color.r = 1.0
        warningZone.color.g = 0.984
        warningZone.color.b = 0.0
        warningZone.color.a = 0.5

        dangerZone = Marker()
        dangerZone.id = 1
        dangerZone.action = Marker.ADD
        dangerZone.type = Marker.CYLINDER
        dangerZone.scale.z = 0.001
        dangerZone.scale.x = self.danger_distance * 2
        dangerZone.scale.y = self.danger_distance * 2
        dangerZone.color.r = 1.0
        dangerZone.color.g = 0.0
        dangerZone.color.b = 0.0
        dangerZone.color.a = 0.5
        dangerZone.pose.position.z = 0.01

        self._zones.markers = [warningZone, dangerZone]


        self.state = State.FREE
        self.prev_state = self.state
        self.is_first_msg = True
    
    def laser_callback(self, msg:LaserScan):
        self.state = State.FREE
        for range_value in msg.ranges:
            if not math.isinf(range_value) and (range_value <= self.warning_distance):
                self.state = State.WARNING
                if (range_value <= self.danger_distance):
                    self.state = State.DANGER
                    break
        if self.state != self.prev_state:
            is_safety_stop = Bool()
            if self.state == State.WARNING:
                is_safety_stop.data = False
                self._zones.markers[0].color.a = 1.0
                self._zones.markers[1].color.a = 0.5
                self.decrease_speed_client.send_goal_async(JoyTurbo.Goal())
                
            elif self.state == State.DANGER:
                self._zones.markers[0].color.a = 1.0
                self._zones.markers[1].color.a = 1.0
                is_safety_stop.data = True

            elif self.state == State.FREE:
                self._zones.markers[0].color.a = 0.5
                self._zones.markers[1].color.a = 0.5
                is_safety_stop.data = False
                self.increase_speed_client.send_goal_async(JoyTurbo.Goal())
            
            self.prev_state = self.state
            self.safety_stop_pub.publish(is_safety_stop)
        if (self.is_first_msg):
            for zone in self._zones.markers:
                zone.header.frame_id = msg.header.frame_id
            self.is_first_msg = False

        self.zones_pub.publish(self._zones)

def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()