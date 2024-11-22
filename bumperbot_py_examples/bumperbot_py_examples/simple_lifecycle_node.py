import rclpy
import time
import rclpy.executors
from rclpy.lifecycle import Node, State
from rclpy.lifecycle.node import TransitionCallbackReturn
from std_msgs.msg import String

class SimpleLifecycleNode(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
    

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.sub = self.create_subscription(String, "chatter", self.msg_callback, 10)
        self.get_logger().info("Life Cycle node on_configure() called")
        return super().on_configure(state)
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.sub)
        self.get_logger().info("Life Cycle node on_shutdown() called")
        return super().on_shutdown(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.sub)
        self.get_logger().info("Life Cycle node on_cleanup() called")
        return super().on_cleanup(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Life Cycle node on_activate() called")
        time.sleep(2.0)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Life Cycle node on_deactivate() called")
        return super().on_deactivate(state)
    
    def msg_callback(self, msg: String):
        current_state = self._state_machine.current_state
        if current_state[1] == "active":
            self.get_logger().info("I heard: {}".format(msg.data))

def main():
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    simple_lifecycle_node = SimpleLifecycleNode("simple_lifecycle_node")
    executor.add_node(simple_lifecycle_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        simple_lifecycle_node.destroy_node()

if __name__ == "__main__":
    main()


