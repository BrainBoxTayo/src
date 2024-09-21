import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    def __init__(self, a, b):
        super().__init__("simple_service_client")

        self.simple_service_client = self.create_client(AddTwoInts, "add_two_ints")
        
        while not self.simple_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again....")

        self.req = AddTwoInts.Request()
        self.req.a = int(a)
        self.req.b = int(b)

        self.future = self.simple_service_client.call_async(self.req)
        self.future.add_done_callback(self.responseCallback)
    
    def responseCallback(self, future):
        self.get_logger().info("Received response: {}".format(future.result().sum))


def main():
    rclpy.init()
    if len(sys.argv) != 3:
        print("wrong number of arguments!! Usage: \
              simple_service_client A B")
        return (-1)
    simpleClient = SimpleServiceClient(sys.argv[1], sys.argv[2])
    
    # rclpy.spin(simpleClient)
    simpleClient.destroy_node()
    rclpy.shutdown()

if __name__ == "main":
    main()