import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')

        self.service = self.create_service(AddTwoInts, "add_two_ints", self.serviceCallback)
        self.get_logger().info("service add_two_ints Ready")
    

    def serviceCallback(self, req, res):
        self.get_logger().info("new request received a: {}, b: {}".format(req.a, req.b))
        res.sum = req.a + req.b
        self.get_logger().info("Returning Sum: {}".format(res.sum))
        return res

def main():
    rclpy.init()
    simpleService_Server = SimpleServiceServer()
    rclpy.spin(simpleService_Server)
    simpleService_Server.destroy_node()
    rclpy.shutdown()

if __name__ == "main":
    main()