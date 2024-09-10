import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter


class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")  # This takes the node name as an arg

        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_string_param", "Tayo")

        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangeCallback(self, param):
        result = SetParametersResult()

        for par in param:
            if par.name == "simple_int_param" and par.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("Param Simple_int_param changed: New value {}".format(par.value))
                result.successful = True

            if par.name == "simple_string_param" and par.type_ == Parameter.Type.STRING:
                self.get_logger().info("Param Simple_string_param changed: New value {}".format(par.value))
                result.successful = True

        return result

def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()