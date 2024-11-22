#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
import rclpy.time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException
from tf_transformations import euler_from_quaternion


PRIOR_PROB = 0.5
OCC_PROB = 0.9
FREE_PROB = 0.35

class Pose:
    def __init__(self, px = 0, py = 0):
        self.x = px
        self.y = py

def poseToCell(pose: Pose, map_info: MapMetaData):
    return map_info.width * pose.y + pose.x
def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose(px, py)
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution) 
    pose.y = round((py - map_info.origin.position.x) / map_info.resolution) 

    return pose

def poseOnMap(pose: Pose, map_info: MapMetaData):
    return pose.x < map_info.width and pose.x >=0 and pose.y < map_info.height and pose.y >= 0

def bresenham(start: Pose, end: Pose):
    line = []

    dx = end.x - start.x
    dy = end.y - start.y

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0

    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy

    return line

def inverseSensorModel(p_robot: Pose, p_beam: Pose):
    occ_values = []
    line = bresenham(p_robot, p_beam)
    
    for pose in line[:-1]:
        occ_values.append((pose, FREE_PROB))
    occ_values.append((line[-1], OCC_PROB))
    return occ_values
def prob2Logodds(p):
    return math.log(p / (1 - p))

def logodds2prob(l):
    try:
        return 1 - (1 / (1 + math.exp(l)))
    except OverflowError:
        return 1.0 if l > 0 else 0.0
    


class MappingWithKnownPoses(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter("width", 50.0)
        self.declare_parameter("height", 50.0)
        self.declare_parameter("resolution", 0.1)

        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        resolution = self.get_parameter("resolution").value

        self.map = OccupancyGrid()
        self.map.info.resolution = resolution
        self.map.info.width = round(width / resolution)
        self.map.info.height = round(height / resolution)
        self.map.info.origin.position.x = float(-round(width / 2.0))
        self.map.info.origin.position.y = float(-round(height / 2.0))
        self.map.header.frame_id = "odom"
        self.map.data = [-1] * (self.map.info.width * self.map.info.height)

        self.probability_map = [prob2Logodds(PRIOR_PROB)] * (self.map.info.width * self.map.info.height)

        self.map_pub = self.create_publisher(OccupancyGrid, "map", 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_Callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) 
    
    def scan_callback(self, msg: LaserScan):
        try:
            t = self.tf_buffer.lookup_transform(self.map.header.frame_id, msg.header.frame_id, rclpy.time.Time())
        except LookupException:
            self.get_logger().error("Unable to transform between /odom and /base_footprint")
            return
        
        (roll, pitch, yaw) =  euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y,
                                                        t.transform.rotation.z, t.transform.rotation.w])
        robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map.info)
        if not poseOnMap(robot_p, self.map.info):
            self.get_logger().error("The robot is out of the map!")
            return
        for i in (range(len(msg.ranges))):
            if math.isinf(msg.ranges[i]):
                continue
            
            angle = msg.angle_min + (i * msg.angle_increment) + yaw
            px = msg.ranges[i] * math.cos(angle) + t.transform.translation.x
            py = msg.ranges[i] * math.sin(angle) + t.transform.translation.y
            beam_pose = coordinatesToPose(px, py, self.map.info)

            if not poseOnMap(beam_pose, self.map.info):
                continue
            poses = inverseSensorModel(robot_p, beam_pose)
            for pose, value in poses:
                cell = poseToCell(pose, self.map.info)
                self.probability_map[cell] += prob2Logodds(value) - prob2Logodds(PRIOR_PROB)

        

    def timer_Callback(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map.data = [int(logodds2prob(value) * 100) for value in self.probability_map]
        self.map_pub.publish(self.map)


def main():
    rclpy.init()
    node = MappingWithKnownPoses("mapping_with_known_poses")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()