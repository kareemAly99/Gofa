#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class AddTanksNode(Node):
    def __init__(self):
        super().__init__('add_tanks_node')
        self.publisher = self.create_publisher(Marker, 'tanks_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_tanks)
        self.get_logger().info("AddTanksNode started")

        # Tank parameters
        self.tank_width = 0.3      # 300 mm
        self.tank_height = 0.46    # 460 mm
        self.tank_spacing = 0.14   # 140 mm
        self.offset_x = 0.805      # distance along x
        self.num_tanks = 4

        # Robot is exactly between the two rightmost tanks (tank 2 and tank 3)
        half_gap = (self.tank_width + self.tank_spacing) / 2.0

        # Compute y positions so robot is centered between tank 2 and tank 3
        # Let robot_y = 0 for reference
        self.robot_y = 0.0
        y3 = self.robot_y + half_gap
        y2 = self.robot_y - half_gap
        y1 = y2 - (self.tank_width + self.tank_spacing)
        y0 = y1 - (self.tank_width + self.tank_spacing)

        self.tank_positions_y = [y0, y1, y2, y3]

    def create_tank_marker(self, tank_index):
        marker = Marker()
        # Use base_link as frame, since RViz fixed frame = base_link
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tanks"
        marker.id = tank_index
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # line thickness
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        y_offset = self.tank_positions_y[tank_index]
        x_offset = self.offset_x
        # Tank bottom 460 mm above ground (z = 0 in base_link)
        z_offset = self.tank_height / 2.0

        w = self.tank_width / 2.0
        h = self.tank_height / 2.0

        corners = [
            Point(x=x_offset - w, y=y_offset - w, z=z_offset - h),
            Point(x=x_offset + w, y=y_offset - w, z=z_offset - h),
            Point(x=x_offset + w, y=y_offset + w, z=z_offset - h),
            Point(x=x_offset - w, y=y_offset + w, z=z_offset - h),
            Point(x=x_offset - w, y=y_offset - w, z=z_offset + h),
            Point(x=x_offset + w, y=y_offset - w, z=z_offset + h),
            Point(x=x_offset + w, y=y_offset + w, z=z_offset + h),
            Point(x=x_offset - w, y=y_offset + w, z=z_offset + h),
        ]

        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]

        for start, end in edges:
            marker.points.append(corners[start])
            marker.points.append(corners[end])

        return marker

    def publish_tanks(self):
        for i in range(self.num_tanks):
            marker = self.create_tank_marker(i)
            self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = AddTanksNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
