#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray, BoundingBox2D
from nav_msgs.msg import Odometry
import math

class BoundingBoxController:
    def __init__(self):
        rospy.init_node('bounding_box_controller', anonymous=True)
        self.image_width = 640
        self.image_height = 512
        self.bbox_sub = rospy.Subscriber('/yolov7/yolov7', Detection2DArray, self.bbox_callback)
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.search_mode = False
        self.search_direction = -1  # -1 for counterclockwise
        self.search_speed = 1  # Angular velocity for search mode
        self.vel_odom = 0
        self.vel_cmd = 0

    def odom_callback(self, msg):
        self.vel_odom = msg.twist.twist.linear.x

    def bbox_callback(self, msg):
        self.search_mode = len(msg.detections) == 0
        if self.search_mode:
            self.search_for_box()
        else:
            self.process_detections(msg.detections)

    def search_for_box(self):
        twist_cmd = Twist()
        twist_cmd.linear.x = self.search_speed
        twist_cmd.angular.z = self.search_direction * self.search_speed
        self.cmd_vel_pub.publish(twist_cmd)

        print("Searching for Box 1..")

        vel_cmd_old = self.vel_cmd
        self.vel_cmd = twist_cmd.linear.x

        if vel_cmd_old == -self.search_speed or self.vel_cmd == 0:
            self.vel_cmd = 0
            return

        if abs(self.vel_odom) < 0.001 and vel_cmd_old:
            twist_cmd = Twist()
            twist_cmd.linear.x = -self.search_speed
            twist_cmd.angular.z = self.search_direction * self.search_speed * 2
            self.cmd_vel_pub.publish(twist_cmd)
            print("Reversing and turning around...")

    def process_detections(self, detections):
        for detection in detections:
            bbox = detection.bbox
            bbox_center_x = bbox.center.x
            bbox_center_y = bbox.center.y
            bbox_size_x = bbox.size_x
            bbox_size_y = bbox.size_y

            error_x = (self.image_width / 2) - bbox_center_x
            error_y = (self.image_height / 2) - bbox_center_y

            print(f"Detected Box 1 at x={bbox_center_x}, y={bbox_center_y}, width={bbox_size_x}, height={bbox_size_y}")

            twist_cmd = Twist()

            if bbox_size_x > self.image_width * 0.5 or bbox_size_y > self.image_height * 0.5:
                twist_cmd.linear.x = 0
                twist_cmd.angular.z = 0
            else:
                twist_cmd.linear.x = 1000 / (bbox_size_x + bbox_size_y)
                twist_cmd.angular.z = 0.2 * error_x
                twist_cmd.linear.x *= math.cos(twist_cmd.angular.z * 0.02)
                print(f"Moving forward {twist_cmd.linear.x} steering {twist_cmd.angular.z}")

            self.cmd_vel_pub.publish(twist_cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = BoundingBoxController()
    controller.run()
