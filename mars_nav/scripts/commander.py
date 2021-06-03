#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from mars_nav.msg import custom
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt


def rad2deg(rad):
    return rad * 180.0 / 3.14159


class Commander:
    def __init__(self):
        rospy.init_node('mars_commander', anonymous=False)

        self.gps_x = 0
        self.gps_y = 0
        self.gps_yaw = 0
        self.fid_x = 0
        self.fid_y = 0
        self.fid_yaw = 0
        self.fid_pose = False

        self.is_charging = False
        self.need_charging = False
        rospy.Subscriber("/geonav_odom", Odometry, self.gps_position)
        rospy.Subscriber("/plugged", Bool, self.charging)
        rospy.Subscriber("/battery_empty", Bool, self.battery_monitor)
        rospy.Subscriber("/fiducials_transforms",
                         Detection2DArray, self.fid_position)
        self.vel_maister = rospy.Publisher("/vel_cmd", custom, queue_size=1)

    # Triggers the robot to start driving to the charging station
    def battery_monitor(self, msg):
        self.need_charging = msg.data

    # Checks if the robot started charging
    def charging(self, msg):
        self.is_charging = msg.data

    def gps_position(self, msg):
        # extract data
        self.gps_x = msg.pose.pose.position.x
        self.gps_y = msg.pose.pose.position.x
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        (roll, pitch, self.gps_yaw) = euler_from_quaternion(qx, qy, qz, qw)

    def fid_position(self, msg):
        fiducials = [
            fid for detection in msg.detections for fid in detection.results]
        fiducial = fiducials[0]

        # extract data
        # self.id = fiducial.id # TODO: Needed once we will have multiple markers
        self.fid_x = fiducial.pose.pose.position.x
        self.fid_y = fiducial.pose.pose.position.y
        qx = fiducial.pose.pose.orientation.x
        qy = fiducial.pose.pose.orientation.y
        qz = fiducial.pose.pose.orientation.z
        qw = fiducial.pose.pose.orientation.w
        (roll, pitch, self.fid_yaw) = euler_from_quaternion(qx, qy, qz, qw)

        self.fid_pose = True

    # TODO: Shift origin to be away from the station and the perform docking maneuver
    # TODO: Robot should always be coming from on direction otherwise you have to deal with circumveing the charging station
    def send_goal(self, x, y, yaw):
        yaw_error = int(rad2deg(atan2(y, x)))
        dist = int(sqrt(x*x + y*y))
        msg = custom()
        # TODO: Use proportional controller and apply saturation function [0, 5] ~ undocking will be negative velocity
        speed = 2
        msg.data.append(speed)
        msg.data.append(dist)
        msg.data.append(yaw_error)

        self.vel_maister.publish(msg)

    def run(self):

        # Robots need charging
        if not self.is_charging and self.need_charging:
            # Get position
            if self.fid_pose:
                x = self.fid_x
                y = self.fid_y
                yaw = self.fid_yaw
            else:
                x = self.gps_x
                y = self.gps_y
                yaw = self.gps_yaw

                self.send_goal(x, y, yaw)

        # Follow a sequence of predefined points on the field(of some shape)
        elif not self.need_charging:
            x = 5.0
            y = 5.0
            yaw = 0
            self.send_goal(x, y, yaw)

        else:  # is_charging = True
            print("Commander: Robot is charging")

        # TODO: Need custom maneuvers to move away from the charging station

            # Update flag
        self.fid_pose = False


if __name__ == '__main__':
    commander = Commander()

    r = rospy.Rate(2)
    try:
        while not rospy.is_shutdown():
            commander.run()
            r.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr("GroundFiducials: Script interrupted.")
