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
        self.fid_id = None

        self.is_charging = False
        self.need_charging = False
        self.init_fixed_nav_waypoints()
        rospy.Subscriber("/geonav_odom", Odometry, self.gps_position)
        rospy.Subscriber("/plugged", Bool, self.charging)
        rospy.Subscriber("/battery_empty", Bool, self.battery_monitor)
        rospy.Subscriber("/fiducials_transforms",
                         Detection2DArray, self.fid_position)
        self.vel_maister = rospy.Publisher("/vel_cmd", custom, queue_size=1)

    def init_fixed_nav_waypoints(self):
        self.waypoints = {
            '1': [[0, 1], [0, 0]],
            '2': [[1, 0], [0, 1], [0, 0]],
            '3': [[-1, 0], [0, 1], [0, 0]],
        }

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
        self.fid_id = fiducial.id
        self.fid_x = fiducial.pose.pose.position.x
        self.fid_y = fiducial.pose.pose.position.y
        qx = fiducial.pose.pose.orientation.x
        qy = fiducial.pose.pose.orientation.y
        qz = fiducial.pose.pose.orientation.z
        qw = fiducial.pose.pose.orientation.w
        (roll, pitch, self.fid_yaw) = euler_from_quaternion(qx, qy, qz, qw)

        self.fid_pose = True

    def get_waypoints(self, fid_id, x, y):
        '''Transforms robot current pose to the frame of waypoint'''
        transform_waypoints = []
        waypoints = self.waypoints[fid_id] if fid_id else [[0, 0]]
        for point in waypoints:
            x_r = point[0]
            y_r = point[1]

            x_e = x_r - x
            y_e = y_r - y
            transform_waypoints.append([x_e, y_e])

        return transform_waypoints

    def send_goals(self, waypoints):
        if waypoints:
            # Goals are nested in a queue of length 20
            for waypoint in waypoints:
                x = waypoint[0]
                y = waypoint[1]
                dist_e = int(sqrt(x*x + y*y))
                yaw_e = int(rad2deg(y, x))
                msg = custom()
                # TODO: Use proportional controller and apply saturation function [0, 5] ~ undocking will be negative velocity
                speed = 2
                msg.data.append(speed)
                msg.data.append(dist_e)
                msg.data.append(yaw_e)

                self.vel_maister.publish(msg)

    def run(self):
        '''
        Main loop
        '''
        r = rospy.Rate(2)
        try:
            while not rospy.is_shutdown():
                # Robots need charging
                if not self.is_charging and self.need_charging:
                    # Get position
                    if self.fid_pose:
                        x = self.fid_x
                        y = self.fid_y
                        id = self.fid_id
                    else:
                        x = self.gps_x
                        y = self.gps_y
                        id = None

                    waypoints = self.get_waypoints(id, x, y)

                    self.send_goals(waypoints)

                # Follow a sequence of predefined points on the field(of some shape)
                elif not self.need_charging:
                    x = 5.0
                    y = 5.0
                    yaw = 0
                    self.send_goals(x, y, yaw)

                else:  # is_charging = True
                    print("Commander: Robot is charging")

                # TODO: Need custom maneuvers to move away from the charging station

                    # Update flag
                self.fid_pose = False
                r.sleep()

        except rospy.ROSInterruptException:
            rospy.logerr("GroundFiducials: Script interrupted.")


'''
Main function
'''
if __name__ == '__main__':
    commander = Commander()
    commander.run()
