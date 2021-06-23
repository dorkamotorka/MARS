#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Bool
from imega_arduino.msg import custom
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt


def rad2deg(rad):
    return rad * 180.0 / 3.14159

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

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
        self.need_charging = True 
        self.init_fixed_nav_waypoints()
        rospy.Subscriber("/geonav_odom", Odometry, self.gps_position)
        rospy.Subscriber("/plugged", Bool, self.charging)
        rospy.Subscriber("/battery_empty", Bool, self.battery_monitor)
        rospy.Subscriber("/stag_ros/markers_array",
                         Detection2DArray, self.fid_position)
        self.vel_maister = rospy.Publisher("/vel_cmd", custom, queue_size=100)

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
        (roll, pitch, self.gps_yaw) = euler_from_quaternion((qx, qy, qz, qw))

	print("got GPS position")

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
        (roll, pitch, self.fid_yaw) = euler_from_quaternion((qx, qy, qz, qw))

	print("Got fiducial pose")
        self.fid_pose = True

    def get_waypoints(self, fid_id, x, y):
        '''Transforms robot current pose to the frame of waypoint'''
        transform_waypoints = []
        waypoints = self.waypoints[str(fid_id)] if fid_id else [[1, 1]]
	print(waypoints)
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
                dist_e = clamp(int(sqrt(x*x + y*y)), -65535, 65535)
                yaw_e = int(rad2deg(atan2(y, x)))
		print("dist_e: ", dist_e)
		print("yaw: ", yaw_e)
                # TODO: Use proportional controller and apply saturation function [-5000, 5000] ~ undocking will be negative velocity
                speed = clamp(dist_e, -5000, 5000) 
                msg = custom()
                msg.data.append(speed)
                msg.data.append(dist_e)
                msg.data.append(yaw_e)

		print("Sending goals to the Arduino")
                self.vel_maister.publish(msg)

    def run(self):
        '''
        Main loop
        '''
        r = rospy.Rate(0.1)
        try:
            while not rospy.is_shutdown():
                r.sleep()
                # Robots need charging
                id = None
		waypoints = None
		print("before ifs")
                if not self.is_charging and self.need_charging:
		    print("NEEDS CHARGING")
                    # Get position
                    if self.fid_pose:
                        x = self.fid_x
                        y = self.fid_y
                        id = self.fid_id
			print("using fid pose")
                    else:
                        x = self.gps_x
                        y = self.gps_y
			print("using gps pose")

		    if id:
                    	waypoints = self.get_waypoints(id, x, y)
		    else:
			waypoints = self.get_waypoints(None, x, y)
		    if waypoints:
                    	self.send_goals(waypoints)
		    else:
			print("Waypoints are None!")
			continue

                # Back away from the charging station 
                elif not self.need_charging:
		    print("BACKING AWAYY!!")
                    x = 5.0
                    y = 5.0
                    yaw = 0

		    if id:
                    	waypoints = self.get_waypoints(id, x, y)
		    else:
			waypoints = self.get_waypoints(None, x, y)
		    if waypoints:
                    	self.send_goals(waypoints)
		    else:
			print("Waypoints are None!")
			continue

                else:  # is_charging = True
                    print("Commander: Robot is charging")

                # TODO: Need custom maneuvers to move away from the charging station

                    # Update flag
                self.fid_pose = False

        except rospy.ROSInterruptException:
            rospy.logerr("GroundFiducials: Script interrupted.")


'''
Main function
'''
if __name__ == '__main__':
    commander = Commander()
    commander.run()
