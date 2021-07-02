#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Bool, Int8
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
    NEED_CHARGING = 1
    CHARGING = 2   
    CHARGED = 3
    GO_AWAY = 4
    #NORMAL = 5

    CHARGING_STATION = [[0, 0]]
    FIELD_WAYPOINT = [[5, 5]]
    MARKER1_MAN = [[1, 1], [0, 0]]
    MARKER2_MAN = [[1, 1], [0, 0]]
    MARKER3_MAN = [[1, 1], [0, 0]]
    MARKER4_MAN = [[0, 0]]

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

        self.current_state = None 
        self.init_fixed_nav_waypoints()
        rospy.Subscriber("/geonav_odom", Odometry, self.gps_position)
        rospy.Subscriber("/battery", Int8, self.battery_monitor)
        rospy.Subscriber("/stag_ros/markers_array",
                         Detection2DArray, self.fid_position)
        self.vel_maister = rospy.Publisher("/vel_cmd", custom, queue_size=1)

    def init_fixed_nav_waypoints(self):
        self.waypoints = {
            '1': self.MARKER1_MAN,
            '2': self.MARKER2_MAN,
            '3': self.MARKER3_MAN,
            '4': self.MARKER4_MAN,
        }

    # Triggers the robot to start driving to the charging station
    def battery_monitor(self, msg):
        self.current_state = msg.data

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

    def get_waypoints(self, fid_id, x, y, in_way): # robot pose and goal pose
        '''Transforms robot current pose to the frame of waypoint'''
        transform_waypoints = []
        waypoints = self.waypoints[str(fid_id)] if fid_id else in_way
	print(waypoints)
        for point in waypoints:
            x_r = point[0]
            y_r = point[1]

            x_e = x_r - x
            y_e = y_r - y
            transform_waypoints.append([x_e, y_e])

        return transform_waypoints

    def send_goals(self, waypoints, slow):
        if waypoints:
            # Goals are nested in a queue of length 20
            for waypoint in waypoints:
                x = waypoint[0]
                y = waypoint[1]
                dist_e = clamp(int(sqrt(x*x + y*y)), -65535, 65535)
                yaw_e = int(rad2deg(atan2(y, x)))
		#print("dist_e: ", dist_e)
		#print("yaw: ", yaw_e)
                # TODO: Use proportional controller and apply saturation function [-5000, 5000] ~ undocking will be negative velocity
                speed = clamp(dist_e, -5000, 5000) 
		if slow:
		    speed = 10
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
        r = rospy.Rate(1)
        try:
            while not rospy.is_shutdown():
                r.sleep()
                id = None
		got_pose = False
		waypoints = None
                if self.current_state:

		    # If the robot is charging don't do anything
                    if self.current_state == self.CHARGING:
                        rospy.loginfo('Robot is CHARGING')
                        continue

		    # If the robot needs charging send charging station waypoint (Dont send [0,0] because the waypoints dont get preempted)
                    elif self.current_state == self.NEED_CHARGING:
                        print("NEEDS CHARGING")
                        # Get position
                        if self.fid_pose:
                            x = self.fid_x
                            y = self.fid_y
                            id = self.fid_id
			    got_pose = True
                            print("using fid pose")
                        else:
                            x = self.gps_x
                            y = self.gps_y
                            print("using gps pose")
			    got_pose = True

		
			if got_pose:
			    waypoints = self.get_waypoints(id, x, y, self.CHARGING_STATION)
			    if waypoints:
		                self.send_goals(waypoints, id == 4) # If ID == 4 this is the SMALL marker and we should drive slow!
			    else:
			        print("Waypoints are None!")
		     	        continue
			else:
			    rospy.loginfo('Dont have robot pose to plan a path')
			    continue

                    # Back away from the charging station 
                    elif self.current_state == self.GO_AWAY:

                        if self.fid_pose:
                            x = self.fid_x
                            y = self.fid_y
                            id = self.fid_id
			    got_pose = True
                            print("using fid pose")
                        else:
                            x = self.gps_x
                            y = self.gps_y
                            print("using gps pose")
			    got_pose = True
			
			if got_pose:
                            waypoints = self.get_waypoints(None, x, y, self.FIELD_WAYPOINT) # ID must be none otherwise it might override field goal with fiducial goal
                            if waypoints:
                                self.send_goals(waypoints, False)
                            else:
                                print("Waypoints are None!")
                                continue
			else:
			    rospy.loginfo('Dont have robot pose to plan a path')
			    continue

		    #elif self.current_state == self.NORMAL:
			# print('NORMAL surveying of the field!')

                    # TODO: Need custom maneuvers to move away from the charging station

                        # Update flag
                    self.fid_pose = False
		    self.current_state = None

        except rospy.ROSInterruptException:
            rospy.logerr("GroundFiducials: Script interrupted.")


'''
Main function
'''
if __name__ == '__main__':
    commander = Commander()
    commander.run()
