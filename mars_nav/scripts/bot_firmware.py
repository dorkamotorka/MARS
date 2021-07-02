#!/usr/bin/env python

import sys
import rospy
import pigpio
from std_msgs.msg import Int8


class BatteryState:
    NEED_CHARGING = 1
    CHARGING = 2
    CHARGED = 3
    GO_AWAY = 4
    NORMAL = 5

    BATTERY_LOW_THRESH = 15.0
    BATTERY_FULL_THRESH = 18.0

    #BATTERY_PIN = 40
    BUMPER1_PIN = 3
    BUMPER2_PIN = 4

    def __init__(self):
        rospy.init_node("battery_node", anonymous=False)
        self.battery_voltage = 14.0
        self.bat_pub = rospy.Publisher("/battery", Int8, queue_size=10)
        rospy.Subscriber("/battery_voltage", Int8, self.battery_callback)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("pigpio not connected")
            sys.exit()

        #self.pi.set_mode(self.BATTERY_PIN, pigpio.INPUT)
        self.pi.set_mode(self.BUMPER1_PIN, pigpio.INPUT)
        self.pi.set_mode(self.BUMPER2_PIN, pigpio.INPUT)
        self.current_state = None

    def battery_callback(self, msg):
        val = msg.data
        #self.battery_voltage = 0.155 * val

    def get_state(self, voltage):
        if voltage:
            if voltage < self.BATTERY_LOW_THRESH:
                #rospy.logwarn('Battery LOW!')
                return self.NEED_CHARGING
            elif voltage > self.BATTERY_FULL_THRESH:
                #rospy.logwarn('Battery FULL!')
                return self.CHARGED
        return None

    def run(self):
        r = rospy.Rate(1)

        while not rospy.is_shutdown():
            r.sleep()

            in_bump1 = int(not self.pi.read(self.BUMPER1_PIN))
            in_bump2 = int(not self.pi.read(self.BUMPER2_PIN))
            #print("one: ", in_bump1)
            #print("second: ", in_bump2)
            bump = in_bump1 or in_bump2
            self.current_state = self.CHARGING if bump else None

            # If charging
            # Check if it charged, if medium/low persist charging
            if self.current_state:
                #rospy.loginfo('Bumpers hit')
                if self.battery_voltage:
                    if self.get_state(self.battery_voltage) == self.CHARGED:
                        #rospy.loginfo("Battery full")
                        msg = Int8()
                        msg.data = self.GO_AWAY
                        self.bat_pub.publish(msg)
                        continue
                    else:
                        #rospy.loginfo("Battery not full yet, persist charging")
                        msg = Int8()
                        msg.data = self.CHARGING
                        self.bat_pub.publish(msg)
                        continue

                else:
                    #rospy.loginfo('Battery voltage is None')
                    continue

            else:
                if self.battery_voltage:
                    print(self.battery_voltage)
                    if self.get_state(self.battery_voltage) == self.NEED_CHARGING:
                        #rospy.loginfo('Robot need charging')
                        msg = Int8()
                        msg.data = self.NEED_CHARGING
                        self.bat_pub.publish(msg)
                        continue
                    else:
                        #rospy.loginfo('Robot is OK')
                        msg = Int8()
                        msg.data = self.NORMAL
                        self.bat_pub.publish(msg)
                        continue
                else:
                    rospy.logwarn("Battery state is None")
                    continue

        rospy.logwarn("Stopping Battery Node")
        self.pi.stop()


if __name__ == '__main__':
    bs = BatteryState()
    bs.run()
