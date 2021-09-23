#!/usr/bin/python

import rospy
import time
from rospy.timer import sleep
from std_msgs.msg import Bool
from diagnostics_msgs.msg import LampsDiagnostics, LampStates
from diagnostics_srvs.srv import NotifyLampTurnOn;

class BrokenLampDetector():
    def __init__(self):
        self.node_handler = rospy.init_node('broken_lamp_detector')

        self.lamps = {"1": {"turned_on": False, "actually_on": False}, "2": {"turned_on": False, "actually_on": False},
                        "3": {"turned_on": False, "actually_on": False}, "4": {"turned_on": False, "actually_on": False},
                        "5": {"turned_on": False, "actually_on": False}, "6": {"turned_on": False, "actually_on": False},
                        "7": {"turned_on": False, "actually_on": False}, "8": {"turned_on": False, "actually_on": False}}

        self.timer = 0

        lamps_diagnostics_subscriber = rospy.Subscriber("diagnostics/lamps", LampsDiagnostics, self.lamps_diagnostics_callback)
        notify_lamp_turn_on_service = rospy.Service("broken_lamps_detector/notify_lamp_turn_on", NotifyLampTurnOn, self.turn_lamp_on_handler)

        self.broken_lamps_publisher = rospy.Publisher("broken_lamps_detector/broken_lamps", LampStates, queue_size=10)

    def turn_lamp_on_handler(self, req):
        lamp_number = req.lamp_number

        for lamp in self.lamps:
            if self.lamps[lamp] == lamp_number and self.lamps[lamp]["turned_on"] == False:
                self.lamps[lamp]["turned_on"] = True

        while (self.lamps[lamp]["actually_on"] == False and self.timer < 10):
            time.sleep(1)
            self.timer += 1

        self.timer = 0
        
    def lamps_diagnostics_callback(self, data):
        lamp_1_on = data.lamp_states.lamp_1.data
        lamp_2_on = data.lamp_states.lamp_2.data
        lamp_3_on = data.lamp_states.lamp_3.data
        lamp_4_on = data.lamp_states.lamp_4.data
        lamp_5_on = data.lamp_states.lamp_5.data
        lamp_6_on = data.lamp_states.lamp_6.data
        lamp_7_on = data.lamp_states.lamp_7.data
        lamp_8_on = data.lamp_states.lamp_8.data

        for lamp in self.lamps:
            self.lamps[lamp]["actually_on"] = eval("lamp_" + lamp + "_on")

    def publish_broken_lamps(self):
        lamp_states = LampStates()
        
        for lamp in self.lamps:
            exec("lamp_states.lamp_" + lamp + ".data = " + str(self.lamps[lamp]["actually_on"]))

        self.broken_lamps_publisher.publish(lamp_states)
        
if __name__ == "__main__":
    try:
        
        broken_lamp_detector = BrokenLampDetector()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            broken_lamp_detector.publish_broken_lamps()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

