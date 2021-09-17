#!/usr/bin/python

import rospy
from diagnostics.msg import LampsDiagnostics, LampStates

class BrokenLampDetector():
    def __init__(self):
        self.node_handler = rospy.init_node('broken_lamp_detector')


        self.lamps = {"1": {"turned_on": False, "actually_on": False}, "2": {"turned_on": False, "actually_on": False},
                        "3": {"turned_on": False, "actually_on": False}, "4": {"turned_on": False, "actually_on": False},
                        "5": {"turned_on": False, "actually_on": False}, "6": {"turned_on": False, "actually_on": False},
                        "7": {"turned_on": False, "actually_on": False}, "8": {"turned_on": False, "actually_on": False}}

        self.timer = 0

        lamps_diagnostics_subscriber = rospy.Subscriber("diagnostics/lamps", LampsDiagnostics, self.lamps_diagnostics_callback)
        lamps_try_turn_on_service = rospy.Service("broken_lamp/notify_try_in_turn_on", BrokenLamp, self.try_turning_on_handler)

        self.broken_lamps_publisher = rospy.Publisher("broken_lamps", LampStates)

    def try_turning_on_handler(self, req):
        lamp_number = req.lamp_name

        for lamp in self.lamps:
            if self.lamps[lamp] == lamp_number and self.lamps[lamp]["turned_on"] == False:
                self.lamps[lamp]["turned_on"] = True
                
    def lamps_diagnostics_callback(self, data):
        lamp_1_on = data.lamp_states.lamp_1
        lamp_2_on = data.lamp_states.lamp_2
        lamp_3_on = data.lamp_states.lamp_3
        lamp_4_on = data.lamp_states.lamp_4
        lamp_5_on = data.lamp_states.lamp_5
        lamp_6_on = data.lamp_states.lamp_6
        lamp_7_on = data.lamp_states.lamp_7
        lamp_8_on = data.lamp_states.lamp_8

        for lamp in self.lamps:
            self.lamps[lamp]["actually_on"] = eval("lamp_" + lamp + "_on")

    def publish_broken_lamps(self):
        lamp_states = LampStates()
        
        for lamp in self.lamps:
            eval("lamp_states.lamp_" + lamp + " = " + str(self.lamps[lamp]["actually_on"]))

        self.broken_lamps_publisher.publish(lamp_states)
        
if __name__ == "__main__":
    try:
        
        broken_lamp_detector = BrokenLampDetector()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

