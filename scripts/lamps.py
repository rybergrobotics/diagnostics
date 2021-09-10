#!/usr/bin/python

import rospy
from std_msgs.msg import Bool
from diagnostics.msg import LampsDiagnostics

class DiagnosticsLamps():
    def __init__(self, diagnostics_lamps_topic):
        self.node_handler = rospy.init_node('diagnostics_lamps')
        
        self.topic_name = diagnostics_lamps_topic

        self.lamps = {}
        self.lamp_drivers = {}

        self.lamps_diagnostics_publisher = rospy.Publisher(self.topic_name, LampsDiagnostics, queue_size=10)

    def add_lamp(self, lamp_name, topic_name):
        subscriber = rospy.Subscriber(topic_name, Bool, self.lamps_callback, ( lamp_name ))
        
        self.lamps[lamp_name] = {}
        self.lamps[lamp_name]["value"] = None
        self.lamps[lamp_name]["subscriber"] = subscriber

    def add_lamp_driver(self, lamp_driver_name, topic_name):
        subscriber = rospy.Subscriber(topic_name, Bool, self.lamps_callback, ( lamp_driver_name ))
        
        self.lamps[lamp_driver_name] = {}
        self.lamps[lamp_driver_name]["value"] = None
        self.lamps[lamp_driver_name]["subscriber"] = subscriber

    def lamps_callback(self, data, args):
        lamp_name = args[0]
        self.lamps[lamp_name]["value"] = data.data

    def publish_diagnostics(self):
        
        message = LampsDiagnostics()
        message.lamp_states.lamp_1 = Bool(self.lamps["1"]["value"])
        message.lamp_states.lamp_2 = Bool(self.lamps["2"]["value"])
        message.lamp_states.lamp_3 = Bool(self.lamps["3"]["value"])
        message.lamp_states.lamp_4 = Bool(self.lamps["4"]["value"])
        message.lamp_states.lamp_5 = Bool(self.lamps["5"]["value"])
        message.lamp_states.lamp_6 = Bool(self.lamps["6"]["value"])
        message.lamp_states.lamp_7 = Bool(self.lamps["7"]["value"])
        message.lamp_states.lamp_8 = Bool(self.lamps["8"]["value"])

        message.lamp_driver_states.lamp_driver_1 = Bool(self.lamps["1"]["value"])
        message.lamp_driver_states.lamp_driver_2 = Bool(self.lamps["2"]["value"])
        message.lamp_driver_states.lamp_driver_3 = Bool(self.lamps["3"]["value"])
        message.lamp_driver_states.lamp_driver_4 = Bool(self.lamps["4"]["value"])

        self.lamps_diagnostics_publisher.publish(message)

if __name__ == "__main__":
    try:
        diagnostics_lamps = DiagnosticsLamps("diagnostics/lamps")
        diagnostics_lamps.add_lamp("1", "diagnostics/lamp_1_value")
        diagnostics_lamps.add_lamp("2", "diagnostics/lamp_2_value")
        diagnostics_lamps.add_lamp("3", "diagnostics/lamp_3_value")
        diagnostics_lamps.add_lamp("4", "diagnostics/lamp_4_value")
        diagnostics_lamps.add_lamp("5", "diagnostics/lamp_5_value")
        diagnostics_lamps.add_lamp("6", "diagnostics/lamp_6_value")
        diagnostics_lamps.add_lamp("7", "diagnostics/lamp_7_value")
        diagnostics_lamps.add_lamp("8", "diagnostics/lamp_8_value")

        diagnostics_lamps.add_lamp_driver("1", "diagnostics/driver_1_value")
        diagnostics_lamps.add_lamp_driver("2", "diagnostics/driver_2_value")
        diagnostics_lamps.add_lamp_driver("3", "diagnostics/driver_3_value")
        diagnostics_lamps.add_lamp_driver("4", "diagnostics/driver_4_value")

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            diagnostics_lamps.publish_diagnostics()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

