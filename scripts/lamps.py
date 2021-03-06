#!/usr/bin/python

import rospy
from std_msgs.msg import Bool
from diagnostics_msgs.msg import LampsDiagnostics

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
        subscriber = rospy.Subscriber(topic_name, Bool, self.lamp_drivers_callback, ( lamp_driver_name ))
        
        self.lamp_drivers[lamp_driver_name] = {}
        self.lamp_drivers[lamp_driver_name]["value"] = None
        self.lamp_drivers[lamp_driver_name]["subscriber"] = subscriber

    def lamps_callback(self, data, args):
        lamp_name = args[0]
        self.lamps[lamp_name]["value"] = data.data

    def lamp_drivers_callback(self, data, args):
        lamp_driver_name = args[0]
        self.lamp_drivers[lamp_driver_name]["value"] = data.data

    def publish_diagnostics(self):
        
        message = LampsDiagnostics()

        lamp1On = self.lamps["1"]["value"]
        lamp2On = self.lamps["2"]["value"]
        lamp3On = self.lamps["3"]["value"]
        lamp4On = self.lamps["4"]["value"]
        lamp5On = self.lamps["5"]["value"]
        lamp6On = self.lamps["6"]["value"]
        lamp7On = self.lamps["7"]["value"]
        lamp8On = self.lamps["8"]["value"]

        driver1On = self.lamp_drivers["1"]["value"]
        driver2On = self.lamp_drivers["2"]["value"]
        driver3On = self.lamp_drivers["3"]["value"]
        driver4On = self.lamp_drivers["4"]["value"]

        message.lamp_states.lamp_1 = Bool(lamp1On)
        message.lamp_states.lamp_2 = Bool(lamp2On)
        message.lamp_states.lamp_3 = Bool(lamp3On)
        message.lamp_states.lamp_4 = Bool(lamp4On)
        message.lamp_states.lamp_5 = Bool(lamp5On)
        message.lamp_states.lamp_6 = Bool(lamp6On)
        message.lamp_states.lamp_7 = Bool(lamp7On)
        message.lamp_states.lamp_8 = Bool(lamp8On)

        if (lamp1On and lamp2On and lamp3On and lamp4On and lamp5On 
            and lamp6On and lamp7On and lamp8On):
            
            message.lamp_states.ok = Bool(True)
        else:
            message.lamp_states.ok = Bool(False)

        message.lamp_driver_states.lamp_driver_1 = Bool(driver1On)
        message.lamp_driver_states.lamp_driver_2 = Bool(driver2On)
        message.lamp_driver_states.lamp_driver_3 = Bool(driver3On)
        message.lamp_driver_states.lamp_driver_4 = Bool(driver4On)

        if (driver1On and driver2On and driver3On and driver4On):
            message.lamp_driver_states.ok = Bool(True)
        else:
            message.lamp_driver_states.ok = Bool(False)

        if (message.lamp_states.ok.data == True and message.lamp_driver_states.ok.data == True):
            message.ok = Bool(True)
        else:
            message.ok = Bool(False)
            
        self.lamps_diagnostics_publisher.publish(message)

if __name__ == "__main__":
    try:
        diagnostics_lamps = DiagnosticsLamps("diagnostics/lamps")
        diagnostics_lamps.add_lamp("1", "diagnostics/lamp_1_on")
        diagnostics_lamps.add_lamp("2", "diagnostics/lamp_2_on")
        diagnostics_lamps.add_lamp("3", "diagnostics/lamp_3_on")
        diagnostics_lamps.add_lamp("4", "diagnostics/lamp_4_on")
        diagnostics_lamps.add_lamp("5", "diagnostics/lamp_5_on")
        diagnostics_lamps.add_lamp("6", "diagnostics/lamp_6_on")
        diagnostics_lamps.add_lamp("7", "diagnostics/lamp_7_on")
        diagnostics_lamps.add_lamp("8", "diagnostics/lamp_8_on")

        diagnostics_lamps.add_lamp_driver("1", "diagnostics/driver_1_on")
        diagnostics_lamps.add_lamp_driver("2", "diagnostics/driver_2_on")
        diagnostics_lamps.add_lamp_driver("3", "diagnostics/driver_3_on")
        diagnostics_lamps.add_lamp_driver("4", "diagnostics/driver_4_on")

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            diagnostics_lamps.publish_diagnostics()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

