#!/usr/bin/python

import enum
import rospy
from std_msgs.msg import String, Int16
from std_srvs.srv import Empty
from diagnostics_msgs.msg import IrSensorsDiagnostics, LampsDiagnostics, VoltageMeasurementsDiagnostics
from diagnostics_srvs.srv import TurnLedRed

class Diagnostics():
    def __init__(self):
        self.node_handler = rospy.init_node('diagnostics')


        self.lamps_working = {"1": False, "2": False, "3": False, "4": False,
                                "5": False, "6": False, "7": False, "8": False}

        self.lamps_and_drivers_diagnostics_status = None
        self.voltage_measurements_diagnostics_status = None
        self.emergency_button_pressed = None

        self.turn_front_leds_red = rospy.ServiceProxy("leds/front/turn_red", Empty)
        self.turn_all_leds_orange = rospy.ServiceProxy("leds/turn_orange", Empty)
        self.turn_all_leds_green = rospy.ServiceProxy("leds/turn_green", Empty)
        self.turn_led_red = rospy.ServiceProxy("led/turn_red", TurnLedRed)

        lamps_diagnostics_subscriber = rospy.Subscriber("diagnostics/lamps", LampsDiagnostics, self.lamps_diagnostics_callback)
        voltage_measurements_diagnostics_subscriber = rospy.Subscriber("diagnostics/voltage_measurements", VoltageMeasurementsDiagnostics, self.voltage_measurements_diagnostics_callback)
        emergency_button_status_subscriber = rospy.Subscriber("/emergencybt_status", Int16, self.emergency_button_callback)

    def emergency_button_callback(self, data):
        self.emergency_button_pressed = data.data

    def voltage_measurements_diagnostics_callback(self, data):
        self.voltage_measurements_diagnostics_status = data.ok.data            

    def lamps_diagnostics_callback(self, data):
        for lamp_number in self.lamps_working:
            self.lamps_working[lamp_number] = eval("data.lamp_states.lamp_" + str(lamp_number) + ".data")

        self.lamps_and_drivers_diagnostics_status = data.ok.data            

    def are_voltage_levels_ok(self):
        return self.voltage_measurements_diagnostics_status == True
    
    def is_lamp_not_working(self, lamp_name):
        for lamp in self.lamps_working:
            if self.lamps_working[lamp] == False and lamp == lamp_name:
                return True
        
        return False
    
    def all_lamps_working(self):
        return self.lamps_and_drivers_diagnostics_status == True
            
    def is_emergency_button_pressed(self):
        return self.emergency_button_pressed == True

    def determine_and_send_lamp_states(self):
        if (self.is_emergency_button_pressed()):
            self.turn_all_leds_orange()
        else:
            if (self.are_voltage_levels_ok() and self.all_lamps_working()):
                self.turn_all_leds_green()
            elif (self.are_voltage_levels_ok() and not self.all_lamps_working()):
                for lamp in self.lamps_working:
                    if self.is_lamp_not_working(lamp):
                        self.turn_led_red(lamp)

if __name__ == "__main__":
    try:
        
        diagnostics = Diagnostics()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            diagnostics.determine_and_send_leds_states()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

