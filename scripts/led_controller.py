#!/usr/bin/python

import enum
import rospy
from std_msgs.msg import String, Int16
from std_srvs.srv import Empty
from diagnostics.msg import IrSensorsDiagnostics, LampsDiagnostics, VoltageMeasurementsDiagnostics

class LedController():
    def __init__(self):
        self.node_handler = rospy.init_node('diagnostics_led_controller')

        self.lamps_that_are_on = {"1": False, "2": False, "3": False, "4": False,
                                "5": False, "6": False, "7": False, "8": False}

        self.lamps_and_drivers_diagnostics_status = None
        self.voltage_measurements_diagnostics_status = None
        self.ir_sensor_diagnostics_status = None
        self.emergency_button_pressed = None

        self.turn_leds_red = rospy.ServiceProxy("leds/turn_red", Empty)
        self.turn_leds_orange = rospy.ServiceProxy("leds/turn_orange", Empty)
        self.turn_leds_green = rospy.ServiceProxy("leds/turn_green", Empty)
        self.turn_leds_off = rospy.ServiceProxy("leds/turn_off", Empty)
    
        ir_sensors_diagnostics_subscriber = rospy.Subscriber("diagnostics/ir_sensors", IrSensorsDiagnostics, self.ir_sensors_diagnostics_callback)
        lamps_diagnostics_subscriber = rospy.Subscriber("diagnostics/lamps", LampsDiagnostics, self.lamps_diagnostics_callback)
        voltage_measurements_diagnostics_subscriber = rospy.Subscriber("diagnostics/voltage_measurements", VoltageMeasurementsDiagnostics, self.voltage_measurements_diagnostics_callback)

        emergency_buutton_status_subscriber = rospy.Subscriber("/emergencybt_status", Int16, self.emergency_button_callback)

        rospy.wait_for_message("diagnostics/ir_sensors", IrSensorsDiagnostics)
        rospy.wait_for_message("diagnostics/lamps", LampsDiagnostics)
        rospy.wait_for_message("diagnostics/voltage_measurements", VoltageMeasurementsDiagnostics)

    def emergency_button_callback(self, data):
        self.emergency_button_pressed = data.data

    def ir_sensors_diagnostics_callback(self, data):
        self.ir_sensor_diagnostics_status = data.ok.data            

    def voltage_measurements_diagnostics_callback(self, data):
        self.voltage_measurements_diagnostics_status = data.ok.data            

    def lamps_diagnostics_callback(self, data):
        for lamp_number in self.lamps_that_are_on:
            self.lamps_that_are_on[lamp_number] = eval("data.lamp_states.lamp_" + str(lamp_number) + ".data")

        self.lamps_and_drivers_diagnostics_status = data.ok.data            

    def at_least_one_lamp_is_on(self):
        for lamp, lamp_on in self.lamps_that_are_on.items():
            if lamp_on:
                return True
        
        return False

    def all_lamps_are_on(self):
        return self.lamps_and_drivers_diagnostics_status == True

    def voltages_are_ok(self):
        return self.voltage_measurements_diagnostics_status == True

    def no_obstacle_present(self):
        return self.ir_sensor_diagnostics_status == True

    def determine_and_send_leds_states(self):
        try:
            if self.at_least_one_lamp_is_on():
                self.turn_leds_red()
            
            elif not self.at_least_one_lamp_is_on():
            
                if self.voltages_are_ok() and self.no_obstacle_present():
                    self.turn_leds_green()

                else:
                    self.turn_leds_orange()

        except rospy.service.ServiceException as e:
            pass
if __name__ == "__main__":
    try:
        
        led_controller = LedController()

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            led_controller.determine_and_send_leds_states()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

