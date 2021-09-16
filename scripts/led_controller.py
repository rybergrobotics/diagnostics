#!/usr/bin/python

import rospy
from std_msgs.msg import String, Int16
from std_srvs.srv import Empty
from diagnostics.msg import IrSensorsDiagnostics, LampsDiagnostics, VoltageMeasurementsDiagnostics

class LedController():
    def __init__(self):
        self.node_handler = rospy.init_node('diagnostics_led_controller')
        
        self.sensor_diagnostics_state = {"ir_sensors_ok": False, "voltage_measurements_ok": False, "lamps_and_drivers_on": False}
        
        self.turn_leds_red = rospy.ServiceProxy("leds/turn_red", Empty)
        self.turn_leds_orange = rospy.ServiceProxy("leds/turn_orange", Empty)
        self.turn_leds_green = rospy.ServiceProxy("leds/turn_green", Empty)
        self.turn_leds_off = rospy.ServiceProxy("leds/turn_off", Empty)
    
        ir_sensors_diagnostics_subscriber = rospy.Subscriber("diagnostics/ir_sensors", IrSensorsDiagnostics, self.ir_sensors_diagnostics_callback)
        lamps_diagnostics_subscriber = rospy.Subscriber("diagnostics/lamps", LampsDiagnostics, self.lamps_diagnostics_callback)
        voltage_measurements_diagnostics_subscriber = rospy.Subscriber("diagnostics/voltage_measurements", VoltageMeasurementsDiagnostics, self.voltage_measurements_diagnostics_callback)

        rospy.wait_for_message("diagnostics/ir_sensors", IrSensorsDiagnostics)
        rospy.wait_for_message("diagnostics/lamps", LampsDiagnostics)
        rospy.wait_for_message("diagnostics/voltage_measurements", VoltageMeasurementsDiagnostics)


    def ir_sensors_diagnostics_callback(self, data):
        self.sensor_diagnostics_state["ir_sensors_ok"] = data.ok.data            

    def voltage_measurements_diagnostics_callback(self, data):
        self.sensor_diagnostics_state["voltage_measurements_ok"] = data.ok.data            

    def lamps_diagnostics_callback(self, data):
        self.sensor_diagnostics_state["lamps_and_drivers_on"] = data.ok.data            

    def determine_and_send_leds_states(self):
        try:
            if self.sensor_diagnostics_state["lamps_and_drivers_on"] == True:
                self.turn_leds_red()
            
            elif self.sensor_diagnostics_state["lamps_and_drivers_on"] == False:
            
                if self.sensor_diagnostics_state["voltage_measurements_ok"] == True and self.sensor_diagnostics_state["ir_sensors_ok"] == True:
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

