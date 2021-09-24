#!/usr/bin/python

import enum
import rospy
from std_msgs.msg import String, Int8
from std_srvs.srv import Empty
from diagnostics_srvs.srv import TurnLedOn, TurnLedOnRequest, TurnLedRed

class LedController():
    def __init__(self):
        self.node_handler = rospy.init_node('diagnostics_led_controller')

        self.leds = ["A1", "B1", "A2", "B2", "A3", "B3", "A4", "B4", "A5", "B5", "A6", "B6", "A7", "B7", "A8", "B8"]

        self.turn_led_on = rospy.ServiceProxy("led/turn_on", TurnLedOn)
        self.turn_led_on_request = TurnLedOnRequest()

        self.turn_front_leds_red_service = rospy.Service("leds/front/turn_red", Empty, self.turn_front_leds_red_handler)
        self.turn_all_leds_orange_service = rospy.Service("leds/turn_orange", Empty, self.turn_all_leds_orange_handler)
        self.turn_all_leds_green_service = rospy.Service("leds/turn_green", Empty, self.turn_all_leds_green_handler)
        self.turn_led_red_service = rospy.Service("led/turn_red", TurnLedRed, self.turn_led_red_handler)
    
    def turn_led_red_handler(self, req):
        self.turn_led_red(req.led_name)
        
    def turn_led_red(self, led_name):
        self.turn_led_on_request.led_name = String(led_name)
        self.turn_led_on_request.r = Int8(255)
        self.turn_led_on_request.g = Int8(0)

        response = self.turn_led_on(self.turn_led_on_request)

    def turn_led_green(self, led_name):        
        self.turn_led_on_request.led_name = String(led_name)
        self.turn_led_on_request.r = Int8(0)
        self.turn_led_on_request.g = Int8(255)

        response = self.turn_led_on(self.turn_led_on_request)
    
    def turn_led_orange(self, led_name):
        self.turn_led_on_request.led_name = String(led_name)
        self.turn_led_on_request.r = Int8(255)
        self.turn_led_on_request.g = Int8(255)

        response = self.turn_led_on(self.turn_led_on_request)
    
    def turn_led_off(self, led_name):
        self.turn_led_on_request.led_name = String(led_name)
        self.turn_led_on_request.r = Int8(0)
        self.turn_led_on_request.g = Int8(0)

        response = self.turn_led_on(self.turn_led_on_request)
    
    def turn_all_leds_off(self, req):
        for led_name in self.leds:
            self.turn_led_off(led_name)

    def turn_all_leds_orange_handler(self, req):
        for led_name in self.leds:
            self.turn_led_orange(led_name)

    def turn_all_leds_green_handler(self, req):
        for led_name in self.leds:
            self.turn_led_green(led_name)
    
    def turn_front_leds_red_handler(self, req):
        self.turn_led_red("A1")
        self.turn_led_red("A5")
        self.turn_led_red("B5")

if __name__ == "__main__":
    try:
        
        led_controller = LedController()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

