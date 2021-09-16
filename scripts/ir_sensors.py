#!/usr/bin/python

import rospy
from std_msgs.msg import String, Int16, Bool
from diagnostics.msg import IrSensorsDiagnostics

class DiagnosticsIrSensors():
    def __init__(self, diagnostics_ir_sensors_topic):
        self.node_handler = rospy.init_node('diagnostics_ir_sensors')
        
        self.topic_name = diagnostics_ir_sensors_topic

        self.ir_sensors = {}
        
        self.ir_sensor_diagnostics_publisher = rospy.Publisher(self.topic_name, IrSensorsDiagnostics, queue_size=10)

    def add_ir_sensor(self, sensor_name, topic_name):
        subscriber = rospy.Subscriber(topic_name, Int16, self.ir_sensors_callback, ( sensor_name ))
        
        self.ir_sensors[sensor_name] = {}
        self.ir_sensors[sensor_name]["value"] = None
        self.ir_sensors[sensor_name]["subscriber"] = subscriber

    def ir_sensors_callback(self, data, args):
        sensor_name = args[0]
        self.ir_sensors[sensor_name]["value"] = data.data

    def publish_diagnostics(self):
        message = IrSensorsDiagnostics()

        if (self.ir_sensors["1"]["value"] < 500 or self.ir_sensors["2"]["value"] < 500):
            message.message = String("Obstacle too close")
            message.ok = Bool(False)
        else:
            message.message = String("No obstacle present")
            message.ok = Bool(True)

        self.ir_sensor_diagnostics_publisher.publish(message)

if __name__ == "__main__":
    try:
        diagnostics_ir_sensors = DiagnosticsIrSensors("diagnostics/ir_sensors")
        diagnostics_ir_sensors.add_ir_sensor("1", "diagnostics/ir_sensor_1_value")
        diagnostics_ir_sensors.add_ir_sensor("2", "diagnostics/ir_sensor_2_value")

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            diagnostics_ir_sensors.publish_diagnostics()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

