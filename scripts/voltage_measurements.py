#!/usr/bin/python

import rospy
from std_msgs.msg import Float32, String, Bool

from diagnostics_msgs.msg import VoltageMeasurementsDiagnostics
from voltage_measurements_circuit.voltage_measurements_circuit import VoltageMeasurementsCircuit

class DiagnosticsVoltageMeasurements():
    def __init__(self, diagnostics_voltage_measurements_topic):
        self.node_handler = rospy.init_node('diagnostics_voltage_measurements')
        
        self.voltage_measurements_circuit = VoltageMeasurementsCircuit()
        self.topic_name = diagnostics_voltage_measurements_topic

        self.voltage_measurements = {}
        self.voltage_measurements_publisher = rospy.Publisher(self.topic_name, VoltageMeasurementsDiagnostics, queue_size=10)

    def add_voltage_measurement(self, voltage_measurement_name, topic_name):
        subscriber = rospy.Subscriber(topic_name, Float32, self.voltage_measurements_callback, ( voltage_measurement_name ))
        
        self.voltage_measurements[voltage_measurement_name] = {}
        self.voltage_measurements[voltage_measurement_name]["value"] = None
        self.voltage_measurements[voltage_measurement_name]["subscriber"] = subscriber

    def voltage_measurements_callback(self, data, args):
        voltage_measurement_name = args
        self.voltage_measurements[voltage_measurement_name]["value"] = data.data
    
    def publish_diagnostics(self):

        message = VoltageMeasurementsDiagnostics()
        
        adc_value_12v_rail = self.voltage_measurements["12v"]["value"]
        adc_value_24v_rail = self.voltage_measurements["24v"]["value"]
        adc_value_battery = self.voltage_measurements["battery"]["value"]

        if (adc_value_12v_rail is not None and adc_value_24v_rail is not None and adc_value_battery is not None):
            self.voltage_measurements_circuit.calculate_voltages({"12v": adc_value_12v_rail, "24v": adc_value_24v_rail, "battery": adc_value_battery})

            message.voltage_12v_rail = Float32(self.voltage_measurements_circuit.get_12v_voltage())
            message.voltage_24v_rail = Float32(self.voltage_measurements_circuit.get_24v_voltage())
            message.voltage_battery = Float32(self.voltage_measurements_circuit.get_battery_voltage())
            
            if (self.voltage_measurements_circuit.get_battery_voltage() < 24.0 or self.voltage_measurements_circuit.get_battery_voltage() > 30):
                message.message = String("Battery voltage too low!")
                message.ok = Bool(False)
            else:
                message.message = String("All voltages are OK!")
                message.ok = Bool(True)
        else:
            message.message = String("No voltage readouts present yet")        

        self.voltage_measurements_publisher.publish(message)

if __name__ == "__main__":
    try:
        diagnostics_voltage_measurements = DiagnosticsVoltageMeasurements("diagnostics/voltage_measurements")
        diagnostics_voltage_measurements.add_voltage_measurement("12v", "diagnostics/adc_value_12v_rail")
        diagnostics_voltage_measurements.add_voltage_measurement("24v", "diagnostics/adc_value_24v_rail")
        diagnostics_voltage_measurements.add_voltage_measurement("battery", "diagnostics/adc_value_battery")

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            diagnostics_voltage_measurements.publish_diagnostics()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

