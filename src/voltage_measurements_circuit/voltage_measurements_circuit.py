#!/usr/bin/python

class VoltageMeasurementsCircuit():
    def __init__(self):
        self.__define_circuit()
        self.voltage_battery = 0
        self.voltage_12v_rail = 0
        self.voltage_24v_rail = 0

    def calculate_voltages(self, adc_output_values = None):
            
        offset_probably_due_to_opamp_power_to_measurement_too_small_difference = 0.7

        self.voltage_12v_rail = ( self.R5 + self.R6 ) / self.R5 * adc_output_values["12v"]
        self.voltage_24v_rail = ( self.R3 + self.R4 ) / self.R4 * adc_output_values["24v"] + offset_probably_due_to_opamp_power_to_measurement_too_small_difference
        self.voltage_battery = (adc_output_values["battery"] + (self.R1 / self.R2) * self.voltage_24v_rail) * (self.R7 + self.R8) / self.R8 * self.R2 / (self.R1 + self.R2)

        return self

    def get_24v_voltage(self):
        return self.voltage_24v_rail

    def get_12v_voltage(self):
        return self.voltage_12v_rail

    def get_battery_voltage(self):
        return self.voltage_battery

    def __define_circuit(self):
        self.__define_resistors_12v_rail()
        self.__define_resistor_values_24v_rail()
        self.__define_resistor_values_battery()

    def __define_resistors_12v_rail(self):
        self.R5 = 20e3
        self.R6 = 60e3
    
    def __define_resistor_values_24v_rail(self):
        self.R3 = 60e3
        self.R4 = 10e3
    
    def __define_resistor_values_battery(self):
        self.R1 = 22e3
        self.R2 = 47.5e3
        self.R7 = 46.4e3
        self.R8 = 24e3