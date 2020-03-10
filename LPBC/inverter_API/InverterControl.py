'''
Inverter Control at FLexgrid (v1.0 - 10/26/2018)
Christoph Gehbauer (Lawrence Berkeley National Laboratory)
cgehbauer@lbl.gov
'''

from pymodbus.client.sync import  ModbusSerialClient
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian
from Development.convert_data import *
from time import sleep

import struct

class ModbusRTUClient(ModbusSerialClient):
    def __init__(self, portName='com4', baudrate=115200):
        ModbusSerialClient.__init__(self,"rtu", port=portName, baudrate=baudrate, timeout=1)

    #### Functions for MODBUS communication ####

    def read_register_int16(self, address, inv_id):
        return data_to_int16(self.read_holding_registers(address, count=1, unit=inv_id).registers)

    def read_register_uint16(self, address, inv_id):
        return data_to_uint16(self.read_holding_registers(address, count=1, unit=inv_id).registers)

    def read_register_float32(self, address, inv_id):
        res = self.read_holding_registers(address, count=2, unit=inv_id).registers
        return BinaryPayloadDecoder.fromRegisters(res, byteorder=Endian.Little).decode_32bit_float()
 
        #return data_to_float32_inv(client.read_holding_registers(address, count=2, unit=inv_id).registers)

    def read_register_uint32(self, address, inv_id):
        return data_to_uint32(self.read_holding_registers(address, count=2, unit=inv_id).registers)

    def read_register_int32(self, address, inv_id):
        return data_to_int32(self.read_holding_registers(address, count=2, unit=inv_id).registers)


#### Functions for framework ####

def debug_printing(readings=[]):
    '''
        Function to print debug data as tables.
        
        Inputs:
            readings - A Python array of equally indexed Python dictionaries.
    '''
    for k in readings[0].keys():
        print("{:<25}".format(k)[0:25]), 
        for i in range(len(readings)):
            print("{:<10}".format(readings[i][k])[0:10]),
        print


class Inverter(object):
    def __init__(self, inv_id = 1, client = ModbusRTUClient):
        self.inv_id = inv_id
        self.client = client
        self.maxPower = 7600 # Watts
    
    def connect_modbus(self):
        self.client.connect()
    
    def close_modbus(self):
        self.client.close()

    ##### Functions for general operations ####

    def test_communication(self):
        '''
            Function to test the communiction with the inverter.
            
            Returns:
                error - "ok" if connection works, "error 1" or "error 2" if error
        '''
        if self.client.read_register_uint16(40002, self.inv_id) == 1:
            if self.client.read_register_uint32(40000, self.inv_id) == 1400204883:
                return 'ok'
            else:
                return 'error 2'
        else:
            return 'error 1'
    
    def read_inverter_status(self, msg=False):
        '''
            Function to get the status of the inverter.
            
            Returns:
                status - "off", "sleep", "on", "NA", "unknown"
        '''
        # Inverter Status [1-off, 2-sleep, 4-on] unit16 (40107) 
        status = self.client.read_register_uint16(40107,self.inv_id)
        if not msg: 
            return status
        else:
            status_msg = {1:'off', 2:'sleep', 4:'on'}
            return status_msg.get(status, 'unknown')

        
    def read_inverter_telemetry(self):
        '''
            Function to read inverter telemetry data.
            
            Returns:
                data - A Python dictionary with readings.
        '''
        data = {}    
        # Inverter Status Code [1-4]
        data['Inv_Stat_1'] = self.read_inverter_status(msg=False)
        # Temperature [C] int16 (40103)
        scale = self.client.read_register_int16(40106, self.inv_id)
        data['Inv_T_C'] = self.client.read_register_int16(40103, self.inv_id)*10**scale
        # Total Current [A] uint16 40071
        scale = self.client.read_register_int16(40075, self.inv_id)
        data['AC_I_A'] = self.client.read_register_uint16(40071, self.inv_id)*10**scale
        # Voltage [V] uint16 (40076,40077,40078)
        scale = self.client.read_register_int16(40082, self.inv_id)
        data['AC_U-AB_V'] = self.client.read_register_uint16(40076, self.inv_id)*10**scale
        # Real Power [Watt] int16 (40083)
        scale = self.client.read_register_int16(40084, self.inv_id)
        data['AC_P_W'] = self.client.read_register_int16(40083, self.inv_id)*10**scale
        # Frequency [Hz] unit16 (40085)
        scale = self.client.read_register_int16(40086, self.inv_id)
        data['AC_F_Hz'] = self.client.read_register_uint16(40085, self.inv_id)*10**scale
        # Apparent Power [VA] int16 (40087)
        scale = self.client.read_register_int16(40088, self.inv_id)
        data['AC_S_VA'] = self.client.read_register_int16(40087, self.inv_id)*10**scale
        # Reactive Power [Var] int16 (40089)
        scale = self.client.read_register_int16(40090, self.inv_id)
        data['AC_Q_Var'] = self.client.read_register_int16(40089, self.inv_id)*10**scale
        # Power Factor [%] int16 (40091)
        scale = self.client.read_register_int16(40092, self.inv_id)
        data['AC_PF_%'] = self.client.read_register_int16(40091, self.inv_id)*10**scale
        # DC Current [A] unit16 (40096)
        scale = self.client.read_register_int16(40097, self.inv_id)
        data['DC_I_A'] = self.client.read_register_uint16(40096, self.inv_id)*10**scale
        # DC Voltage [V] unit16 (40098)
        scale = self.client.read_register_int16(40099, self.inv_id)
        data['DC_U_V'] = self.client.read_register_uint16(40098, self.inv_id)*10**scale
        # DC Power [W] int16 (40100)
        scale = self.client.read_register_int16(40101, self.inv_id)
        data['DC_P_W'] = self.client.read_register_int16(40100, self.inv_id)*10**scale
        return data

    def read_battery_telemetry(self):
        '''
            Function to read battery telemetry data.
            
            Returns:
                data - A Python dictionary with readings.
        '''
        data = {}
        # Battery Temperature [C] float32 (0xF56C)
        data['Batt_T_C'] = self.client.read_register_float32(0xF56C, self.inv_id)
        # Battery Voltage [V] float32 (0xF570)
        data['Batt_U_V'] = self.client.read_register_float32(0xF570, self.inv_id)
        # Battery Current [A] float32 (0xF572)
        data['Batt_I_A'] = self.client.read_register_float32(0xF572, self.inv_id)
        # Battery Power [W] float32 (0xF574)
        data['Batt_P_W'] = self.client.read_register_float32(0xF574, self.inv_id)
        # Battery State of Health [1] float32 (0xF582)
        data['Batt_SOH_1'] = self.client.read_register_float32(0xF582, self.inv_id)
        # Battery State of Charge [1] float32 (0xF584)
        data['Batt_SOC_1'] = self.client.read_register_float32(0xF584, self.inv_id)
        return data
        
    def read_all_telemetry(self):
        '''
            Function to read all telemetry data of the inverter.
            
            Returns:
                data - A Python dictionary with readings.
        '''
        data = {}
        data.update(self.read_inverter_telemetry())
        data.update(self.read_battery_telemetry())
        return data
    
    def read_power_parameter(self):
        '''
            Function to read all registers relevant to power control.
            
            Returns:
                data - A Python dictionary with readings.
        '''
        data = {}
        data['RRCR State'] = self.client.read_register_uint16(0xF000, self.inv_id)
        # AdvancedPwrControlEn [0/1] - R/W 0xF142
        data['Advanced Power Control'] = self.client.read_register_int16(0xF142, self.inv_id)
        # Active Power Control [%] - R/W 0xF001
        data['Active Power Control'] = self.client.read_register_uint16(0xF001, self.inv_id)
        # ReactivePwrConfig [4-RRCR, 0-fixed cosphi] - R/W 0xF104
        data['Reactive Power Control'] = self.client.read_register_int16(0xF104, self.inv_id)
        # Power Factor Control [+/-1] - R/W 0xF002
        data['Power Factor Control'] = self.client.read_register_float32(0xF002, self.inv_id)
        return data

    def read_battery_parameter(self):
        '''
            Function to read all registers relevant to battery control.
            
            Returns:
                data - A Python dictionary with readings.
        '''
        data = {}
        # Storage Control Mode - R/W 0xF704 0-disabled, 4-remote
        data['Control Mode'] = self.client.read_register_uint16(0xF704, self.inv_id)
        # Storage AC Charge Policy - R/W 0xF705 0-no AC charge, 1-AC charge
        data['AC Charge'] = self.client.read_register_uint16(0xF705, self.inv_id)
        # Storage Backup Reserved Setting - R/W 0xF708 in %
        data['Reserved Capacity'] = self.client.read_register_float32(0xF708, self.inv_id)
        # Storage Charge/Discharge Default Mode - R/W 0xF70A 0-off when timeout
        data['Default Mode'] =  self.client.read_register_uint16(0xF70A, self.inv_id)
        # Storage Remote Control Command Timeout - R/W 0xF70B timeout in seconds
        data['Command Timeout'] =  self.client.read_register_uint16(0xF70B, self.inv_id)
        # Storage Remote Control Command Mode - R/W 0xF70D OR 0xF70C??? 3-charge(PV+AC), 4-Discharge(PV+Batt)
        data['Command Mode'] =  self.client.read_register_uint16(0xF70D, self.inv_id)
        # Storage Remote Control Charge Limit - R/W 0xF70E charge limit W
        data['Charge Limit'] =  self.client.read_register_float32(0xF70E, self.inv_id)
        # Storage Remote Control Command Discharge Limit - R/W 0xF710 discharge limit W
        data['Discharge Limit'] =  self.client.read_register_float32(0xF710, self.inv_id)
        return data

    def read_site(self):
        '''
        Function to read all telemetry / parameter data of the site.
        
        Returns:
            data - A Python dictionary with readings.
        '''
        data = {}
        data.update(self.read_inverter_telemetry())
        data.update(self.read_battery_telemetry())
        data.update(self.read_power_parameter())
        data.update(self.read_battery_parameter())
        return data
    
    def read_soc(self):
        '''
            Function to read battery soc of the site.
            
            Returns:
                data - A Python dictionary with readings.
        '''
        data= {'Batt_SOC_1':self.client.read_register_float32(0xF584, self.inv_id)}
        return data

    #### Functions for power control ####

    def enable_power_control(self, debug=False):
        '''
            Enable the remote control of the inverter for P and pf.
        '''
        if debug: read_1 = read_power_parameter(client,self.inv_id)
        # Active Power Control [%] - R/W 0xF001
        self.client.write_register(0xF001, 0, unit=self.inv_id)
        sleep(2)
        # AdvancedPwrControlEn [0/1] - R/W 0xF142
        self.client.write_register(0xF142, 1, unit=self.inv_id) # 0-Disable; 1-Enable 
        # ReactivePwrConfig [4-RRCR, 0-fixed cosphi] - R/W 0xF104
        self.client.write_register(0xF104, 4, unit=self.inv_id) # 0-Disable; 4-RRCR
        # Enable Dynamic Power Control - R/W 0xF300 (not necessary, 0 is default) 
        self.client.write_register(0xF300, 0, unit=self.inv_id) # 0-Disable; 1-Enable
        # Power Reduce [0-100] - R/W 0xF140
        self.client.write_registers(0xF140, float32_to_data_inv(100), unit=self.inv_id)
        # Commit Power Control Settings? write=1 - R/W 0xF100 
        self.client.write_register(0xF100, 1, unit=self.inv_id)
        sleep(15) # Wait 15s before read back (should be =0 no error)
        # Active Power Control [%] - R/W 0xF001
        self.client.write_register(0xF001, 100, unit=self.inv_id)
        # Power Factor Control [+/-1] - R/W 0xF002
        self.client.write_registers(0xF002, float32_to_data_inv(1), unit=self.inv_id)
        if self.client.read_register_int16(0xF100, self.inv_id) != 0: 
            print('error')
        if debug: 
            debug_printing([read_1, read_power_parameter(client,self.inv_id)])

    def disable_power_control(self, debug=False):
        '''
            Disable the remote control of the inverter for P and pf.
        '''
        if debug: read_1 = self.read_power_parameter()
        # Active Power Control [%] - R/W 0xF001
        self.client.write_register(0xF001, 0, unit=self.inv_id)
        sleep(2)
        
        #### TODO Check if really reset ####

        # Restore Power Control Default Settings? write=1 - R/W 0xF101 
        self.client.write_register(0xF101, 1, unit=self.inv_id)
        # Wait 15s before read back (should be =0 no error)
        sleep(15)
        if self.client.read_register_int16(0xF101, self.inv_id) != 0: print('error')
        # AdvancedPwrControlEn [0/1] - R/W 0xF142
        self.client.write_register(0xF142, 0, unit=self.inv_id) # 0-Disable; 1-Enable 
        # ReactivePwrConfig [4-RRCR, 0-fixed cosphi] - R/W 0xF104
        self.client.write_register(0xF104, 0, unit=self.inv_id) # 0-Disable; 4-RRCR
        # Active Power Control [%] - R/W 0xF001
        self.client.write_register(0xF001, 100, unit=self.inv_id)
        # Power Factor Control [+/-1] - R/W 0xF002
        self.client.write_registers(0xF002, float32_to_data_inv(1), unit=self.inv_id)
        if debug: debug_printing([read_1, self.read_power_parameter()])
        
    def set_P(self, P, debug=False):
        '''
            Set active power limit in % of rated capacity.
        '''
        # Protection 
        if P < 0 or P > 100:
            print('error - out of range')
            return

        # Active Power Control [%] - R/W 0xF001
        if debug: read_1 = self.read_power_parameter()
        self.client.write_register(0xF001, int(P), unit=self.inv_id)
        if self.client.read_register_uint16(0xF001,self.inv_id) != P: print('error')
        if debug: debug_printing([read_1, read_power_parameter(client,self.inv_id)])
            
    def set_pf(self, pf, safety=True, debug=False):
        '''
            Set pf of the inverter.
        '''
        if pf < -1 or pf > 1:
            print('error - out of range')
            return
        # Power Factor Control [+/-1] - R/W 0xF002
        if debug: read_1 = read_power_parameter(client,self.inv_id)
        if safety:
            # read current P
            # Real Power [Watt] int16 (40083)
            scale = self.client.read_register_int16(40084, self.inv_id)
            p_power = self.client.read_register_int16(40083, self.inv_id)*10**scale
            p_power_margin = 1.2 * p_power
            if pf < p_power_margin / self.maxPower: 
                pf = p_power_margin / self.maxPower
        self.client.write_registers(0xF002, float32_to_data_inv(pf), unit=self.inv_id)
        #if round(self.client.read_register_float32(0xF002,self.self.inv_id),1) != round(pf,1): print('error')
        if debug: debug_printing([read_1, read_power_parameter(client,self.inv_id)])
        
    # Dynamic Power Control Block

    def enable_dyn_power_control(self, debug=False):
        '''
            Enable the remote control of the inverter for P and pf.
            Enhanced Dynamic Power Control Block
        '''
        # First enable regular power control
        if debug: read_1 = self.read_power_parameter()
        #Ramp Down Active Power Control [%] - R/W 0xF001
        self.client.write_register(0xF001, 0, unit=self.inv_id)
        sleep(2)
        # AdvancedPwrControlEn [0/1] - R/W 0xF142
        self.client.write_register(0xF142, 1, unit=self.inv_id) # 0-Disable; 1-Enable 
        # ReactivePwrConfig [4-RRCR, 0-fixed cosphi] - R/W 0xF104
        self.client.write_register(0xF104, 4, unit=self.inv_id) # 0-Disable; 4-RRCR
        # Power Reduce [0-100] - R/W 0xF140
        self.client.write_registers(0xF140, float32_to_data_inv(100), unit=self.inv_id)
        # Commit Power Control Settings? write=1 - R/W 0xF100 
        self.client.write_register(0xF100, 1, unit=self.inv_id)
        sleep(15) # Wait 15s before read back (should be =0 no error)
        if self.client.read_register_int16(0xF100,self.inv_id) != 0: print('error')
        if debug: debug_printing([read_1, read_power_parameter(client,self.inv_id)])
        
        # Configure Advanced Control 

        # Enable Active Power (1) or Reactive Power (0) Preference - R/W 0xF308 
        self.client.write_register(0xF308, 1, unit=self.inv_id) # 0-Q ; 1-P
        # CosPhi (0) or Q (1) Preference - R/W 0xF309 
        self.client.write_register(0xF309, 1, unit=self.inv_id) # 0-CosPhi (default); 1-Q
        # Set Max Inverter Active Power (W)
        self.client.write_registers(0xF30C, float32_to_data_inv(7000), unit=self.inv_id)
        # Set Max Inverter Reactive Power (VAR)
        self.client.write_registers(0xF30E, float32_to_data_inv(7000), unit=self.inv_id)
        # Set Timeout
        self.client.write_registers(0xF310, uint32_to_data(3600), unit=self.inv_id)
        # Set fallback P
        self.client.write_registers(0xF30C, float32_to_data_inv(7000), unit=self.inv_id)
        # Set Fallback CosPhi
        self.client.write_registers(0xF30C, float32_to_data_inv(1), unit=self.inv_id)
        # Set P ramp up rate
        self.client.write_registers(0xF30C, float32_to_data_inv(100), unit=self.inv_id)
        # Set P ramp down rate
        self.client.write_registers(0xF30C, float32_to_data_inv(100), unit=self.inv_id) 
        # Set Q ramp up rate
        self.client.write_registers(0xF30C, float32_to_data_inv(100), unit=self.inv_id)
        # Set Q ramp down rate
        self.client.write_registers(0xF30C, float32_to_data_inv(100), unit=self.inv_id)
        # Set CosPhi rate
        self.client.write_registers(0xF30C, float32_to_data_inv(3.1415), unit=self.inv_id) # set to Pi
        
        # Enable Dynamic Power Control - R/W 0xF300 # 0-Disable; 1-Enable
        self.client.write_register(0xF300, 1, unit=self.inv_id) 

    def disable_dyn_power_control(self, debug=False):
        '''
            Disable the dynamic control of the inverter for P and Q.
        '''
        # Disable Dynamic Power Control - R/W 0xF300 # 0-Disable; 1-Enable
        self.client.write_register(0xF300, 0, unit=self.inv_id)

    def set_dyn_P(self, P, debug=False):
        '''
            Set active power limit in Watts
        '''
        if P < 0 or P > 100:
            print('error - out of range')
            return
        client.write_registers(0xF322, float32_to_data_inv(P), unit=self.inv_id)

    def set_dyn_Q(self, Q, safety = True, debug=False):
        '''
            Set active power limit in Watts
        '''
        if Q < 0 or Q > 100:
            print('error - out of range')
            return
        if safety:
            # read current P
            # Real Power [Watt] int16 (40083)
            scale = self.client.read_register_int16(40084, self.inv_id)
            p_power = self.client.read_register_int16(40083, self.inv_id)*10**scale
            p_power_margin = 1.2 * p_power
            q_power_max = self.client.read_register_float32(0xF30E, self.inv_id)
            q_power = Q / 100 * q_power_max
            s_power_request = (p_power_margin**2 + q_power**2)**(1/2)
            if  s_power_request > self.maxPower: 
                new_q_power = (self.maxPower**2 - p_power_margin**2)**(1/2)
                Q = new_q_power / q_power_max * 100
        client.write_registers(0xF324, float32_to_data_inv(Q), unit=self.inv_id)


    # Functions for Battery Control
            
    def enable_battery_control(self, batt_low=10, debug=False):
        '''
            Setup the configuration to control the battery.
        '''
        if debug: read_1 = self.read_battery_parameter()
        # Storage Control Mode - R/W 0xF704 0-disabled, 4-remote
        self.client.write_register(0xF704, 4, unit=self.inv_id)
        # Storage AC Charge Policy - R/W 0xF705 0-no AC charge, 1-AC charge
        self.client.write_register(0xF705, 1, unit=self.inv_id)
        # Storage Backup Reserved Setting - R/W 0xF708 in %
        self.client.write_registers(0xF708, float32_to_data_inv(batt_low), unit=self.inv_id)
        # Storage Charge/Discharge Default Mode - R/W 0xF70A 0-off when timeout
        self.client.write_register(0xF70A, 0, unit=self.inv_id)
        # Storage Remote Control Command Timeout - R/W 0xF70B timeout in seconds
        self.client.write_register(0xF70B, 3600, unit=self.inv_id)
        # Storage RRemote Control Command Mode - R/W 0xF70D OR 0xF70C??? 3-charge(PV+AC), 4-Discharge(PV+Batt)
        self.client.write_register(0xF70D, 0, unit=self.inv_id)
        # Storage Remote Control Charge Limit - R/W 0xF70E charge limit W
        self.client.write_registers(0xF70E, float32_to_data_inv(3300), unit=self.inv_id)
        # Storage Remote Control Command Discharge Limit - R/W 0xF710 discharge limit W
        self.client.write_registers(0xF710, float32_to_data_inv(3300), unit=self.inv_id)
        if debug: debug_printing([read_1, read_battery_parameter(client,self.inv_id)])
        
    def disable_battery_control(self, debug=False):
        '''
            Reset all parameter for battery control.
        '''
        if debug: read_1 = read_battery_parameter()
        # Storage Control Mode - R/W 0xF704 0-disabled, 4-remote
        self.client.write_register(0xF704, 0, unit=self.inv_id)
        # Storage AC Charge Policy - R/W 0xF705 0-no AC charge, 1-AC charge
        self.client.write_register(0xF705, 1, unit=self.inv_id)
        # Storage Backup Reserved Setting - R/W 0xF708 in %
        self.client.write_registers(0xF708, float32_to_data_inv(25), unit=self.inv_id)
        # Storage Charge/Discharge Default Mode - R/W 0xF70A 0-off when timeout
        self.client.write_register(0xF70A, 0, unit=self.inv_id)
        # Storage Remote Control Command Timeout - R/W 0xF70B timeout in seconds
        self.client.write_register(0xF70B, 3600, unit=self.inv_id)
        # Storage RRemote Control Command Mode - R/W 0xF70D OR 0xF70C??? 3-charge(PV+AC), 4-Discharge(PV+Batt)
        self.client.write_register(0xF70D, 0, unit=self.inv_id)
        # Storage Remote Control Charge Limit - R/W 0xF70E charge limit W
        self.client.write_registers(0xF70E, float32_to_data_inv(3300), unit=self.inv_id)
        # Storage Remote Control Command Discharge Limit - R/W 0xF710 discharge limit W
        self.client.write_registers(0xF710, float32_to_data_inv(3300), unit=self.inv_id)
        if debug: debug_printing([read_1, self.read_battery_parameter()])
            
    def battery_control(self, P, debug=False):
        '''
            Function to control the battery output.
            
            Inputs:
                P - Battery power in W (positive is charge, negative is discharge)
        '''
        if debug: read_1 = read_battery_parameter()
        if P > 0:
            # Charge
            # Storage Remote Control Charge Limit - R/W 0xF70E charge limit W
            self.client.write_registers(0xF70E, float32_to_data_inv(float(P)), unit=self.inv_id)
            # Storage Remote Control Command Discharge Limit - R/W 0xF710 discharge limit W
            self.client.write_registers(0xF710, float32_to_data_inv(0), unit=self.inv_id)
            # Storage RRemote Control Command Mode - R/W 0xF70D OR 0xF70C??? 3-charge(PV+AC), 4-Discharge(PV+Batt)
            self.client.write_register(0xF70D, 3, unit=self.inv_id)
        elif P < 0:
            # Discharge
            # Storage Remote Control Charge Limit - R/W 0xF70E charge limit W
            self.client.write_registers(0xF70E, float32_to_data_inv(0), unit=self.inv_id)
            # Storage Remote Control Command Discharge Limit - R/W 0xF710 discharge limit W
            self.client.write_registers(0xF710, float32_to_data_inv(float(P*-1)), unit=self.inv_id)
            # Storage RRemote Control Command Mode - R/W 0xF70D OR 0xF70C??? 3-charge(PV+AC), 4-Discharge(PV+Batt)
            self.client.write_register(0xF70D, 4, unit=self.inv_id)
        else:
            # Rest
            # Storage RRemote Control Command Mode - R/W 0xF70D OR 0xF70C??? 3-charge(PV+AC), 4-Discharge(PV+Batt)
            self.client.write_register(0xF70D, 0, unit=self.inv_id)
        if debug: debug_printing([read_1, self.read_battery_parameter()])
        
    def battery_max_export(self, P, debug=False):
        '''
            Function to enable fast control of the battery output.
            
            Inputs:
                P - maximal Battery power in W for charging and discharging.
                    Should be higher than intended maximal power to avoid PV fluctuation.
        '''
        if debug: read_1 = read_battery_parameter(client,self.inv_id)
        P = int(abs(P))
        if P != 0:
            # set Bounds for Batt participation
            # Storage Remote Control Charge Limit - R/W 0xF70E charge limit W
            self.client.write_registers(0xF70E, float32_to_data_inv(P), unit=self.inv_id)
            # Storage Remote Control Command Discharge Limit - R/W 0xF710 discharge limit W
            self.client.write_registers(0xF710, float32_to_data_inv(P), unit=self.inv_id)
            # Storage RRemote Control Command Mode - R/W 0xF70D OR 0xF70C??? 3-charge(PV+AC), 4-Discharge(PV+Batt)
            self.client.write_register(0xF70D, 4, unit=self.inv_id)
        else:
            # Rest
            # Storage RRemote Control Command Mode - R/W 0xF70D OR 0xF70C??? 3-charge(PV+AC), 4-Discharge(PV+Batt)
            self.client.write_register(0xF70D, 0, unit=self.inv_id)
        if debug: debug_printing([read_1, self.read_battery_parameter()])