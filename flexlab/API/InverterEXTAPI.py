import time
import json
from datetime import datetime
from InverterControl import Inverter
from InverterControl import ModbusRTUClient
import multiprocessing as mp


class Flexgrid_API(object):
    def __init__(self, inv_ids=[1,2,3], portNames=['com4'], baudrate=115200, parallel=False, safety = True, debug=False, ComClient = ModbusRTUClient):
        self.debug = debug
        self.inv_ids = inv_ids
        self.baudrate = baudrate
        self.portNames = portNames
        self.parallel = parallel
        self.safety = safety
        self.inverters=[None]*len(inv_ids)
        if self.parallel:
             assert len(inv_ids) == len(portNames), "In parallel mode, you need 1 modbus interface per inverter"
             for ind  in range(len(inv_ids)):
                 client = ComClient(portName=portNames[ind], baudrate=115200)
                 self.inverters[ind] = Inverter(inv_id = inv_ids[ind], client = client)
        else:
            self.client = ComClient(portName=portNames[0], baudrate=115200)
            for ind  in range(len(inv_ids)):
                 self.inverters[ind] = Inverter(inv_id = inv_ids[ind], client = self.client)
    
    ### Calling function to support either setups
    def __series_execution(self, func, inverters, **kwargs):
            ''' 
                Implements a series call iterating though all inverters using a single client
            '''
            data = {}
            self.client.connect()
            for inv in inverters:
                inv_id = inv.inv_id
                data[str(inv_id)] = {}
                res = func(inv, **kwargs)
                if res is not None:
                    data[str(inv_id)].update(func(inv, **kwargs))
            self.client.close()
            return data

    
    def __parallel_execution(self, func, inverters , **kwargs):
            '''
                Implements a parallel call iterating though all inverters each using their own client
            '''
            data = [None]*len(inverters)
            for inv in inverters:
                inv_id = inv.inv_id
                inv.connect_modbus()
                data[inv_id-1] = func(inv, **kwargs)
                inv.close_modbus()
            return data

    def __inv_select(self, inv_ids):
        '''
            Selects the inverters from a list of ids
        '''
        if type(inv_ids) != type([]): inv_ids = [inv_ids]
        myInverters = []
        for inv_id in inv_ids:
            ind = self.inv_ids.index(inv_id)
            myInverters.append(self.inverters[ind])
        return myInverters
        
    def get_all_readings(self):
        '''
            obtain all the available readings from all sites
        '''
        if self.parallel:
            self.readings = self.__parallel_execution(Inverter.read_site, self.inverters)
        else:
            self.readings = self.__series_execution(Inverter.read_site, self.inverters)

    def get_all_soc(self):
        '''
            obtain the SoC for all sites
        '''
        if self.parallel:
            self.readings = self.__parallel_execution(Inverter.read_soc, self.inverters)
        else:
            self.readings = self.__series_execution(Inverter.read_soc, self.inverters)
    
    ### Power Control 
    def enable_control(self, batt_low=10):
        '''
            Enable control on all sites
        '''
        def func_list(Inverter):
            Inverter.enable_power_control(debug=self.debug)
            Inverter.enable_battery_control(batt_low=batt_low, debug=self.debug)

        if self.parallel:
            self.__parallel_execution(func_list, self.inverters)
        else:
            self.__series_execution(func_list, self.inverters)
        
    def disable_control(self):
        '''
            Disable control on all sites
        '''
        def func_list(Inverter):
            Inverter.disable_power_control(debug=self.debug)
            Inverter.disable_battery_control(debug=self.debug)

        if self.parallel:
            self.__parallel_execution(func_list, self.inverters)
        else:
            self.__series_execution(func_list, self.inverters)
        
    def set_P(self, P, inv_ids):
        '''
            Set P limit as percentage of P_max
        '''
        inverters = self.__inv_select(inv_ids)

        def func_list(Inverter):
            Inverter.set_P(P, debug=self.debug)
            if self.debug:
                 res = Inverter.read_power_parameter()['Active Power Control']
                 return res

        if self.parallel:
            self.__parallel_execution(func_list, inverters)
        else:
            self.__series_execution(func_list, inverters)
        
    def set_pf(self, pf, inv_ids):
        ''' 
            Set Power factor of inverters
        '''
        inverters = self.__inv_select(inv_ids)

        def func_list(Inverter):
            Inverter.set_pf(pf, safety = self.safety, debug=self.debug)
            if self.debug:
                 res = Inverter.read_power_parameter()['Power Factor Control']
                 return res

        if self.parallel:
            self.__parallel_execution(func_list, inverters)
        else:
            self.__series_execution(func_list, inverters)

    ### Dynamic Power Control
    def enable_dyn_control(self, inv_ids):
        '''
            enable dynamic poweer control (PQ control)
        '''
        inverters = self.__inv_select(inv_ids)
        if self.parallel:
            self.__parallel_execution(Inverter.enable_dyn_power_control, inverters, debug = self.debug)
        else:
            self.__series_execution(Inverter.enable_dyn_power_control, inverters, debug = self.debug)

    def disable_dyn_control(self, inv_ids):
        '''
            disable dynamic poweer control (PQ control)
        '''
        inverters = self.__inv_select(inv_ids)
        if self.parallel:
            self.__parallel_execution(Inverter.disable_dyn_power_control, inverters, debug = self.debug)
        else:
            self.__series_execution(Inverter.disable_dyn_power_control, inverters, debug = self.debug)

    def set_dyn_P(self, P, inv_ids):
        '''
            Set P limit as percentage of P_max
        '''
        inverters = self.__inv_select(inv_ids)

        def func_list(Inverter):
            Inverter.set_dyn_P(P, debug=self.debug)
            if self.debug:
                 #res = Inverter.read_power_parameter()['Active Power Control']
                 return res

        if self.parallel:
            self.__parallel_execution(func_list, inverters)
        else:
            self.__series_execution(func_list, inverters)
        
    def set_dyn_Q(self, Q, inv_ids):
        ''' 
            Set Q of Inverters in dynamic mode
        '''
        inverters = self.__inv_select(inv_ids)

        def func_list(Inverter):
            Inverter.set_dyn_Q(Q, safety = self.safety, debug=self.debug)
            if self.debug:
                 #res = Inverter.read_power_parameter()['Power Factor Control']
                 return res

        if self.parallel:
            self.__parallel_execution(func_list, inverters)
        else:
            self.__series_execution(func_list, inverters)

### Battery Control

    def battery_control(self, P, inv_ids):
        ''' 
            Set Power of Battery
        '''
        inverters = self.__inv_select(inv_ids)

        def func_list(Inverter):
            Inverter.battery_control(P, debug=self.debug)
            if self.debug:
                 res = Inverter.read_battery_telemetry()['Batt_P_W']
                 return res

        if self.parallel:
            self.__parallel_execution(func_list, inverters)
        else:
            self.__series_execution(func_list, inverters)
        
    def battery_max_export(self, P, inv_ids):
        ''' 
            Set Power limit of Battery in Max Export mode
        '''
        inverters = self.__inv_select(inv_ids)

        def func_list(Inverter):
            Inverter.battery_max_export(P, debug=self.debug)
            if self.debug:
                 res = Inverter.read_battery_telemetry()['Batt_P_W']
                 return res

        if self.parallel:
            self.__parallel_execution(func_list, inverters)
        else:
            self.__series_execution(func_list, inverters)