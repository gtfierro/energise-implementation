#!/usr/bin/env python

'''
    communicate with the Opal-RT MODBUS client at
    Lawrence Berkeley National Laboratory's FlexGrid.
    Christoph Gehbauer (cgehbauer@lbl.gov)
    07/29/2019
    
    FOR INTERNAL LBNL USE ONLY!
'''

# In[switches]:

# Imports
import pandas as pd
import numpy as np
import time
import datetime as dt

#Read in swtich info for ORT imports and exports
#pathname = 'sw_mat/'
#filename = 'HIL_switch_matrix_13NF_bal.xlsx'
#filepath = pathname + filename

def read_sw_mat(filepath):
    #pathname = '/Users/jasperpakshong/Documents/Berkeley/ENERGISE/LBNL/FLEXLAB/'
    dfsw = pd.ExcelFile(filepath)
    dfsw_in = dfsw.parse('inputs')
    dfsw_out = dfsw.parse('outputs')
    
    return dfsw_in, dfsw_out

# In[Modbus]:
    
# Imports
from pymodbus.client.sync import ModbusTcpClient as ModbusClient    
    
#MODBUS
def sim_start_stop(dfsw_in,dfsw_out,test_ID,sim_length_min):
    
    if test_ID in dfsw_in.columns:
        
        IP = '131.243.41.14'
        PORT = 503
        id = 2
        
        # Connect to client
        client = ModbusClient(IP, port=PORT)
        
        # Get indeces & assign values
        sw_idx = []
        scales = {}
        
        
    
        for i in range(dfsw_in.shape[0]):
            if dfsw_in.Description.values[i][:3] == 'sw_':
                sw_idx.append(i)
            if 'scale_flexgrid' in dfsw_in.Description.values[i]:
                scales['flexgrid'] = {'register':dfsw_in['Register'][i], 'value':dfsw_in[test_ID][i]}
            if 'scale_loadrack' in dfsw_in.Description.values[i]:
                scales['loadrack'] = {'register':dfsw_in['Register'][i], 'value':dfsw_in[test_ID][i]}
                
        mtx = []
        mtx_register = []
    
        for i in sw_idx:
            mtx.append(dfsw_in[test_ID][i])
            mtx_register.append(dfsw_in['Register'][i])
        
        
        try:
            
            # Write the scaling
            client.write_registers(scales['flexgrid']['register'],
                                   int(scales['flexgrid']['value']), unit=id)
            client.write_registers(scales['loadrack']['register'],
                                   int(scales['loadrack']['value']), unit=id)
            
            # write switch positions for config
            for i in range(len(mtx)):
                client.write_registers(mtx_register[i], int(mtx[i]), unit=id)
                
            # Read simulaiton time
            sim_start = client.read_input_registers(1, count=1, unit=id).registers[0]
            
            # start recording data (sim flag on)
            client.write_registers(int(1), int(0), unit=id)
            client.write_registers(int(1), int(1), unit=id) #sets simulation flag to 1 (ON)
            print('simulation start time:',sim_start)
            print('epoch:',time.time())
            print(dt.datetime.fromtimestamp(time.time()))
                
            #for sw in mtx:
            #    client.write_registers(mtx_register, int(sw), unit=id)
            #    mtx_register += 1
            w = 1
            
            while w > 0:
                sim_cur = client.read_input_registers(1, count=1, unit=id).registers[0]
                if sim_cur - sim_start >= sim_length_min*60:
                    break                
            print('simulation end time',sim_cur)
            client.write_registers(int(1), int(0), unit=id)
            print('epoch:',time.time())
            print(dt.datetime.fromtimestamp(time.time()))
            print('All Done.',client.read_input_registers(1, count=1, unit=id).registers[0],'[sim_start_stop]')
            
        except Exception as e:
            print(e)
            
        finally:
            client.close()
            print('client closed')
            
    else:
        print('Test ID error')
    
    return

def sim_start_pause_stop(sim_length_min):
    
        
    IP = '131.243.41.14'
    PORT = 503
    id = 2
    
    # Connect to client
    client = ModbusClient(IP, port=PORT)
    
    
    try:
            
        # Read simulaiton time
        sim_start = client.read_input_registers(1, count=1, unit=id).registers[0]
        print('simulation start time:',sim_start)
        
        # start recording data (sim flag on)
        client.write_registers(int(1), int(3), unit=id) #sets simulation flag to 1 (ON)
            
        w = 1
        
        while w > 0:
            sim_cur = client.read_input_registers(1, count=1, unit=id).registers[0]
            if sim_cur - sim_start >= sim_length_min*60:
                break                
        print('simulation end time',sim_cur)
        client.write_registers(int(1), int(0), unit=id)
        print('All Done.',client.read_input_registers(1, count=1, unit=id).registers[0])
        
    except Exception as e:
        print(e)
        
    finally:
        client.close()
        print('client closed')

    return
        
def sim_stop():
    
    IP = '131.243.41.14'
    PORT = 503
    id = 2
    
    client = ModbusClient(IP, port=PORT)
    try:
        client.write_registers(int(1), int(0), unit=id)
        print('simulation stop time:',client.read_input_registers(1, count=1, unit=id).registers[0])
        print('epoch:',time.time())
        print(dt.datetime.fromtimestamp(time.time()))
        print('All Done.',client.read_input_registers(1, count=1, unit=id).registers[0])
        
    except Exception as e:
        print(e)
        
    finally:
        client.close()
        print('client closed [stop]')
    
    return
        


def sim_start():

    IP = '131.243.41.14'
    PORT = 503
    id = 2
    
    client = ModbusClient(IP, port=PORT)
    try:
        client.write_registers(int(1), int(1), unit=id)
        print('simulation start time:',client.read_input_registers(1, count=1, unit=id).registers[0])
        print('epoch:',time.time())
        print(dt.datetime.fromtimestamp(time.time()))
        print('All Done.',client.read_input_registers(1, count=1, unit=id).registers[0])
        
        
    except Exception as e:
        print(e)
        
    finally:
        client.close()
        print('client closed [start]')
    
    return

def sim_pause():

    IP = '131.243.41.14'
    PORT = 503
    id = 2
    
    client = ModbusClient(IP, port=PORT)
    try:
        client.write_registers(int(1), int(3), unit=id)
        print('simulation start/pause time:',client.read_input_registers(1, count=1, unit=id).registers[0])
        print('epoch:',time.time())
        print(dt.datetime.fromtimestamp(time.time()))
        print('All Done.',client.read_input_registers(1, count=1, unit=id).registers[0])
        
        
    except Exception as e:
        print(e)
        
    finally:
        client.close()
        print('client closed [pause]')
    
    return

def upmu(upmu,sw_value):

    IP = '131.243.41.14'
    PORT = 503
    id = 2
    
    if upmu == '123p':
        register = int(2)
    if upmu == '4':
        register = int(3)

    client = ModbusClient(IP, port=PORT)
    try:
        client.write_registers(register, int(sw_value), unit=id)
        print(f'uPMU{upmu} to {sw_value} ~~ time:',client.read_input_registers(1, count=1, unit=id).registers[0])
        print('epoch:',time.time())
        print(dt.datetime.fromtimestamp(time.time()))
        print('All Done.',client.read_input_registers(1, count=1, unit=id).registers[0])     
        
    except Exception as e:
        print(e)
        
    finally:
        client.close()
        print('client closed [uPMU]')
    
    return
    
def set_switches(dfsw_in,dfsw_out,test_ID):
    
        
    IP = '131.243.41.14'
    PORT = 503
    id = 2
    
     # Connect to client
    client = ModbusClient(IP, port=PORT)
    
    # Get indeces & assign values
    sw_idx = []
    scales = {}
    
    

    for i in range(dfsw_in.shape[0]):
        if dfsw_in.Description.values[i][:3] == 'sw_':
            sw_idx.append(i)
        if 'scale_flexgrid' in dfsw_in.Description.values[i]:
            scales['flexgrid'] = {'register':dfsw_in['Register'][i], 'value':dfsw_in[test_ID][i]}
        if 'scale_loadrack' in dfsw_in.Description.values[i]:
            scales['loadrack'] = {'register':dfsw_in['Register'][i], 'value':dfsw_in[test_ID][i]}
            
    mtx = []
    mtx_register = []

    for i in sw_idx:
        mtx.append(dfsw_in[test_ID][i])
        mtx_register.append(dfsw_in['Register'][i])
    
    
    try:
        
        # Write the scaling
        client.write_registers(scales['flexgrid']['register'],
                               int(scales['flexgrid']['value']), unit=id)
        client.write_registers(scales['loadrack']['register'],
                               int(scales['loadrack']['value']), unit=id)
        
        # write switch positions for config
        for i in range(len(mtx)):
            client.write_registers(mtx_register[i], int(mtx[i]), unit=id)
            
    except Exception as e:
        print(e)
        
    finally:
        client.close()
        print('client closed')

    return