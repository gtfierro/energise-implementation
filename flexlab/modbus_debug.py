#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 17 18:11:05 2020

@author: energise

modbus debugging script
"""

# In[imports]

# Imports
from pymodbus.client.sync import ModbusTcpClient as ModbusClient 

import pandas as pd
import numpy as np
import time
import datetime as dt

# In[test script]

IP = '131.243.41.14'
PORT = 503
id = 2

# Connect to client
client = ModbusClient(IP, port=PORT)

### read in sw matrix
pathname = 'sw_mat_HIL2/'
filename = 'HIL2_switch_matrix_13NF_bal.xlsx'
filepath = pathname+filename

dfsw = pd.ExcelFile(filepath)
dfsw_in = dfsw.parse('inputs')
dfsw_out = dfsw.parse('outputs')
###

test_ID = 'T3.3'

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
    
    if test_ID in dfsw_in.columns:
        print(f'{test_ID} found')
        
        # Write the scaling
        client.write_registers(scales['flexgrid']['register'],
                               int(scales['flexgrid']['value']), unit=id)
        client.write_registers(scales['loadrack']['register'],
                               int(scales['loadrack']['value']), unit=id)
        
        # write switch positions for config
        for i in range(len(mtx)):
            client.write_registers(mtx_register[i], int(mtx[i]), unit=id)
            
    else:
        print('ID NOT FOUND')
            
except Exception as e:
    print(e)
    

try:        
        
    print(client.read_input_registers(1, count=1, unit=id).registers[0])
    print('client time read - success')
    
    
    
except Exception as e:
    print(e)
    
finally:
    client.close()
    print('client closed')