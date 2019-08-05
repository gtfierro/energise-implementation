#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 31 18:20:43 2019

@author: energise
"""
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

IP = '131.243.41.14'
PORT = 504
id = 2

#IP = '131.243.41.14'
#PORT = 502
#id = 1

# Connect to client
client = ModbusClient(IP, port=PORT)

mtx = [0]*6
mtx_register = np.arange(1,7).tolist()


try:
    
    # write switch positions for config
    for i in range(len(mtx)):
        client.write_registers(int(mtx_register[i]), int(mtx[i]), unit=id)
    print('sent')
    
    #client.write_registers(mtx_register, mtx, unit=id)
    #print('sent')
        
    
except Exception as e:
    print(e)
    
finally:
    client.close()
    
    
    
