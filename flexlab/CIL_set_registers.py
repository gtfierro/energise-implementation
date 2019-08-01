#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 31 18:20:43 2019

@author: energise
"""

IP = '131.243.41.14'
PORT = 503
id = 2

# Connect to client
client = ModbusClient(IP, port=PORT)

mtx = [1]*6
mtx_register = np.arange(1,7).tolist()


try:
    
    # write switch positions for config
    for i in range(len(mtx)):
        client.write_registers(int(mtx_register[i]), int(mtx[i]), unit=id)
    print('sent')
        
    
except Exception as e:
    print(e)
    
finally:
    client.close()