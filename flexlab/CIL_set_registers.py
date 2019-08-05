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

P1, P2, P3 = 20000, 20000, 20000
Q1, Q2, Q3 = 20000, 20000, 20000
#           P,Q
sign_vec = [1,1,
            1,1,
            1,1]
sign_base = [2**0,2**1,2**2,2**3,2**4,2**5]
sign_list = (np.array(sign_vec)*np.array(sign_base)).tolist()

#mtx = [0]*6
mtx = [P1,Q1,P2,Q2,P3,Q3,sign_list]
mtx_register = np.arange(1,8).tolist()


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
    
    
    
