#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 31 18:20:43 2019

@author: energise
"""
import numpy as np
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

IP = '131.243.41.14'
PORT = 504
id = 3

#IP = '131.243.41.14'
#PORT = 502
#id = 1

# Connect to client
client = ModbusClient(IP, port=PORT)


#P,Q commands in W and VAR (not kilo)
c = 500/3.3/1000

#P1, P2, P3 = 1, 3, 5
#Q1, Q2, Q3 = 2, 4, 6

P1, P2, P3 = 0, 0, 0
Q1, Q2, Q3 = 0, 0, 0



# set signs of commands through sign_vec
#           P,Q      1 is positive, 0 is negative
sign_vec = [0,0,
            0,0,
            0,0]
sign_base = 2**5 * sign_vec[0] + 2**4 * sign_vec[1] + 2**3 * sign_vec[2] + 2**2 * sign_vec[3] + 2**1 * sign_vec[4] + 2**0  * sign_vec[5]
# sign_list = (np.array(sign_vec)*np.array(sign_base)).tolist()

#mtx = [0]*6
mtx = [P1,Q1,P2,Q2,P3,Q3,sign_base]
mtx_register = np.arange(1,8).tolist()


try:
    
    # write switch positions for config
    for i in range(len(mtx)):
        client.write_registers(int(mtx_register[i]), mtx[i], unit=id)
    print('sent')
    
    #client.write_registers(mtx_register, mtx, unit=id)
    #print('sent')
        
    
except Exception as e:
    print(e)
    
finally:
    client.close()
    
    
    
