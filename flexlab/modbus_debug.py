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


try:
    print(client.read_input_registers(1, count=1, unit=id).registers[0])
    
except Exception as e:
    print(e)
    
finally:
    client.close()
    print('client closed')