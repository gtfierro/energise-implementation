#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 13 14:58:02 2020

@author: energise
"""

#from json import dumps, loads
from datetime import datetime, timedelta
from pathlib import *
import csv
import sys
import time
from InverterEXTAPI import Flexgrid_API 
from InverterControl import ModbusRTUClient
from convert_data import *
import pause
import requests


# Instantiate Flexgrid API
flexgrid = Flexgrid_API(inv_ids=[1,2,3], portNames=['/dev/ttyUSB0'], baudrate=115200, parallel=False, safety = True, debug=False, ComClient = ModbusRTUClient)
print('Flexgrid API ready')


# Configure constant
iter_nb = 24
iter_freq = 1/6 # Hz
iter_delta=timedelta(seconds=1/iter_freq)
power_table = [15, 20, 25, 20, 15, 10, 15, 20, 25, 20, 15, 10, 20, 30, 20, 10, 20, 30, 20, 10, 30, 10, 30, 10]



# direct python script traditional P control
dt=datetime.now()
print(f'test executed at {dt}')


#set Inverters to 10%
flexgrid.set_P(5,1)
flexgrid.set_P(5,2)
time.sleep(15)

t0 = time.time()
dt=datetime.now()
for itr in range(0,iter_nb):
    #Your code below here
    print(f'timestep: {time.time()-t0}')
    power = power_table[itr]-5
    flexgrid.set_P(power,[1,2])
    print(f'Set Power to {power} %')
    print(f'time to execute: {time.time()-dt.timestamp()}')
    # Code above    
    dt+=iter_delta
    pause.until(dt)
    dt=datetime.now()

print('End of Test')
time.sleep(10)
flexgrid.set_P(100,[1,2])

