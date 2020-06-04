import requests
import numpy as np
import time
import datetime as dt
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

groups = [0,1,2]
P_init = 500

for group in groups:
    command = f"http://131.243.41.59:9090/control?group_id={group},P_ctrl={P_init}"
    r = requests.get(command)
    print(r.status_code)
    print('api cmd:', command, dt.datetime.now())