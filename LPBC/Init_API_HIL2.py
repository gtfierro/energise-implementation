from json import dumps, loads
from datetime import datetime, timedelta
from pathlib import *
import csv
import sys
from time import sleep, time, gmtime, mktime
from Development.InverterEXTAPI import Flexgrid_API
from Development.InverterControl import ModbusRTUClient
from Development.convert_data import *

# Instantiate Flexgrid API
flexgrid = Flexgrid_API(inv_ids=[1,2,3], portNames=['COM3'], baudrate=115200, parallel=False, safety = True, debug=False, ComClient = ModbusRTUClient)

#set inverters to zero
flexgrid.set_P(0, [1,2,3])

#set batteries to max
flexgrid.battery_max_export(3300,[1,2,3])


