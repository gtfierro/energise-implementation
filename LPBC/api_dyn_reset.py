import requests
import numpy as np
import time
import datetime as dt
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

IP = '131.243.41.14'
PORT = 504
client = ModbusClient(IP, port=PORT)

# value mapping - 1: [-1, -1], 2: [1, -1], 3: [-1, 1], 4: [1, 1]
# multipliers to inverter values [P, Q] - positive inverter values corresponds to injecting P and Q (value 4)
# FLEXLAB'S QUADRANT CONVENTION 5/22/20 Flexlab set up quadrant convention and will take care of rest into ephasorsim
# Quadrant 1: P consume, Q consume
# Quadrant 2: P inject, Q consume
# Quadrant 3: P consume, Q inject
# Quadrant 4: P inject, Q inject

'''USE THIS SCRIPT BEFORE TESTING TO INITIALIZE INVERTERS'''
# inverter values:
Pcmd_perc_phase = 0.5 # set to 0.5% of inv max (set to 7000W by Maxime from Flexlab) = 35 W ; do not go to 0% unless OK
Qcmd_perc_phase = 1 # set Q to zero
inv = [1] #input inverters [1], [1,2,3], ... so on...

t0 = time.time()

#~~~~~~~~~~~
# INVERTER
for i in inv:
    
    # command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},inv_id={i}"
    command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},dyn_Q_ctrl={Qcmd_perc_phase},inv_id={i}"
    print(command)
    r = requests.get(command)
    print(f'time to execute one command: {time.time()-t0}')
    print(r.status_code)
    print('api cmd:', command, dt.datetime.now())

print(f'time to execute API: {time.time()-t0}')


# inverter 1 (phaseA) = 101
# inverter 2 (phaseB)= 102
# inverter 3 (phaseC) = 103
inv_act_idxs_registers = [101] #input inverters format: [101], [101,102,103], ... so on...
quadrant = 3 # see above for quadrant convention
id = 3

t1 = time.time()
try:
    client.connect()
    for i in inv_act_idxs_registers:  # write quadrant changes to modbus registers
        client.write_registers(i, quadrant, unit=id)
        print('Quadrant change for inv:', i, 'to quadrant', quadrant)

except Exception as e:
    print(e)

finally:
    client.close()

print(f'time to execute modbus quadrant: {time.time() - t1}')
print(f'total time: {time.time() - t0}')