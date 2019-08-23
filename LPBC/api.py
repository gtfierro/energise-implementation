'''
list of request commands:
    
inverter:
    - f"http://131.243.41.47:9090/control?Batt_ctrl={batt},pf_ctrl={pf_ctrl},inv_id={inv}"

load racks:
    http://131.243.41.118:9090/control_enable
        -- enable
    http://131.243.41.118:9090/control?P_ctrl=1000 
        -- set all
    http://131.243.41.118:9090/control?group_id=1,P_ctrl=1000 
        -- set one
    http://131.243.41.118:9090/control_disable
        -- disable, however we are now just setting all to 0 when done
        
'''
import requests
import numpy
import time
import datetime as dt


# load rack values:
P_ctrl = 0
group_id = 0 # [0,1,2]

# inverter values:
Batt_ctrl = 100 # (+) is charging!
pf_ctrl = 0.99 # [-1,1] - BUT abs() > 0.85
inv_id = 1 # [1,2,3]

inv_perc = 2


if np.abs(pf_ctrl) < 0.85:
    pf_ctrl = 0.85
    
t0 = time.time()

#~~~~~~~~~~~
# LOAD RACKS:

#command = 'http://131.243.41.118:9090/control_enable'
#command = 'http://131.243.41.118:9090/control_disable'

#command = f'http://131.243.41.118:9090/control?P_ctrl={P_ctrl}'
#command = f'http://131.243.41.118:9090/control?P_ctrl={P_ctrl},group_id={group_id}'

#~~~~~~~~~~~
# INVERTER

##### batt only
#command = f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl}'  # works
#####  pf only
#command = f'http://131.243.41.47:9090/control?pf_ctrl={pf_ctrl}' # works
#####  batt / inv
#command = f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl},inv_id={inv_id}'  # works
##### batt / pf
#command = f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl},pf_ctrl={pf_ctrl}'  # no pf cmd? had no enable command
##### batt / pf / inv
#command = f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl},pf_ctrl={pf_ctrl},inv_id={inv_id}'  # works

##### inv perc
#command = f'http://131.243.41.47:9090/control?P_ctrl={inv_perc}'

r = requests.get(command)

print(f'time to execute: {time.time()-t0}')
print(r)
print('api cmd:', command, dt.datetime.now())