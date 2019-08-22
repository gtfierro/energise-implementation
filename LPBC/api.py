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
import time
import datetime as dt


# load rack values:
P_ctrl = 0
group_id = 0 # [0,1,2]

# inverter values:
Batt_ctrl = 500 # (+) is charging!
pf_ctrl = 0.8
inv_id = 3 # [1,2,3]

inv_perc = 97

t0 = time.time()

# load racks:
#r = requests.get('http://131.243.41.118:9090/control_enable')
#r = requests.get('http://131.243.41.118:9090/control_disable')

#r = requests.get(f'http://131.243.41.118:9090/control?P_ctrl={P_ctrl}')
#r = requests.get(f'http://131.243.41.118:9090/control?P_ctrl={P_ctrl},group_id={group_id}')

# INVERTER

##### batt only
#r = requests.get(f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl}')  # works
#####  pf only
#r = requests.get(f'http://131.243.41.47:9090/control?pf_ctrl={pf_ctrl}') # ?
#####  batt / inv
#r = requests.get(f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl},inv_id={inv_id}')  # works
##### batt / pf
r = requests.get(f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl},pf_ctrl={pf_ctrl}')  # no pf cmd
##### batt / pf / inv
#r = requests.get(f'http://131.243.41.47:9090/control?Batt_ctrl={Batt_ctrl},pf_ctrl={pf_ctrl},inv_id={inv_id}')  # works

##### inv perc
#r = requests.get(f'http://131.243.41.47:9090/control?P_ctrl={inv_perc}')

print(f'time to execute: {time.time()-t0}')
print(r)
print('api cmd', dt.datetime.now())