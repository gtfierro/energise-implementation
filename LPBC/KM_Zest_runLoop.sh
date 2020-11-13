#!/bin/bash

for (( counter=2; counter>0; counter-- ))
do
echo -n "counter = $counter "
python3 KM_Zest_initCIL.py

python3 KM_Zestwrapper_CIL.py

done
printf "\n"
