#!/bin/bash

for (( counter=2; counter>0; counter-- ))
do
printf "\n"
printf "\n"
printf "\n"
echo -n "&&&&&&&&&&~~~~~~~~~~&&&&&&&&&& NEW RUN, this many remaining: $counter "
printf "\n"
printf "\n"
printf "\n"
printf "\n"
python3 KM_Zest_initCIL.py

python3 KM_Zestwrapper_CIL.py

done
printf "\n"
