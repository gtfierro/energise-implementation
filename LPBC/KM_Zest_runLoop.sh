#!/bin/bash



for (( counter=2; counter>0; counter-- ))
do
echo -n "$counter "
python KM_Zest_initCIL.py

done
printf "\n"
