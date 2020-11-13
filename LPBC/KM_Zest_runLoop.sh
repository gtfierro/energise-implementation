#!/bin/bash

CONFIG="KM_Zest_params.cfg"

function set_config(){
    # sudo sed -i "s/^\($1\s*=\s*\).*\$/\1$2/" $CONFIG
    sed -i "s/^\($1\s*=\s*\).*\$/\1$2/" $CONFIG
}


# for (( counter=2; counter>0; counter-- ))
for eps in 0.75 1.25
do
set_config initParam $eps
printf "\n"
printf "\n"
printf "\n"
# echo -n "&&&&&&&&&&~~~~~~~~~~&&&&&&&&&& NEW RUN, this many remaining: $counter "
echo -n "&&&&&&&&&&~~~~~~~~~~&&&&&&&&&& NEW RUN, this many remaining: $eps "
printf "\n"
printf "\n"
printf "\n"
printf "\n"
python3 KM_Zest_initCIL.py

python3 KM_Zestwrapper_CIL.py

done
printf "\n"
