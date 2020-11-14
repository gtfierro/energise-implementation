#!/bin/bash

CONFIG="KM_Zest_params.cfg"

function set_config(){
    # sudo sed -i "s/^\($1\s*=\s*\).*\$/\1$2/" $CONFIG
    sed -i "s/^\($1\s*=\s*\).*\$/\1$2/" $CONFIG
}

array=(0.5 0.75 1.25 1.5 2)
# array=(-1 -2 -3 -4 -5)
# array=(0.5)
array=(1.25)
# array=(-1)
for eps in "${array[@]}"
do
set_config initParam $eps
printf "\n"
printf "\n"
printf "\n"
# echo -n "&&&&&&&&&&~~~~~~~~~~&&&&&&&&&& NEW RUN, this many remaining: $counter "
echo -n "&&&&&&&&&&~~~~~~~~~~&&&&&&&&&& NEW RUN, Zeffk init param: $eps "
printf "\n"
printf "\n"
printf "\n"
printf "\n"
python3 KM_Zest_initCIL.py

python3 KM_Zestwrapper_CIL.py

done
printf "\n"
