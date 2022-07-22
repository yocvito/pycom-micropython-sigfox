#!/bin/bash

if [[ $# -lt 1 ]]
then
    echo "Usage: $0 <remote-dev-port>"
    exit 1
fi

PORT="${1}"
IP="147.210.129.212"
FW="$(find ./esp32/build/ -name 'FiPy-*.tar.gz' | head -n1)"
FW_NAME="$(basename ${FW})"

# compile
make BOARD=FIPY PHYSEC_ENABLED=1 -C esp32 -j8

# send firmware
scp -q ${FW} doctorant@${IP}:~/

# flash firmware
ssh -q doctorant@${IP} "python3 ./fw_updater/updater.py --port ${PORT} --speed 912600 flash -t ~/${FW_NAME}"