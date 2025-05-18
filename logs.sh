#!/bin/bash
cd `dirname $0`
SCRIPTDIR=`pwd`
cd && source venv/bin/activate 
cd -
esphome logs nrf_ble_main.yaml 
deactivate
read -p "Press any key to continue" x
