#!/bin/bash
sudo screen -dmS sound soundmodem
sudo screen -dmS beacon python2 /home/craag/mobile-aprs/beacon-scripts/beacon_usb.py
sudo screen -dmS aprs aprx -dd -f /home/craag/mobile-aprs/aprx-configs/mobile-digi-aprx.conf
