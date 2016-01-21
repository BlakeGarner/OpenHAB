#OpenHAB Modules
*Modules and scripts intended for various embedded and PC hardware designed to provide interaction / functionality to my OpenHAB setup.*


##mqttToArduinoCape.py Script Install Instructions
*Creating a Raspberry Pi OpenHAB module on the old NinjaBlock PiCrust hardware*

1. Create raspbian-lite image on SD Card using Win32DiskImager (in Windows, check the Raspberry Pi home page for installation instructions on other systems).
2. Boot up module, and ssh into it. Username/password is pi/raspberry
3. Run `sudo raspi-config` and perform the following settings changes:
  a. Expand Filesystem
  b. Change the Password
  c. Internationalisation Options: Change Locale & Timezone
  d. Advanced Options -> Hostname
  e. Advanced Options -> Serial. Choose "No" to the question "Would you like a login shell to be accessible over serial?" (We need the serial port for the app).
  f. Advanced Options -> Update
4. Reboot: `sudo shutdown -r now`
5. Once it's rebooted, log back in and run `sudo apt-get update && sudo apt-get upgrade`
6. Install Python: `sudo apt-get install python3 python3-pip python3-rpi.gpio`
7. Install needed python packages: `sudo pip3 install paho-mqtt pyserial`
8. Copy the python script `mqttToArduinoCape.py` into your scripts directory (I put them all into `~/python-scripts/`)
9. Check the serial port defined in mqttToArduinoCape.py `SERIAL_PORT` matches the serial port in `/dev/`. (`/dev/ttyAMA0` in my RPi2's case)
10. `sudo crontab -e` and add the following line to the bottom: `@reboot python3 /home/pi/python-scripts/mqttToArduinoCape.py &`
11. Reboot: `sudo shutdown -r now` and check the script is running by sending a command to the pi.
