
##Creating a Raspberry Pi OpenHAB module on the old NinjaBlock PiCrust hardware##

#1: Create raspbian-lite image on SD Card using Win32DiskImager
#2: Boot up module, and ssh into it. Username/password is pi/raspberry
#3: Run "sudo raspi-config" and perform the following settings changes:
#3a: Expand Filesystem
#3b: Change the Password
#3c: Internationalisation Options: Change Locale & Timezone
#3d: Advanced Options -> Hostname
#3e: Advanced Options -> Serial. Choose "No" to the question "Would you like a login shell to be accessible over serial?" (We need the serial port for the app).
#3f: Advanced Options -> Update
#4: Reboot: "sudo shutdown -r now"
#5: Once it's rebooted, log back in and run "sudo apt-get update && sudo apt-get upgrade"
#6: Install Python: "sudo apt-get install python3 python3-pip python3-rpi.gpio"
#7: Install needed python packages: "sudo pip3 install paho-mqtt pyserial"
#8: Copy the python script "mqttToArduinoCape.py" into your scripts directory (I put them all into ~/python-scripts/)
#9: Check the serial port defined in mqttToArduinoCape.py "SERIAL_PORT" matches the serial port in /dev/. (/dev/ttyAMA0 in my RPi2's case)

