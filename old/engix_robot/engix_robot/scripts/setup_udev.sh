#!/usr/bin/env bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' | sudo tee /etc/udev/rules.d/91-odrive.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/92-rplidar.rules
echo 'KERNEL=="js*", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c21f", SYMLINK="input/f710"'  | sudo tee /etc/udev/rules.d/93-logitechf710.rules
echo 'KERNEL=="js*", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c21d", SYMLINK="input/f310"'  | sudo tee /etc/udev/rules.d/94-logitechf310.rules

sudo udevadm control --reload-rules
sudo udevadm trigger