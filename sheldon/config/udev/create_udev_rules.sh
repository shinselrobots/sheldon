#!/bin/bash

echo "start copy rules to  /etc/udev/rules.d/"
sudo cp *.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finished "
echo "check results using the command : ls -la /dev/ttyA* (ACM) or U (USB)"
ls -la /dev/ttyA*
ls -la /dev/ttyU*


