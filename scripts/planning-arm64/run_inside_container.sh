#!/bin/bash
echo "================================="
echo "== Run script inside container =="
echo "================================="
for file in "/catkin_ws/scripts/scripts"/*; do
	chmod +x $file
    ln -s $file /usr/bin/$(basename $file)
done

cd /catkin_ws && catkin init