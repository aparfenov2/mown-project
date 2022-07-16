#!/bin/bash
echo "================================="
echo "== Run script inside container =="
echo "================================="
ln -s /mown-project/dev/src /catkin_ws/src
for file in "/mown-project/scripts"/*; do
	chmod +x $file
    ln -s $file /usr/bin/$(basename $file)
done

cd /catkin_ws && catkin init