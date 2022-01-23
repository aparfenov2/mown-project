FILE=/catkin_ws/devel/setup.bash
if test -f "$FILE"; then
    source $FILE
    echo "Source SECCESSED"
else
    echo "File $FILE DOESN'T exist "
fi