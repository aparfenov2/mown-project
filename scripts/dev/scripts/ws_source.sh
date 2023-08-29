echo "==========================="
echo "= USAGE: . ws_source.bash ="
echo "==========================="
FILE=/mown-project/devel/setup.bash
if test -f "$FILE"; then
    source $FILE
    echo ""
    echo "Source SECCESSED"
    echo ""
else
    echo ""
    echo "File $FILE DOESN'T exist "
    echo ""
fi