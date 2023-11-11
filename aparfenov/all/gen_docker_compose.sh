set -e

VOLUMES=()
for f in $(find ws/src -type l); do
    if ! readlink -f $f > /dev/null; then
        echo "failed to map $f"
    fi
    VOLUMES+=("$(readlink -f $f):/cdir/$f")
done

[ -e "data" ] && {
    VOLUMES+=("$(readlink -f data):/cdir/data")
}
DCG="docker-compose.yaml"
echo "version: \"3.9\"" > $DCG
echo "x-common-mounts: &common-mounts" >> $DCG
for f in "${VOLUMES[@]}"; do
    echo "    - $f" >> $DCG
done

cat docker-compose.main.yaml >> $DCG
xhost +
