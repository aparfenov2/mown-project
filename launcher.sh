#!/bin/bash

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --sim) SIM=1 ;;
        --rviz) RVIZ=1 ;;
        --teleop) TELEOP=1 ;;
        --segmentation) SEGMENTATION=1 ;;
        --projection) PROJECTION=1 ;;
        --localization) LOCALIZATION=1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

[ -n "$TELEOP" ] && {
    cd simulation
    bash run_teleop.sh
    exit 0
}

[ -n "$SIM" ] && {
    cd simulation
    bash run_sim.sh
    exit 0
}

[ -n "$SEGMENTATION" ] && {
    cd segmentation
    bash run_segmentation.sh
    exit 0
}

[ -n "$PROJECTION" ] && {
    cd projection
    bash run_this.sh
    exit 0
}
