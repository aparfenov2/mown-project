#!/bin/bash
SEGMENTATION_BYPASS=1
ROSCORE=1

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --sim) SIM=1 ;;
        --rviz) RVIZ=1 ;;
        --teleop) TELEOP=1 ;;
        --roscore) ROSCORE=1 ;;
        ---roscore) ROSCORE="" ;;
        --segm) SEGMENTATION=1 ;;
        ---segm_bypass) SEGMENTATION_BYPASS="" ;;
        --proj) PROJECTION=1 ;;
        --loca) LOCALIZATION=1 ;;
        --sim_basic) SIM=1; RVIZ=1; TELEOP=1; ;;
        --preset1) SIM=1; RVIZ=1; TELEOP=1; SEGMENTATION=1; PROJECTION=1; LOCALIZATION=1;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

[ -n "$ROSCORE" ] && {
    guake -n 1 -r core -e "cd $PWD/segmentation && bash run_cmd.sh roscore"
    sleep 2
}

[ -n "$SIM" ] && {
    guake -n 1 -r sim -e "cd $PWD/simulation && bash run_sim.sh"
}

[ -n "$TELEOP" ] && {
    gnome-terminal -- /bin/sh -c 'cd $PWD/segmentation && ls && bash run_teleop.sh'
}

[ -n "$RVIZ" ] && {
    # guake -n 1 -r rviz -e "cd $PWD/segmentation && bash run_rviz.sh"
    guake -n 1 -r rviz -e "cd $PWD/projection && bash run_this.sh --rviz"
}


[ -n "$SEGMENTATION" ] && {
    [ -n "${SEGMENTATION_BYPASS}" ] && {
        _BYPASS="--bypass"
    }
    guake -n 1 -r seg -e "cd $PWD/segmentation && bash run_segmentation.sh ${_BYPASS}"
}

[ -n "$LOCALIZATION" ] && {
    guake -n 1 -r prj -e "cd $PWD/localization && bash run_loam.sh"
}

[ -n "$PROJECTION" ] && {
    guake -n 1 -r prj -e "cd $PWD/projection && bash run_this.sh"
}
