Mower Robot software

Installation:
-----------------------
Clone the repository
! contains submodules
git submodule update --init --recursive
! this command is also included in all/run_all.sh

Start up:
-----------------------
There are several modules need to be started depending of required functionality:

use all/run_all.sh for experiments

expmple:
cd all
bash run_all.sh --sim --rviz --mb_mod --segm --loca --proj --mb_mod

args are:
- --sim   - gazebo
- --rviz  - rviz
- --mb_mod - move_bas_mod (motion controller)
- --segm  - segmanetation
- --loca  - A-LOAM localization (used with sim only)
- --proj  - projection (of segmentation data to grid costmap)
- --teleop - run teleop (run it in a separate window)

OBSOLETE: 
1. simulation/run_sim.sh  - starts gazebo simulation with virtual robot model
2. etc

launcher.sh - common launcher script for all submodules

