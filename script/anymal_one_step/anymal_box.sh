#! /bin/bash


gepetto-gui &
hpp-rbprm-server &
cp *.py /media/data/dev/linux/hpp/src/hpp-rbprm-corba/script/scenarios/memmo
#~ ipython -i --no-confirm-exit $DEVEL_HPP_DIR/src/multicontact-locomotion-planning/scripts/run_mlp.py talos_circle
ipython -i --no-confirm-exit $DEVEL_HPP_DIR/src/multicontact-locomotion-planning/scripts/run_mlp.py anymal_box.py

pkill -f  'gepetto-gui'
pkill -f  'hpp-rbprm-server'

