#!/bin/bash         

gepetto-gui &
hppcorbaserver &
ipython -i --no-confirm-exit ./$1

pkill -f  'gepetto-gui'
pkill -f  'hppcorbaserver'
