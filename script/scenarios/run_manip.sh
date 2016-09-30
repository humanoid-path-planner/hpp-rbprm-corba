#!/bin/bash         

gepetto-viewer-server & 
hpp-manipulation-server & 
#~ hpp-rbprm-server &
ipython -i --no-confirm-exit ./$1

pkill -f  'gepetto-viewer-server'
pkill -f  'hpp-manipulation-server'
#~ pkill -f  'hpp-rbprm-server'
