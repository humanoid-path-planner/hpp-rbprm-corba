#!/bin/bash         

gepetto-viewer-server & 
hpp-rbprm-server &
ipython -i --no-confirm-exit ./$1

pkill -f  'gepetto-viewer-server'
pkill -f  'hpp-rbprm-server'
