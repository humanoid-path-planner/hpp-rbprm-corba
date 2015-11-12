#!/bin/bash         

gepetto-viewer-server & 
hpp-rbprm-server &
ipython -i ./$1

pkill -f  'gepetto-viewer-server'
pkill -f  'hpp-rbprm-server'
