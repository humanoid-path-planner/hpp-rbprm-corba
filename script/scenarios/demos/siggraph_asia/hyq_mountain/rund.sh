#!/bin/bash         

gepetto-viewer-server & 
ipython -i --no-confirm-exit ./$1

pkill -f  'gepetto-viewer-server'
