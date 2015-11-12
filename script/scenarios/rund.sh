#!/bin/bash         

gepetto-viewer-server & 
ipython -i ./$1

pkill -f  'gepetto-viewer-server'
