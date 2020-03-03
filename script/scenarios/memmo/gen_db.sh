#!/bin/bash         


for i in {1..1000} 
    do
        gepetto-gui &
        hppcorbaserver &
        ipython3 --no-confirm-exit $1

        pkill -f  'gepetto-gui'
        pkill -f  'hppcorbaserver'
        
        sleep 1
done

