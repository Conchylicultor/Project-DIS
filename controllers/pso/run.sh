#!/bin/bash

# Launch a few webots in parallel before launching the PSO optimiser

world='../../worlds/obstacles.wbt'

size=`grep 'SWARM_SIZE = ' pso.cpp  | awk '{print $5}'`

for i in `seq 1 $size`
do
    if [ -x '/Applications/Webots/webots' ]
    then
        /Applications/Webots/webots --minimize --mode=fast "$world" 2>/dev/null &
    else
        # assuming it's in the PATH
        webots --minimize --mode=fast "$world" 2>/dev/null &
    fi
done

echo "waiting 15 secs so that webots instances are all running..."
sleep 15
echo "done!"

ps -A | grep supervisor | grep -v grep | awk '{print $1}' | xargs echo "PIDS: "
ps -A | grep supervisor | grep -v grep | awk '{print $1}' | xargs ./pso


