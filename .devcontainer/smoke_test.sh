#!/bin/bash
set -e

mushr_noetic run 'source ~/.bashrc && roslaunch mushr_sim teleop.launch' > /tmp/output &
sleep 5
if tail -n 1 /tmp/output | grep -q "Rosbridge WebSocket server started at ws://0.0.0.0:9090"; then
    echo "✅ smoke_test passed"
    exit 0;
else
    # Print output of `roslaunch mushr_sim teleop.launch` for debugging
    cat /tmp/output
    echo "❌ smoke_test failed"
    exit 1;
fi