#!/bin/bash
set -e

mushr_noetic run 'source ~/.bashrc && roslaunch mushr_sim teleop.launch' > /tmp/output_roslaunch &
sleep 5
if tail -n 1 /tmp/output_roslaunch | grep -q "Rosbridge WebSocket server started at ws://0.0.0.0:9090"; then
    echo "✅ roslaunch-teleop-test passed"
else
    cat /tmp/output_roslaunch  # print output for debugging purposes
    echo "❌ roslaunch-teleop-test failed"
    exit 1;
fi

docker run --rm -p "8080:8080" ghcr.io/foxglove/studio:latest &
sleep 10;
if curl -sSf http://localhost:8080 >/dev/null; then
    echo "✅ start-foxglove-studio passed"
else
    echo "❌ start-foxglove-studio failed"
    exit 1;
fi

exit 0;