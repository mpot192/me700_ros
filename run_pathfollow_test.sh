#!/bin/bash

# all parameter combos for path gen
PARAMS=(
#   --- pos sweep ---
#   "pos 0.1"
#   "pos 0.2"
#   "pos 0.3"
#   "pos 0.4"
#   "pos 0.5"
# 
#   --- vel sweep (pairs) ---
#   "vel 0.1 0.1"
#   "vel 0.1 0.2"
#   "vel 0.1 0.3"
#   "vel 0.1 0.4" always fails
#   "vel 0.1 0.5" always fails
# 
#   "vel 0.2 0.1"
#   "vel 0.2 0.2"
#   "vel 0.2 0.3"
#   "vel 0.2 0.4"
#   "vel 0.2 0.5"
# 
#   "vel 0.3 0.1"
#   "vel 0.3 0.2"
#   "vel 0.3 0.3"
#   "vel 0.3 0.4"
#   "vel 0.3 0.5"
# 
#   "vel 0.4 0.1"
#   "vel 0.4 0.2"
#   "vel 0.4 0.3"
#   "vel 0.4 0.4"
#   "vel 0.4 0.5"
# 
#   "vel 0.5 0.1"
#   "vel 0.5 0.2"
#   "vel 0.5 0.3"
#   "vel 0.5 0.4"
#   "vel 0.5 0.5"
# 
#   --- carrot sweep ---
#   "carrot 0.1"
#   "carrot 0.2"
#   "carrot 0.3"
#   "carrot 0.4"
  "carrot 0.5"
  "carrot 0.5"
)

touch ./bag/NEW_TESTING_BLOCK_STARTS_HERE.txt

for p in "${PARAMS[@]}"; do
    echo "Running with: $p"
    source devel/setup.bash && rosrun path_follow follow_path $p &
    PID=$!

    # Wait for /status message containing "Ready to fly" with timeout
    TIMEOUT=720
    START_TIME=$SECONDS

    while [ $((SECONDS - START_TIME )) -lt $TIMEOUT ]; do
        # Listen for one message, short timeout
        MSG=$(rostopic echo /status -n 1 -p 2>/dev/null | awk -F, '{print $2}' | tail -n1)
        if [[ "$MSG" == *"FLIGHT_COMPLETE"* ]]; then
            break
        fi
        sleep 0.5
    done
    
    echo "moving on"

    # wait 5 seconds before moving on
    sleep 5 
    # Kill the node before next run
    kill $PID
    wait $PID 2>/dev/null
    
    sleep 5 
done
