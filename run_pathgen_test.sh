 #!/bin/bash

# all parameter combos for path gen
PARAMS=(
  "0.1 2"
  "0.1 3"
  "0.1 4"
  "0.1 5"

  "0.2 2"
  "0.2 3"
  "0.2 4"
  "0.2 5"

  "0.3 2"
  "0.3 3"
  "0.3 4"
  "0.3 5"

  "0.4 2"
  "0.4 3"
  "0.4 4"
  "0.4 5"

  "0.5 2"
  "0.5 3"
  "0.5 4"
  "0.5 5"
)

touch ./bag/NEW_TESTING_BLOCK_STARTS_HERE.txt


for p in "${PARAMS[@]}"; do
    echo "Running with: $p"
    for ((i = 0; i < 30; i ++)); do
        source devel/setup.bash && rosrun path_gen read_stereo 200 200 false $p &
        PID=$!

        # Let it run for e.g. 15 seconds
        sleep 1.5

        # Kill the node before next run
        kill $PID
        wait $PID 2>/dev/null
    done
done
