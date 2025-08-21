#!/usr/bin/env bash
set -euo pipefail

SOCK="unix:/tmp/kitty-ros"

nohup kitty -o allow_remote_control=yes --listen-on "$SOCK" \
      --session "~/catkin_ws/config/rosterm.session" &

# Wait until Kitty is ready
until kitty @ --to "$SOCK" ls >/dev/null 2>&1; do sleep 0.05; done

#sim
kitty @ --to "$SOCK" send-text --match title:sim \
  'source devel/setup.bash && roslaunch survey_simulator run_sitl_topdown.launch' 
  
#follow
kitty @ --to "$SOCK" send-text --match title:follow \
  'source devel/setup.bash && rosrun path_follow follow_path carrot' 
  
#pathgen
kitty @ --to "$SOCK" send-text --match title:pathgen \
  'source devel/setup.bash && rosrun path_gen read_stereo 200 200' 

#hover
kitty @ --to "$SOCK" send-text --match title:hover \
  'source devel/setup.bash && rosrun path_follow hover_origin 0 0' 

#arm
kitty @ --to "$SOCK" send-text --match title:arm \
  'source devel/setup.bash && rosservice call /mavros/cmd/arming "value: true"' 

#offboard
kitty @ --to "$SOCK" send-text --match title:offboard \
  'source devel/setup.bash && rosservice call /mavros/set_mode "custom_mode: 'OFFBOARD'"' 

#posctl
kitty @ --to "$SOCK" send-text --match title:posctl \
  'source devel/setup.bash && rosservice call /mavros/set_mode "custom_mode: 'POSCTL'"'  

#compile  
kitty @ --to "$SOCK" send-text --match title:rviz \
  'rviz' 

#compile  
kitty @ --to "$SOCK" send-text --match title:compile \
  'sudo catkin build' 

exit