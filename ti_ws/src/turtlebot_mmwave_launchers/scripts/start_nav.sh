#!/bin/bash

echo Script will loop until user presses CTRL-C or enters invalid argument...
while [ 1 ]
do

# Read input arguments
read -p 'Starting side? left (l) or right (r) ' starting_side
read -p 'Starting position? (a, b, c)? ' starting_pos
read -p 'Goal position on other side? (a, b, c)? ' goal_pos

# Check arguments
if [ $starting_side = l ]
then
  goal_side=r
elif [ $starting_side = r ]
then
  goal_side=l
else
  echo "ERROR: Starting side ($starting_side) must be 'l' or 'r'"
  exit
fi

if [ $starting_pos != a ] && [ $starting_pos != b ] && [ $starting_pos != c ]
then
  echo "ERROR: Starting position ($starting_pos) must be 'a' or 'b' or 'c'"
  exit
fi

if [ $goal_pos != a ] && [ $goal_pos != b ] && [ $goal_pos != c ]
then
  echo "ERROR: Goal position ($goal_pos) must be 'a' or 'b' or 'c'"
  exit
fi

echo Start: $starting_side\_$starting_pos Goal: $goal_side\_$goal_pos
rostopic pub --once /initialpose  geometry_msgs/PoseWithCovarianceStamped -f init_pose_$starting_side\_$starting_pos.yaml
rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped -f goal_$goal_side\_$goal_pos.yaml

done
