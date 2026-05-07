#!/bin/bash
sed -i 's/if (nav_in_progress_) {/if (false) { \/\/ nav_in_progress_ disabled/g' turtlebot3/turtlebot3_ball_collection/src/density_map_builder_node.cpp
sed -i 's/if (spin_in_progress_) {/if (false) { \/\/ spin_in_progress_ disabled/g' turtlebot3/turtlebot3_ball_collection/src/density_map_builder_node.cpp
sed -i 's/if (spin_in_progress_ || nav_in_progress_) {/if (false) { \/\/ Both disabled/g' turtlebot3/turtlebot3_ball_collection/src/density_map_builder_node.cpp
