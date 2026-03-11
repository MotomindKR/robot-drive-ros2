#!/bin/bash

# Prompt user for input (either foxy or humble)
read -p "Enter ROS 2 version (foxy or humble): " ros_version

# Validate input
if [[ "$ros_version" != "foxy" && "$ros_version" != "humble" ]]; then
  echo "Invalid input. Please enter either 'foxy' or 'humble'."
  exit 1
fi

# Define the alias line based on the version input
alias_line="alias source_ros2='echo \"Sourcing ROS 2 $ros_version setup...\" && source /opt/ros/$ros_version/setup.bash'"

# Check if the alias is already present in ~/.bashrc
if ! grep -q "$alias_line" ~/.bashrc; then
  # If not, append the alias line to ~/.bashrc
  echo "$alias_line" >> ~/.bashrc
  echo "Alias for ROS 2 $ros_version added to ~/.bashrc."
else
  echo "Alias for ROS 2 $ros_version already exists in ~/.bashrc."
fi

# Reload ~/.bashrc to apply the changes
source ~/.bashrc
echo "source ~/.bashrc"
echo "You can now use the 'source_ros2' alias to source the ROS 2 environment."
