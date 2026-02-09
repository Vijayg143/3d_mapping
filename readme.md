# 3D Mapping Workflow
# Step 1: Start Gazebo (Terminal 1)
cd ~/3d_rover
source install/setup.bash
ros2 launch k12_description gazebo.launch.py

# step 2: to run lio-sam and opens the rviz (Terminal 2)
ros2 launch k12_description lio_sam.launch.py

# Step 2: Start 3D Mapping (Terminal 3)
cd ~/3d_rover && source install/setup.bash
python3 src/k12_description/scripts/mapping_node.py
                #or
source ~/3d_rover/install/setup.bash
ros2 run k12_description mapping_node

# Step 3: Drive Around (Terminal 4)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# in this terminal - ros2 run k12_description mapping_node
Key	Action
s	Save map to PCD file + auto-stop recording
l	Load existing PCD file
e	End/Pause recording
r	Resume recording
c	Clear accumulated points
q	Quit node

# to see the map using pcd
pcl_viewer ~/3d_rover/maps/map_*.pcd
