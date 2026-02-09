# 3D Mapping Workflow

# Step 1: Start Gazebo (Terminal 1)
```
cd ~/3d_mapping

source install/setup.bash

ros2 launch k12_description gazebo.launch.py
```

# Step 2: To run lio-sam and opens the rviz (Terminal 2)

```
source install/setup.bash

ros2 launch k12_description lio_sam.launch.py
```


# Step 2: Start 3D Mapping (Terminal 3)
```

cd ~/3d_mapping && source install/setup.bash

python3 src/k12_description/scripts/mapping_node.py
```

#or
```                
source install/setup.bash

ros2 run k12_description mapping_node
```

<img width="1920" height="1080" alt="Screenshot from 2026-02-03 22-30-30" src="https://github.com/user-attachments/assets/8902bd2d-155c-4f64-b92b-fc3f4859497f" />

# Step 3: Drive Around (Terminal 4)
```
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

<img width="1920" height="1080" alt="Screenshot from 2026-02-06 11-52-55" src="https://github.com/user-attachments/assets/429116b5-b672-4ba1-8ea2-ed40a350d4ad" />

# In this terminal - ros2 run k12_description mapping_node

Key	  --   Action

 s	  --    Save map to PCD file + auto-stop recording

 l    --  	Load existing PCD file

 e	  --    End/Pause recording

 r	  --    Resume recording
 
 c	  --    Clear accumulated points

 q	  --    Quit node


# To see the map using pcd
```
pcl_viewer ~/3d_mapping/maps/map_*.pcd
```

[Screencast from 02-06-2026 12:18:20 PM.webm](https://github.com/user-attachments/assets/5befdf22-317c-41e6-bf79-2817db81c0bb)


