# nav_stack
Clone this repo into src directory. 


If your looking for the nav/sensors onboarding go here instead: https://github.com/umigv/nav-onboarding-2024
# Follow the installtion here to get dependinces:
https://docs.nav2.org/getting_started/index.html

# Link to current projects 
https://docs.google.com/document/d/10k_sBHWKxRfVZ4LImK5-UW2cgy3Yt_E14yCGoBRWqZw/edit?usp=sharing

## Tele-Operation with PS4 controller
### Launching
1. Open new terminal.
2. run: `source /opt/ros/humble/setup.bash`
3. run: `source install/setup.bash`
4. run: `ros2 launch marvin_bot_description teleop_launch.py`

### Current Controls
Enable Button: L2\
Turbo Button: L1\
Forward / Backward: Left Stick\
Rotate Left / Rotate Right: Right Stick\
![PS4 Contoller Button Mapping](https://github.com/umigv/nav_stack/assets/97559965/e4291dfd-a7ec-4eb5-8f61-66de1d4bf5fe)
=======

### Rebinding Buttons
1. Follow `Launching`
2. Open new terminal.
3. run `ros2 topic echo /joy`
4. Press the button you want to bind a control to
5. Look for where a `1` appears. The row of the buttons list that it appears in is the button id, zero-indexed.
6. Replace the button id within `marvin_bot_description/config/teleop_configs/umarv.config.yaml`

### Modifying Scales
1. Look for the desired scale within `marvin_bot_description/config/teleop_configs/umarv.config.yaml`
2. Replace it with your desired scale.

**IMPORTANT**: the scale must end in .0 if it does not have any digits below the ones place (e.g. scale of 5 must instead be 5.0)

### Running Nav Stack on Razor
1. Open 3 terminals.
2. `cd ~/ros2_ws` in each terminal.
3. run `source /opt/ros/humble/setup.zsh` && `source install/setup.zsh` in each terminal.
4. Terminal 1: `ros2 launch marvin_simulation simulation.launch.py world:=igvc_flat`
5. Terminal 2: `ros2 launch src/nav_stack/launch/nav_stack.launch.py`
6. Terminal 3: `ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "odom"` \
**Note the static transform publisher will not need to be called when running the sensors stack.**
   
