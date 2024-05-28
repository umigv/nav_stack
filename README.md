# nav_stack
Clone this repo into src directory. 
<<<<<<< HEAD

## Cv-integration
Feature Team Members: 
 - Oscar (@oscarran)
 - Matt (@mattpri)
 - Christian (@chrc)

### Motivation
Take in computer vision lane lines, these lane lines must be tracked and merged with obstacle information from
sensors. The CV-integration team must keep track of the lane lines and save a history. The history + current lane lines are mergered as a result. This final output occupancy grid now contains current obstacle information
in addition to all lane line information.

### Implementation
**GOAL:** Fetching a local(CV) and global(SNSR) occupancy grids. Utilize these two occupancy grids to create a global(NAV) that will pertain information while updating X buffer_time. This update will reflect on the global(NAV) map.

**Step 1**: Implmenting a subscriber node that obtains information from local(CV) and global(SNSR) occupancy grids.

**Step 2**: Create callback functions that transfer information from local(CV) and global(SNSR) occupancy grids to variablized grids.

**Step 3**: Utilize tf2 based off GPS data from global(SNSR) to transform points from local(CV) and global(SNSR) to the world frame.

**Step 4**: Incorporate a callback that merges local(CV) and global(SNSR) into a global(NAV) map.

**Step 5**: Ensure the merge callback calls the other functions in X buffer_time.

### Interface

**Nav Occupancy Grid**:
- *FRAME:* World
- *Grid Type:* Global
- *Size:* Same as SNSR map.
-  *Content:* Contains a history of lane lines to be merged with the SNSR Occupancy Grid.

**CV Occupancy Grid**:
- *FRAME:*
- *Grid Type:* Local
- *Resolution:* 0.05 meters
- *Content:* Contains lane lines.
- *Additional Info:* Robot position within the occupancy grid. Robot can be at (0,0) or any other location
                     as long as this is specified in the message. This allows for allignment of grids.

**SNSR Occupancy Grid**:
- *FRAME:* World
- *Grid Type:* Global
- *Size*: Determined by SNSRs
- *Content:* All obstacle information from slam.
=======
main


## Tele-Operation with PS4 controller
### Launching
1. Open new terminal.
2. run: `source /opt/ros/humble/setup.bash`
3. run: `source install/setup.bash`
4. run: `ros2 launch sam_bot_description teleop_launch.py`

### Controls
Enable Button: R1\
Turbo Button: Triangle\
Forward / Backward: Left Stick\
Rotate Left / Rotate Right: Right Stick\
![PS4 Contoller Button Mapping](https://github.com/umigv/nav_stack/assets/97559965/e4291dfd-a7ec-4eb5-8f61-66de1d4bf5fe)
>>>>>>> main
