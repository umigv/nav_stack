## GPS Setup Instructions
### To find serial port for GPS:
1. Create two `.txt` files: `before_gps.txt` and `after_gps.txt`
2. Before plugging in the GPS, run the following command: `ls /dev > before_gps.txt`
3. Now plug in the GPS, and run:
`ls /dev > after_gps.txt`
`diff before_gps.txt after_gps.txt`
4. This will output all the new ports that were added after plugging into the GPS.
5. In `gps_coord_pub.py`, try each of the new ports when constructing the Serial object until you find the one that works.
### VMWare specific instructions:
- The serial port for the GPS is `/dev/ttyACM0`
- Each time you plug in the GPS, run: `sudo chmod a+rw /dev/ttyACM0`

### Read rosbag files into Foxglove Studio:
- You will need to convert the `.db3` file into a `.mcap` file
- [Importing Your ROS 2 Data Into Foxglove Data Platform](https://foxglove.dev/blog/importing-your-ros2-data-into-foxglove-data-platform)
