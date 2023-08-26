# touchoff
ROS package for finding a base frame using a 6DOF robot and a contact sensor

Install into your catkin workspace by cloning into src/, and running:
rosdep install --from-paths src --ignore-src -r

To run, copy the touchoff.launch file into your machine package and update necessary parameters.

A simple arduino-based electrical touch probe is provided using rosserial;
to install, open the touchoff_probe.pde file in the arduino IDE and upload.
You may have to change the port and baud rate - this can be found in the IDE.
Pin 13 is pulled up, and on making contact with ground (through the tool or base)
the /contact message will flip from False to True.
Any contact probe should work as long as it publishes a boolean ROS message.

Currently the touchoff is operated through terminal. Usage is:
(axes are x,-x,y,-y,z,-z, distance is in meters, rotation is in degrees, bracketed are optional)

- jog AXIS DISTANCE - move along AXIS in the toolhead coordinate frame by DISTANCE
- rotate ROTATION_AXIS ANGLE_DEGREES - rotate about the ROTATION_AXIS by ANGLE_DEGREES
- axis TOUCHOFF_AXIS [search distance] - slowly move along TOUCHOFF_AXIS until contact is detected, and save measurement
- angle TOUCHOFF_AXIS TRAVERSE_AXIS [distance_between_points_m] - measure rotation angle by touching along TOUCHOFF_AXIS at two points along the TRAVERSE AXIS
- (future) plane [x,-x,y,-y,z,-z] [distance_between_points_m] - touch a grid of points to get a plane rotation
- align - after measuring an angle, rotate the toolhead to align with the base
- accept [frame_name] - write a launch.xml file to store touchoff as a static transform
- corner AXIS1 AXIS2 AXIS3 [search_distance] - touch off three axes about an outer-facing corner, see diagram
- (future) edge 
- (future) corneredge
- Quit with ctrl+c or q

