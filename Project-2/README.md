# Project 2
In this project we consider a ROS ecosystem, which consists of a robot with a camera mounted on it as well as an object. To describe the poses of all these items, we define the following coordinate frames:

* A base coordinate frame called 'base'
* A robot coordinate frame called 'robot'
* A camera coordinate frame called 'camera'
* An object coordinate frame 'object'

The following relationships are true:

The transform from the 'base' coordinate frame to the 'object' coordinate frame consists of a rotation expressed as (roll, pitch, yaw) of (0.79, 0.0, 0.79) followed by a translation of 1.0 meter along the resulting y-axis and 1.0m along the resulting z-axis.

The transform from the 'base' coordinate frame to the 'robot' coordinate frame consists of a rotation around the z-axis by 1.5 radians followed by a translation along the resulting y-axis of -1.0m.

The transform from the 'robot' coordinate frame to the 'camera' coordinate frame must be defined as follows: The translation component of this transform is (0.0, 0.1, 0.1) The rotation component this transform must be set such that the camera is pointing directly at the object. In other words, the x-axis of the 'camera' coordinate frame must be pointing directly at the origin of the 'object' coordinate frame.

Write a ROS node that publishes the following transforms to TF:

The transform from the 'base' coordinate frame to the 'object' coordinate frame
The transform from the 'base' coordinate frame to the 'robot' coordinate frame
The transform from the 'robot' coordinate frame to the 'camera' coordinate frame

## Additional Information

For a rotation expressed as roll-pitch-yaw, you can use the `quaternion_from_euler()` or `euler_matrix()` functions with the default axes convention - i.e. `quaternion_from_euler(roll_value, pitch_value, yaw_value)`.

The transforms must be published in a continuous loop at a rate of 10Hz or more.

Once you run your code, these bodies will position themselves in space according to the transforms your code is publishing. The cylinder denotes the object, the cube and arrow the robot and camera respectively. If your code works correctly, you should see the arrow point out of the cube directly at the cylinder. Here is an example of the correct output (note that the colored axes show you the location of the base coordinate frame with the usual convention: x-red, y-green, z-blue):

