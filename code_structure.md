1) read in rgb data
2) convert to a grey scale image with everything being black except the line which is white
3) take the greyscale image and generate a trajectory 
4) publish the trajectory to cmd_vel

One option is to use OpenCV or a similar image processing / vision library together with camera looking forward and downward to do the following:

Place a tape of a known, unusual, and bright color such as yellow or orange. If you're willing to mount a light source on your robot, you might even use retroreflective tape.
Use OpenCV to break the color video image into three HSV planes - hue (color), saturation, and value (intensity).
Use a Stroke Width Transform to identify the line. The following StackOverflow post has a link to a video about the SWT: Stroke Width Transform (SWT) implementation (Java, C#...)
Apply a thinning algorithm (Stentiford or Zhang-Suen) to reduce the segmented tape line to a single pixel width.
Treat the single pixels as input points for a curve fit. Or, more simply, calculate the angle from end point to end point for every group of N successive points in your line.
Apply a little 3D geometry as well as information about your robot's current direction and speed to calculate the time at which it must turn, and how sharply, in order to follow the line.
If your robot moves slowly, then a camera looking downward might be more suitable. The calculations are easier, but the robot wouldn't be able to look ahead as far.
