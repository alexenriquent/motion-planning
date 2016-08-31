# Motion Planning
A wheelchair that can better assist people who are paralyzed from neck down has been developed. Currently, such a wheelchair requires manual control by the user, sometimes using their head and eye movements. UQDS would like to improve such a wheelchair by enabling it to automatically perform the more mundane movement on its own, such that the user only needs to tell where he/she wants to go or what the robot arm on the wheelchair should do. This is where Motion Planning technology can help.

### The System
The simplified wheelchair operates in a 2D workspace (rather than 3D). In particular, the 2D workspace is a plane, represented as [0, 1]X[0, 1] ⊂ R<sup>2</sup>, and is populated by rectangular obstacles. The exact dimension and position of each obstacle in the environment is known prior to execution.
The wheelchair may be composed of three parts: The chair (base), a robot arm, and a gripper. The details of each part are:
* **The chair** can be thought of as a base for the arm (and gripper) and is a square of size 0.04X0.04 unit<sup>2</sup>. The base can only translate. A local coordinate system is attached to this base. The origin of this coordinate system is at the center of the square, while the X and Y axis are parallel to the X and Y axis of the workspace.
* **The robot arm** forms a chain and consists of n links and n joints, where n is a non-negative integer. Each link is a line segment attached to a joint. We number the joints and links sequentially (with the index starting at 1), i.e., joint-1 is located at the center of the base, link-1 is a line-segment attached to joint-1, joint-2 lies at the other end of link-1, etc. Each joint is a rotational joint. A local coordinate system is attached to each joint. The origin of this coordinate system is the position of the joint. For joint-1, this coordinate system coincides with the coordinate system of the base. For other joints, the X axis is the line that coincides with the previous link. Each joint is a rotational joint, which means it can only rotate. We define the joint angle of joint-i as the angle between link-i and the X axis of the coordinate system attached to joint-i. The joint angle of each joint is limited to be within [-150<sup>0</sup>, 150<sup>0</sup>]. Each link is of size 0.05 unit length.
* **The gripper** consists of 2 L shape segments: upper and lower gripper. Each gripper is attached to the last link of the arm and has a fixed relative orientation with respect to the last link of the arm. However, the length of the segments can be altered and is defined as (u<sub>1</sub>, u<sub>2</sub>) for the upper gripper and as (l<sub>1</sub>, l<sub>2</sub>) for the lower gripper, where u<sub>1</sub>, u<sub>2</sub>, l<sub>1</sub>, l<sub>2</sub> ∈ [0.03, 0.07].

### Requirements
Given the initial and goal configurations of the wheelchair, as well as a map of the environment, the program must find a valid path from the initial to the goal configurations. A valid path means that when the wheelchair executes the path, it will satisfy the following requirements:

1. The path consists of primitive steps. In each primitive step, the chair cannot move more than 0.001 unit, each joint angle cannot move more than 0.1<sup>0</sup>, and the length of a segment in the gripper cannot move more 0.001 unit.  
2. It will not collide with any of the obstacles.  
3. It will not collide with itself.  
4. The entire wheelchair must lie inside the workspace.
5. The joint angles (for the arm) and the segment length (for the gripper) must be inside their respective lower and upper limit (i.e. [-150<sup>0</sup>, 150<sup>0</sup>] as described in the previous section and [0.03, 0.07] for the segment length).
6. Since a primitive step is very small, it is sufficient to satisfy requirements
2-5 at the beginning and end of each primitive step.

### Input and Output format
**Format of a configuration:** A configuration is represented by n real numbers, where n is the dimension of the C-space. Each number is separated by a white space. The first two numbers are the position of the origin of the chair’s coordinate system in the workspace. If the wheelchair does not have a gripper, the last n-2 numbers are the joint angles in sequential order (i.e., the third number is the joint angle of joint-1, the fourth number is the joint angle of joint-2, etc.). Each joint angle is defined in radian. If the wheelchair has a gripper, the subsequent n-6 numbers are the joint angles in sequential order, while the last 4 numbers are the values of u<sub>1</sub>, u<sub>2</sub>, l<sub>1</sub>, l<sub>2</sub>, respectively.

**Input.** The program accept an input file. The file contains the type of wheelchair, the initial and goal configurations, and the obstacles position and dimension. The format of the input file is as follows.

1. The file consists of k + 4 lines, where k is the number of obstacles in the environment.
2. The first line is the type of wheelchair. There is only two possibilities, i.e., withGripper and noGripper for a wheelchair with a gripper and that without a gripper, respectively.
3. The second line is the initial configuration.
4. The third line is the goal configuration.
5. The fourth line is the number of obstacles in the environment.
6. Each line in the next k lines represents an obstacle and consists of 4 real numbers. The first two numbers represent the X and Y position of the upper-left vertex of the rectangle, while the last two represent the X and Y position of the lower-right vertex of the rectangle.

**Output.** The program outputs the path to a file with the following format.

1. The file consists of m+2 lines, where m is the number of primitive steps in the path.
2. The first line is the number of line-segments.
3. The second line is the initial configuration.
4. The next m lines are the end configuration of each primitive step.
