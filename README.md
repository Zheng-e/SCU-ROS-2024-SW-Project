Introduction to Robotics Programming with ROS 
Assignment - Leader-follower formation control of multi robots
This coursework constitutes 50% of the total mark. The submission deadline is noon, Monday, 
8th July 2024.
I. Tasks
This assignment contains a task to be completed in a group of 2 or 3 in Python working with ROS Kinetic 
installed on Ubuntu 16.04. Before you start, create a ROS package called assignment_scu_groupid. All your 
files in your package should be organized with the following structure:
• launch folder: contains launch files
• scripts folder: contains your python code
• srv folder: contains your custom ROS service
• msg folder: contains your custom ROS message
• CMakeLists.txt: list of cmake rules for compilation
• Package.xml: Package information and dependencies
Formation control of multiple robots is one of the most important areas in robotics research. In this 
task, you are required to implement the basic concepts of leader-follower formation control using the 
turtlesim simulator which is part of the ROS installation. Multiple turtles will be spawned in the 
simulator, one of which is designated as the leader moving along an arbitrary trajectory. The other 
turtles as the followers keep track of the leader turtle in the desired distance and orientation.
1. Basic requirements:
• Using a launch file to spawn 3 turtles, namely scuGroupidLeader, 
scuGroupidFollowerA, and scuGroupidFollowerB, in the turtlesim simulator
• The leader turtle is located in the center with no rotation. The two follower turtles start at a 
random position and orientation as illustrated in Fig.1(a).
• Define a custom message namely GroupidLeaderMessage.msg as shown below used to 
send an instruction from the leader to two followers
Fig. 1. An illustration of leader-follower formation. i.e. followerA is 1 meter to the left of the leader and follower is 
1 meter to the right of the leader
• The leader starts with sending an instruction using the defined message to two followers asking 
them to move to the formation position, i.e. followerA is 1 meter to the left of the leader and 
followerB is 1 meter to the right of the leader as illustrated in Fig.1(b).
Hints: The formation control could be easily achieved by publishing additional frames to the 
leader turtle. Here is the tutorial you might find helpful.
• Once the leader detects that the two followers are in the required formation position, it starts 
to move in a random direction with a random pen color until it reaches the boundary of the 
turtlesim window. The two follower turtles should track the leader with the desired relative 
distance, maintaining a specific geometric arrangement (e.g. the V shape shown in Fig. 1(b))
• When the leader turtle approaches to the border, it will send another instruction (again using 
the defined message) to the followers. After that, the three turtles will return to their initial 
position with different pen colors using a proportional controller.
Hints: Here is a link which you can find how to use a proportional controller to move a robot to 
a specific location
• A Vodcast is required to be produced to accompany your implementation within which a brief 
walkthrough demonstration of your implementation and a walkthrough of the underlying 
source code, along with an associated commentary. Each member must take part in the 
demonstration vodcast to report their contributions to the solution.
2. Marking scheme
• Using a launch file to spawn 3 turtles at required positions (7 marks)
• Creating a custom ROS message, i.e. GroupidLeaderMessage.msg (5 marks)
• Communication between the leader and follower turtles using the defined message (8 marks)
• The leader turtle moves in a random direction (5 marks)
• The control of leader-follower formation (10 marks)
• Turtles return to their initial position using a proportional controller (10 marks)
• Implementation of any additional features (5 marks)
II. Submission guidelines
1. Create a readme file in PDF named i.e AssignmentReadme_Groupid.pdf in which the following 
information must be included otherwise your submission will not be awarded a mark.
• Step-by-step instructions to run your code
• Evidence of successful implementation by including the following screenshots
o A screenshot after running the following command – you should have 
something similar to the following screenshot
$ rosmsg show scuGroupLeaderMessage
o Screenshots after running your launch file
▪ A screenshot showing 3 turtles at their initial position as illustrated 
below
▪ A screenshot showing two followers move to its formation position 
after receiving the instruction from the leader as illustrated below
o A screenshot in which the leader turtle moves in a random direction with a 
random pen color and the two follower turtles track the leader with the 
desired relative distance.
o Screenshots show turtles return to their initial position when the leader turtle 
approaches to the border and sends the instructions as illustrated below
o Screen captures of code snippets showing how the instructions are sent by the 
leader to two followers 
▪ asking them to move to the formation position
▪ asking them to return to their initial position using a proportional
controller when approaching the border
o A screenshot depicting a diagram of the frames being broadcast by tf2 over 
ROS as illustrated below by issuing the following commands in a separate 
terminal
The tree will be saved as a pdf file and can then be viewed using evince
• A summary table (Table 1) outlining the code examples of where each requirement can 
be found.
$rosrun tf2_tools view_frames.py
$evince frames.pdf
Table 1: Code examples of where each requirement can be found
Requirement
Implemented as 
expected
(Yes/No)
Filename Relevant Line 
of Code
Using a launch file to spawn 3 turtles
Creating a custom ROS message
Communication between the leader 
and follower turtles
The leader turtle moves in a random 
direction
The control of leader-follower 
formation
Turtles return to their initial position 
using proportional controller
Implementation of any additional 
features
2. The completed ROS package along with the readme file must be zipped and submitted by noon, 
Monday, 8th July 2024. Your submission should be in the form of a single zip file called 
Assignment_Groupid.zip. Students must ensure that the submitted package can be 
uncompressed, opened and run in ROS on Ubuntu 16.04.
3. A copy of your Vodcast produced to accompany your implementation must be submitted by 
the deadline, i.e. Noon, Monday, 8th July 2024. Please ensure Please ensure that your vodcast 
conforms to the mp4 format, can be opened and viewed (with sound). The name of the 
submitted vodcast is given as Assignment_Groupid_VODCAST.
4. Each group must submit a signed ‘Team Contribution Declaration’ form to indicate what 
percentage contribution they made to the overall effort. For any submission deviating from an 
equal split in the effort, marks for this project will be scaled according to the declarations made. 
5. Please ensure that you keep a secure backup electronic copy of your assignment.
PLEASE NOTE: 
Avoid plagiarism: Feel free to use online search engines and be inspired by what you read. However, 
please do not try to pass off any downloaded code as your own work. You must acknowledge all sources. 
This includes information from the Internet. Plagiarism is a serious offence.
