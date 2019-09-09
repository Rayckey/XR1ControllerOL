# XR1ControllerOL

## For Controlling the XR1 Hardware with ROS

### Compiling

0. Make sure you have XR1ControllerROS repo
1. Git the package into your workspace src/ folder
```
$ git clone https://github.com/Rayckey/XR1ControllerOL.git xr1controllerol
```
2. Return to root if work space folder and build using catkin build (or catkin_make)
```
$ catkin build
$ source devel/setup.bash 
```



## Running

### Robot Control
1. The node that allows robot control is 
```
$ rosrun xr1controllerol actuator_bridge
```

### Control Groups
Each control group is identified by their first actuatorID:
	OmniWheels = 1, <br>
	MainBody = 4, <br>
	HeadBody = 8,  <br>
	LeftArm = 11, <br>
	RightArm = 18, <br>
	LeftHand = 25, <br>
	RightHand = 30 <br>


### Control Modes
There are a total of 9 control modes:
	DirectMode = 1, <br>
	MoCapMode = 4, <br>
	IKMode = 5, <br>
	TrackMode = 6, <br>
	StableMode = 7, <br>
	AnimationMode = 8, <br>
	DriveMode = 9, <br>
	RoamMode = 10, <br>
	TeachMode = 11 <br>
 
### 1. Direction Mode
The user can directly control the robot via ros messages. <br>
The actuators are set to profile position mode <br>

### 4. MoCap Mode
The user can directly control the robot via Joint State Topics. <br>
The actuators are set to profile position mode <br>

### 5. IK Mode
The inverse kinemaitcs planner mode <br>
The user can assign IK targets through the IK services <br>
The actuators are set to position mode <br>

### 6. Track Mode (legacy)
Don't actually use this yet <br>
The actuators are set to position mode <br>

### 7. Stable Mode (legacy)
Useable for left and right arm. legacy <br>
The arms will try to maintain their hands position at the moment they receive the commands <br>
The actuators are set to position mode <br>

### 8. Animation Mode
Use the data from animation linrary
Any group that gets assigned this mode will immediately start playing animation <br>
The actuators are set to position mode <br>

### 9. Drive Mode
Usable for any control group deside  <br>
Wanted to do that peper thing but we ran out of motion capture units <br>


### 10. Roam Mode
Only usable on the OmniWheels. <br>
Enable driving with the combination of this node: Tele_FPS_Cmd <br>
The actuators are set to velocity mode <br>
 
### 11. Teach Mode
Trigger Gravity Compensation Mode. <br>
Will deactivate collision detection <br>
ONLY WORKS WITH BOTH ARMS <br>
The actuators are set to current mode <br>
When set to Omniwheels, will loose the wheels for you to push it <br>


You can switch the current control modes with the ChainModeChangeMessage <br>
remember you need both the control group id and the mode id <br>


### Animation
Animation will not affect any control group not in AnimationMode or DirectMode <br>
If you want to set animation mode to all control group, publish a message to "/startAnimation" <br>



### Animation type
There are currently three animation type: <br>
Idle, Animation, Teach. <br>
You can queue the animations with /setAnimation topic. <br>


# Collision Detection
## Set Collision Detection Mode 
You can start collision detection mode with the topic /setCollisionDetection <br>
Any control group in gravity compensation mode will switch mode, as the two calculation cannot co-exist <br>

## Deactivate Collision Detection 
When the robot has entered lock down mode (It detected collision) <br>
Set the collision Detection to false to lift the lockdown <br> 

## Dependencies
sudo apt-get install libatlas-base-dev





