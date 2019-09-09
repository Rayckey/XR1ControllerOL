# XR1ControllerOL

## For Controlling the XR1 Hardware with ROS

### Compiling

0. Make sure you have XR1ControllerROS repo
1. Git the package into your workspace src/ folder
```
$ git clone https://github.com/Rayckey/XR1ControllerOL.git xr1controllerol
```
2. Source the correct environment varibles based on your system
```
$ source switch2TX2.bash
```
or 
```
$ source switch2X86.bash
```

3. Return to root if work space folder and build using catkin build (or catkin_make)
```
$ catkin build
$ source devel/setup.bash 
```



### Running

## Robot Control
1. The node that allows robot control is 
```
$ rosrun xr1controllerol actuator_bridge
```

## Control Groups
Each control group is identified by their first actuatorID:
	OmniWheels = 1,
	MainBody = 4,
	HeadBody = 8, 
	LeftArm = 11,
	RightArm = 18,
	LeftHand = 25,
	RightHand = 30


## Control Modes
There are a total of 9 control modes:
	DirectMode = 1,
	MoCapMode = 4,
	IKMode = 5,
	TrackMode = 6,
	StableMode = 7,
	AnimationMode = 8,
	DriveMode = 9,
	RoamMode = 10,
	TeachMode = 11
##dependencies
sudo apt-get install libatlas-base-dev
 
### 1. Direction Mode
The user can directly control the robot via ros messages.
The actuators are set to profile position mode

### 4. MoCap Mode
The user can directly control the robot via Joint State Topics.
The actuators are set to profile position mode

### 5. IK Mode
The inverse kinemaitcs planner mode
The user can assign IK targets through the IK services
The actuators are set to position mode

### 6. Track Mode
Don't actually use this yet 
The actuators are set to position mode

### 7. Stable Mode
Useable for left and right arm.
The arms will try to maintain their hands position at the moment they receive the commands
The actuators are set to position mode

### 8. Animation Mode
Use the data from animation linrary
Any group that gets assigned this mode will immediately start playing animation
The actuators are set to position mode

### 9. Drive Mode
Not implemented yet, meant to be used for driving.
Wanted to do that peper thing but we ran out of motion capture units


### 10. Roam Mode
Only usable on the OmniWheels.
Enable driving with the combination of this node: Tele_FPS_Cmd
The actuators are set to velocity mode
 
### 11. Teach Mode
Trigger Gravity Compensation Mode.
Will deactivate collision detection
ONLY WORKS WITH BOTH ARMS
The actuators are set to current mode


You can switch the current control modes with the ChainModeChangeMessage
remember you need both the control group id and the mode id


## Animation
As soon as any body group enters AnimationMode animation will start playing
Animation will not affect any control group not in AnimationMode or DirectMode
If you want to set animation mode to all control group, publish a message to "/startAnimation"

### Animation type
There are currently three animation type:
Idle, Animation, Teach.
You can queue the animations with /setAnimation topic.


# Collision Detection
## Set Collision Detection Mode
You can start collision detection mode with the topic /setCollisionDetection
Any control group in gravity compensation mode will switch mode, as the two calculation cannot co-exist

## Deactivate Collision Detection
When the robot has entered lock down mode (It detected collision)
Set the collision Detection to false to lift the lockdown


# XR1ControllerOL

