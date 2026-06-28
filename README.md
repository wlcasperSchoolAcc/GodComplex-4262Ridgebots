## GodComplex-4262Ridgebots

Hello and welcome,

This is GodComplex (name subject to change pending vote circa 9/24/2026), an FTC library (well, more like a group of programs) that is meant to offer a more intuitive, simpler way to enable robot movement across spline curves. It was made by me, and made for 4262 Ridgebots. This file is built to be a simple yet effective tutorial regarding this program. It's long enough to give details, and short enough that you won't bore yourself. 

4262 Ridgebots and 9266 Pyrobots are teams of Pacific Ridge School, in Carlsbad, CA. Show up to some of our matches if you'd like.

Good luck, and have fun.

# Overview
Put simply, you put in 4 points, of x and y, and specify a heading, theta. The first and last points are your start and end. The second and third points build a Bezier Curve (shown in Curves, if you want to figure out the algorithm yourself). The robot might not explicitly pass through these secondary points, but they influence the robot's path, allowing it to maneuver around obstacles in one smooth motion

# Hardware
The robot this was first tested on is a parallel plate chassis, with Yellowjacket motors from Gobilda. It utilizes the Gobilda Pinpoint Odometry Computer for its imu, as opposed to the Control Hub imu. The odometry wheels are the Gobilda 4-bar variant, although it is not hard to change it to the swing-arm variation. Personally, I reccomend 

# Downloads and Setup
You will need every single Java file for this to work. Furthermore, you will need to check the offsets of your odometry pods for the rotation system to work. Grab a ruler, and measure in millimeters. I repeat, MM, from the odo-pods to your robot center.

Next, ensure your GoBilda odometry computer is set up. You will need a test code to check which directions the odometry pods are in. After setting that up, move the robot in the direction of the arrows. The values should increase positively. If it does not, reverse the direction in CustomDrive. Additionally, make sure your motors are set up so that when going forward, all 4 motors have the same power values (which should be 1). If they are not, reverse some of them.

# Necessary Disclaimer
If at this point you're asking me "How do I reverse a motor", you should not be looking at this, and should instead focus on learning the FTC documentation first.

Oh, and if you're wondering about all the imports across all the programs. About 90% of those are useless and redundant. They are kept for convenience, since all those imports also happen to be enough for just about any Auton or TeleOp you write. Feel free to copy-paste them whenever.

# MoveScum
You only need to know two values here. MaxV and MaxA. They stand for velocity and acceleration respectively. Basically, they control how fast your robot starts to move and its top speed. If they are too high, the robot may just catapult itself into a wall, so be careful with this. Do not change anything inside the actual program, only through GodObject.

# GodObject
After setup, this should be the only program you change on a regular basis. As stated in Overview, you will see a part with 4 points. Those are where you actually control the robot movements. There are two as of right now. To add more, simply make more Curves Objects, and more MoveScums if you need. Then, call followPath() inside the Opmode if statement. 

Next, you will have to tune the PID values. It's different for each robot, so the values I give will automatically be wrong for your robot. Of course, you can do it through this program, but it's reccomended to do it through a separate tuner TeleOp and the FTC dashboard. Tutorials can be found online.
kV is it's feedforward component, basically just gives the robot an extra kick when it moves. Do with it whatever you wish.

Now, apologies for whiplash, but your movement values are now in INCHES. I repeat, INCHES. A foam mat is 24 inches x 24 inches. I expect you to figure out the rest. They are in doubles, so feel free to mess around with decimals.

# followPath()
followPath() takes 3 values:
path: the path you want it to move along, should be a Curves object
profile: the "motion profile" of your path following. Should be a MoveScum object
heading/theta: What angle you want your robot to be facing when moving. Note that this will execute first before any other movements. 0 will be the direction you face it when you init the robot.

# You absolute clowns.
That should be all the important bits. 
Special thanks to 9266 Pyrobots and 4262 Ridgebots for moral support. I expect you bums to reach Worlds next year or I'll be disappointed.
This was a completely homebrew project for my team, I expect my teams, if no one else, to be using this library at least until a better option is found or created.

Zhixun Wang, out.
