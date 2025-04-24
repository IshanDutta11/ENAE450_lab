# turtlebot3_ws
Description: ENAE450 Final Project Group12 

Final Project Link: https://github.com/SYED-ABRARUDDIN/ENAE450/tree/main/Final_Project 

Setup Related Stuff: SETUP.md

Planning: PLANNING.md

# Random Thoughts
I think it might be best practice to not commit/push the build, install, and log folders. We should be building locally each time anyway, and I'm kinda creeped out by having my machines info uploaded. I ignored the build folders, I also added the other cloned repos as submodules. Check setup for how to deal with that. 

Check with Abrar about how much of the controls we can abstract and the other formal limitaitons: we can't use odom, tf, external packages, or any additional hardware. 

Be sure to comment your code. Don't use any timers. 

# Daily Logs:
2025/4/24 - We discussed hte various algorithmic ideas. TLDR everyone is cringe, so we modified our ideas to make more sense. We will operate from a 4 cardinal directions and states perspective, no intelligent controls framwork. We wont do anyhting by timer, we will make ordered decisions by matching to the approximate NSEW match. Ishann will work on the ros interacing, Soham the algorithm,and Sai the lidar but he also will do weird stuff on his own time that may or may not work.

2025/4/17 - Task was to traverse the maze setup. Worked with the lidar. Got distance readings off of the scan ranges. Made a couple plans for how to work with it. 

2025/4/10 - Second turtlebot lab. Non Ubuntu continued to not work. Published to cmd_vel the instruction to move, got it pretty much .3m. Did not do obstacle avoidance with scan topic yet. 

2025/4/3 - First turtlebot lab. Only one Ubunut machine. Ran through setup steps. Were able to teleop with the keyboard and see the visual for the lidar point cloud. tmux was weird, non Ubuntu didn't work.
