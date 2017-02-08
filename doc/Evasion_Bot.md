# Evasion Bot
This documentation described the approches of our evasion behavior on turtlebot 2 (Kobuki) for the Evasion & Pursuit Competition.

## Overview
The Evasion Bot is a robot (turtlebot 2) that will run away from a following robot under a collision free condition. Specifically, it will be to wander in an environment (the match room) in a manner that prevents another robot from being able to follow it. In other words, a successful evasion bot should be able to move swiftly in the environment without any collisions. 


## Resource Set Up
We used a single PremeSence RGB-D sensor (Asus Xtion Pro) for our evasion bot. This sensor can provide us depth image and RGB image. The field of view that be provided is 58 degree horizontally and 45 degree vertivally. The distance range that it can detact is from 0.7 meters to 3.5 meters according to [the product document](https://www.asus.com/3D-Sensor/Xtion_PRO_LIVE/specifications/). However, we found in pratice the minimum detactable distance could be extend to 0.45 meters.

We mounted our RGB-D sensor at rare of our turtlebbot 2 since this could minimize the blind area of depth image. As per our measurement, the maximum width (diameter) of turtlebot 2 is about 35.5 cm. So, when we mount our RGB-D sensor at rare and sensing the front direction of robot, there will be only roughly 15 cm ahead the robot is blind area. This is a great improvement comparing to the 45 cm minimum detactable distance.

Our turtlebot 2 is distributed by Clearpath Robotics. Referred to the [Kobuki User Guide](https://www.google.ca/url?sa=t&rct=j&q=&esrc=s&source=web&cd=9&ved=0ahUKEwjvhqLH1v_RAhVOw2MKHYAFAY4QFghCMAg&url=https%3A%2F%2Fdocs.google.com%2Fdocument%2Fexport%3Fformat%3Dpdf%26id%3D15k7UBnYY_GPmKzQCjzRGCW-4dIP7zl_R_7tWPLM0zKI&usg=AFQjCNFo0O5d312q_k2JDorv5Q0cIMiZ7A&bvm=bv.146094739,d.cGc&cad=rja), the maximum linear speed of our turtlebot is 0.7 m/s, and the maximum angular speed is 180 degree/s.


## Problem Meet
Basically, there are two main problems we need to solve -- dead end avoidance and blind spot avoidance.

Dead end happens when turtlebot faces to a corner and turning process cannot make it turn around. For example, when turtlebot detect the wall is very close to right side, it turns left. If the turning angle is not large enough, it will detect the wall at left is so close that it has to turn right again.
## {pic: deadend}

However, we cannot just raise the turning angle because if the turning angle is too large, turtlebot will turn too much which causes the other problem -- blind spot.

Blind spot happens when an object is too close to the turtlebot so that turtlebot cannot detect cannot detect the object. It often happens when turtlebot turning. For example, when the turtlebot turns right, and there is an object at its right side, the turtlebot can only detect area behind the object.
## {pic: blindspot}


## Approching Details
Since the competition doesn't tolerate any collision (the robot that responsable for the collision will lose the round), saftey becomes the first priority. At the same time, we have to optimize our speed as fast as possible. Also, evasion bot should be able to go through a narraw path because it's possible that the evasion bot run into a small space (e.t. a corner of walls conbines the persuit robot at the back). 

##### Safety Movement
We use lidar scan data (similated by depth image) to keep our turtlebot 2 safe from collisions. The purpose of using simulated lidar scan data is because we believe lidar scan data will be easier and will excute fast in terms of performance since there are less unneccessary datas compare to depth image. The ROS package we use to convert the depth image to lidar scan is [depthimage_to_laserscan](http://wiki.ros.org/depthimage_to_laserscan).

Once we got the lidar scan data, we could prevent our robot to move forward if thers is a object is in less than a certain distance. This certain distance is the danger zone for robot to move ahead. Since lidar scan data is an array of distance coresponding to different degree of sensor, we only need to the minimum value of lidar scan data to figure out whether thers is a object is inside the danger zone. For those reason, the first approch of safety movement is letting our robot keep turning until there are nothing in it's danger zone. The range of danger distance could be variance in different environment, we pick 0.7 meter as our minimal safe distance. 

We will also use the minimum value of lidar scan data to guide the robot turing in a direction. To be specifically, the robot will turn left if the closest object is located at the front right (the min value index of lidar scan data array is larger then 320 in our case) of it, otherwise, it will turn right. The turning action will be triger if the minimum distance is less than 1.2 meter specifically in our competition. This approching will help robot heading to a open space so it's safer and there will be less chance stuck in a dead corner.

##### Speed Optimization
Speed is another key to win the competition. Therefore, maximum linear speed have been applied if that's safe. But, we only used 50% of the angular speed for two reasons. The first reason is turning too faster will cause over turning angle. This over turning angle might make robot heading to another unsafe direction. So, it will keep turning on the same spot, which is deadly. Secondly, faster turning has high possibility to crash into an object that hide in the blind area of robot's RGB-D sensor. The view field of our RGB-D sensor is 58 degree horizontally. Therefore, the robot won't have enough lidar scan data to determine if it's safe when turning, especially turning and moving forward at the same time. 

##### Ability to Servive in a Narraw Spaces
Ability to servive in narraw spaces is critical for two reasons. One, the ecasion bot will lose if the persuit successfully follow it for more than 1 minute. And, stuck in a narraw space will guarantee that happens. Two, if the evasion robot able to move through a narraw path, then it's more challenge for persuit bot to follow it safely. Also, those near obstacles might obstruct the following algorithm on persuit bot.

Based on our safety rules, our robot won't go forward if there is a detacted object in less than 0.7 meter no matter whether this object will indeed block it path. That being said, our robot cannot go through a narraw path and servive in a narraw space. So, we have to change our safety determination machanism. In other words, we have to narrow down the danger zone to only the front area of the robot. That means, only if the closed object, that will block robot in front of it's path, could be consider as dangerouse. 

Our implementation of this feature will use lidar scan data as well. We filtered out the datas that's not in front of robot from lidar scan data, then apply the same machanisum that we do to make safety movement. As our code below,

```python
angle = (abs(dist_idx - 320) / 320.0 ) * 29
dist_to_center_axis = dist * math.sin(math.radians(angle))
if dist_to_center_axis < 0.28:
	range_ahead.append(dist)
```

We calculate an ```angle``` between the ray from sensor to the detacted point and the central axis of the robot based on the information of the sensor view field. Then, we can calculate the distance from this detacted point to central axis of the robot by using range data and the angle to solve a triangle problem. Now, all the detacted points that has less distance to the central axis than the robot's half width will actually be in front of robot. 

By doing this, our robot is able to go through a narraw path which improve the servive ability in a narraw spaces.


## Result and Discussion
As in our tests and competition, our robot is able to move swiftly without collision. And, it's able to servive in a narraw space (this ability helped us winning one of the rounds). However, it seems problematic if we plan too much for our evation behavior. Those plans is actually became the model of our robot's movements. The robot will follow those kind of pattern so the behavior is more predictable and less randomly. During the competition, we found that this problem will have significant impact of the game result. 

After this competition, we have learned that the robot sensor data is kind of noise in terms of realiability. It's hard to get a persice data twice in a pratice sensor usage. Due to the limitation of the sensor and/or the hardware of robot, we have to apply addition process on the raw sensor datas. 

Another experience we gain is on competition strategy aspect. It turns out having multiple plans will have a huge advantage when competiting with different competitor. There are usually some advantages and disadvantages of a robot and/or algorithm. The weak spot and advantages might be different on each competitor's robot and/or algorithm. Multiple plan will maximize the possibility to catch others' weak spots and avoids others' advantage in game. For those reasons, the chance to win a game will be increased if you have multiple plans.


## Conclusion
