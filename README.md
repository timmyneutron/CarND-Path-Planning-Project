![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)

# Udacity Self-Driving Car Nanodegree: Term 3
# Project #1: Path Planning

## Introduction
This is a project for Udacity's Self-Driving Car Nanodegree. It uses path planning algorithms to drive a simulated car along a highway, following lanes, avoiding collisions, obeying the speed limit, and safely passing other cars.

## Concepts
Concepts explored in this project:

  - Finite state machines and state transitions
  - Behavior planning
  - Trajectory generation
  - Frenet coordinates

## Viewing the Project
To watch a video of the controller in action, click [here](https://www.youtube.com/watch?v=2sHYJWjG5rY).

Source code is located in the `src` folder.

## Write-Up

### Objective

The goal of this project is to navigate a simulated self-driving car down a highway, while following lanes, obeying the speed limit, avoiding collisions with other cars, safely passing and changing lanes, and not exceeding the maximum allowable acceleration and jerk.

### Simulator
The simulator passes the car's global (x, y) and Frenet (s, d) coordinates, yaw angle, velocity, etc, to the path-planning program, along with information about the position and velocity of the other cars on the highway.

The path-planning program must take this information and generate points for the car to move to every 0.02 seconds.

### Lane Following
The first objective was to get the car to follow a lane at constant speed. This was accomplished by using a cubic spline (`spline.h`, documentation [here](http://kluge.in-chemnitz.de/opensource/spline/))

The spline takes in a list of points, and fits a curve to them that passes through each point, while keeping the first and second derivatives continuous. This is ideal for generating paths that minimize acceleration and jerk.

The first two spline points are the last two points of the previous path returned by the simulator (or, in the starting case, the location of the car and a single point directly behind it). The next three points are points 30, 60, and 90 meters down the road in the middle of the target lane (ie, keeping the Frenet d coordinate constant for different values of the s coordinate). These points are converted from (s,d) coordinates to global (x,y) coordinates, and then added to the list of spline points.

However, y is not always a function of x in the global frame (because the car moves in a large circle), so before the spline can be set, the spline points must be converted to the car's reference frame (with the x axis pointing straight ahead, and the y axis pointing to the left). Once this is done, the spline can be set, and points can be added on to the end of the planned path that follow the spline, and converted back into the global frame.

The spacing of the points along the spline is simply the target velocity multiplied by 0.02 seconds (the time the simulator takes to move to each successive point).

### Acceleration and Jerk
The car can now follow the center of a lane in a smooth curve at constant velocity, but exceeds the maximum allowable acceleration and jerk when starting from rest. To fix this, instead of immediately setting to the target velocity, the acceleration is set (to approximately 5 m/s^s), and the change in velocity is set from that. Likewise, if the car needs to slow down, the acceleration will be negative. This keeps the car from exceeding max acceleration and jerk.

### Collision Avoidance
The path-planning program loops through the list of other cars, checking their positions in (s,d) coordinates. If another car is found to be less that 30 m ahead in the same lane, the car will either change lanes or slow down to avoid hitting the car ahead of it.

### Changing Lanes
To change lanes, the car checks the (s,d) coordinates of all other cars. If another car is in an adjacent lane (d coordinate within a certain range), and is close to the self-driving car along the road (s coordinate within a certain range), that lane is flagged as blocked.

If the self driving car has a car in front of it in its lane, and either of the adjacent lanes are clear, it will switch lanes. It does this by changing the last three waypoints on the spline to the middle of the open lane, a bit further down (60 meters) to avoid exceeding maximum acceleration and jerk.

### Further Steps
This project succeeds in navigating the self-driving car along the highway, while obeying all the constraints set out by the project.

However, the response to slowing down and speeding up is a bit jerky, given that there's a lag between points being set on the end the path and the car moving to those points. This is mitigated by reducing the number of points in the path - 20 points is enough but not too much - but a more sophisticated algorithm adjusting the speed would also be beneficial in reducing the lag.