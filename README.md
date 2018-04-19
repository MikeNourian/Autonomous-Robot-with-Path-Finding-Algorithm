# Autonomous-Robot-with-Path-Finding-Algorithm

This autonomous robot is designed for an artificial city where nodes represent the centers of attraction and the weights represent the traffic in the city. We use Pixy camera, Arduino Uno, Sensors and 2 motors and a chassis and run the shortest path algorithm to keep track of the location and find the path.

![final2](https://user-images.githubusercontent.com/28577229/38979754-edb7186e-436f-11e8-8a73-62a7f10d95b7.jpeg)

Check out the video of our robot here:

[Autonomous Path Finding Robot](https://vimeo.com/265468872)


#### The city is modeled as the following image:
![final](https://user-images.githubusercontent.com/28577229/38979575-667b7f8e-436f-11e8-865a-a90a8ee3d697.jpeg)


We gave the robot this graph and it solved it pretty quick. See if you can find the shortest path from start to end!

The following components are used:

1)Pixy camera

2)Arduino uno

3)Motor drivers

4)Motors

5)Rechargable Ion batteries

6)Jumper wires

### This robot was showcased at the UCLA engineering open house and attracted a significant amount of attention from the visitors.

## Description:
The robot uses a pixy camera to detect the color of the blocks on the road. We define different signature colors on the pixy to denote different points of the artificial city (eg. intersections are blue and roads are red). The pixy finds the x-coordinate and the angle of the road it is moving on. To make sure we stay on the desired path, we use the feedback reading from the camera and make adjustments so the robot stays on track. The motion of the robot is controlled by an Arduino Uno which runs the motor drivers; the microcontroller provides commands such as go left or right, and also stop on detecting an obstacle. 



## Code:
The algorithm used to find the shortest path in the city is the infamous Dijkstra's shortest path algorithm which enabled us to write efficient and scalable code that can provide the directions needed for the robot to get to the destination as fast as possible. After the Dijksta code provides the direction, the Arduino starts the motors and traces through the city by stepping through the array of directions and stopping when the destination is reached. Note that the graph (ie. the city) is represented as a 2D array.
