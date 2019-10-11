# Lab 3 - Rapidly-exploring Random Tree (RRT)
Lab 3 for [COMSW4733 Computational Aspects of Robotics](https://www.cs.columbia.edu/~allen/F19/) at Columbia University (Instructor: [Prof. Peter Allen](http://www.cs.columbia.edu/~allen/)).

## Introduction
In this lab, you are required to implement the Rapidly-exploring Random Tree (RRT) algorithm for motion planning on a 3DOF robotic arm, using the [pybullet](https://pybullet.org/wordpress/) simulator.

## Requirements
This repo is tested with python 2.7 and pybullet 2.5.6.

## Usage
To load the pybullet simulated environment for this lab as shown below, simply run

```
python demo.py
```

In the environment, there is a simplified 3DOF [UR5](https://www.universal-robots.com/products/ur5-robot/?gclid=EAIaIQobChMIu9ny1NOU5QIVhJ6fCh0DKAIMEAAYASAAEgJWuvD_BwE) robotic arm. The goal configuration is visualized using a red sphere marker. There are two semi-transparent black blocks and a plane as obstacles.

<p align="center">
  <img src="environment.png", height="350">
</p>


## Details and Rubric

You should get yourself familiar with the `demo.py` file which contains simple example code using pybullet and an overall structure of the expected submission. Visit [here](https://pythonhosted.org/pybullet/) for a detailed documentation of pybullet.

In this lab, you do not need to call [`stepSimulation`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.czaspku18mzs) or [`setRealTimeSimulation`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.ohnirlot3njq) for real phisics simulation. The arm is controlled by [`resetJointState`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.p3s2oveabizm) and collision can be checked by first resetting the arm to the desired configuration and then use [`getClosestPoints`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.cb0co8y2vuvc). The collision types you should consider includes

- self-collision of the robot arm
- collision between the arm and the obstacles (including the plane and two blocks). Note that the arm base is fixed at 0.02m above the plane so initially there is no collision between the arm and the plane.

You should also make sure you do not set the arm to a configuration that violates the joint limits. Joint limits (with other joint information) can be obtained through [`getJointInfo`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.la294ocbo43o). Current joint configuration and link pose can be obtained through [`getJointState`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.p3s2oveabizm) and [`getLinkState`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.3v8gjd1epcrt).

### Part 1 - RRT (50%)
In this part, you should implement the RRT algorithm to plan a collision-free motion to reach the target configuration (specified in the `demo.py`). See a video demo for this part [here](https://youtu.be/o-RCIhsLmqw). 

- Build the tree and visualization (30%). 
- Find and visualize the solution path (10%).
- Control the robot to move to the target configuration following the found path (10%). In your video you should rotate the camera in pybullet to show that no collision happens.

### Part 2 - Bidirectional RRT (50%)
In this part, you should implement the bidirectional RRT algorithm to plan a collision-free motion to reach the target configuration (specified in the `demo.py`). See a video demo for this part [here](https://youtu.be/4nFmFcLg5RQ).

- Build the tree and visualization (30%). You should use different colors for different trees.
- Find and visualize the solution path (10%).
- Control the robot to move to the target configuration following the found path (10%). In your video you should rotate the camera in pybullet to show that no collision happens.

## Submission Instructions
TODO