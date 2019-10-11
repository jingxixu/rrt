# Lab 3 - Rapidly-exploring Random Tree (RRT)
Lab 3 for [COMSW4733 Computational Aspects of Robotics](https://www.cs.columbia.edu/~allen/F19/) at Columbia University (Instructor: [Prof. Peter Allen](http://www.cs.columbia.edu/~allen/)).

## Introduction
In this lab, you are required to implement the Rapidly-exploring Random Tree (RRT) algorithm for motion planning on a 3DOF robotic arm, using the [pybullet](https://pybullet.org/wordpress/) simulator.

## Requirements
This repo is tested with python 2.7 and pybullet 2.5.6.

## Usage
To load the pybullet simulated environment as shown below, simply run

```
python demo.py
```

In the environment, there is a simplified 3DOF [UR5](https://www.universal-robots.com/products/ur5-robot/?gclid=EAIaIQobChMIu9ny1NOU5QIVhJ6fCh0DKAIMEAAYASAAEgJWuvD_BwE) robotic arm. The goal configuration is visualized using a red sphere marker. There are two semi-transparent black blocks as obstacles.

You should get yourself familiar with the `demo.py` file which contains simple example code using pybullet and an overall structure of the expected submission. In this lab, you do not need to call [`stepSimulation`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.czaspku18mzs) or [`setRealTimeSimulation`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.ohnirlot3njq) for real phisics simulation. The arm is controlled by [`resetJointState`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.p3s2oveabizm) and collision can be checked by first resetting the arm to the desired configuration and then use [`getClosestPoints`](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.cb0co8y2vuvc).

<p align="center">
  <img src="environment.png", height="350">
</p>

## Part 1 - RRT (50%)
See a video demo for this part [here](https://youtu.be/o-RCIhsLmqw).

## Part 2 - Bidirectional RRT (50%)
See a video demo for this part [here](https://youtu.be/4nFmFcLg5RQ).

## Submission Instructions
TODO