---
title: 'The o80 C++ templated toolbox: Designing customized Python APIs for synchronizing realtime processes'
tags:
  - Python
  - C++
  - processes
  - robotics
  - shared memory
authors:
  - name: Vincent Berenz^[corresponding author]
    affiliation: 1 
  - name: Maximilien Naveau
    affiliation: 1
  - name: Felix Widmaier
    affiliation: 1
  - name: Manuel Wüthrich
    affiliation: 1
  - name: Jean-Claude Passy
    affiliation: 1
  - name: Simon Guist
    affiliation: 1
  - name: Dieter Büchler
    affiliation: 1
affiliations:
 - name: Max Planck Institute for Intelligent Systems, Tübingen, Germany
   index: 1
date: 05 July 2020
bibliography: paper.bib

# Optional fields if submitting to a AAS journal too, see this blog post:
# https://blog.joss.theoj.org/2018/12/a-new-collaboration-with-aas-publishing
#aas-doi: 10.3847/xxxxx <- update this with the DOI from AAS once you know it.
#aas-journal: Astrophysical Journal <- The name of the AAS journal.
---

# Statement of need

o80 (pronounced "oh-eighty") is a software for synchronizing and organizing message exchange between (realtime) processes via simple customized Python APIs. Its target domain is robotics and machine learning. Our motivation for developing o80 is to ease the setup of robotics experiments (i.e. integration of various hardware and software) by machine learning scientists. Such setup typically requires time and technical effort, especially when realtime processes are involved. Ideally, scientists should have access to a simple Python API that hides the lower level communication details and simply allows to send actions and receive observations. o80 is a tool box for creating such API.

o80 is in some aspects similar to ROS's actionlib [@actionlib], as both allow a process to create and monitor execution of commands executed by another process, with python and c++ interoperability. A particularity of o80 is its support for queues of command and the possibilities to request the server to automatically perform linear interpolation between them. o80 also introduces new synchronization methods (see "bursting mode" in the next section). Contrary to actionlib, o80 does not support network communication as it expects the processes it orchestrates to run on the same computer.

# Overview

For implementing synchronization, o80 organizes two types of processes:

- A server process encapsulates an instance of o80 back-end and an instance of a driver. At each iteration, the back-end instance computes for each actuator the desired state to be transmitted to the hardware via the driver. The back-end also reads sensory information from the driver. Typically, the server process is programmed in C++ with consideration for realtime.
- A client process encapsulates instances of o80 front-end, which provides: 1) an interface to send commands to the back-end (i.e. requests to compute desired states), 2) methods for querying sensory information, and 3) methods for synchronizing the client with the server process.

In the background, back-end and front-end(s) communicate by exchanging serialized object instances via a interprocess shared memory. Serialization is based on the cereal library [@cereal], and the shared memory is based on the Boost interprocess library [@boost].

o80 is templated over actuator states and driver; and may therefore support a wide range of systems.

o80's core API supports:

- methods for specifying via commands either full desired state trajectories or partial trajectories relying on interpolation.
- interpolation methods based on specified duration, speed, or number of server iterations.
- methods for either queuing or interrupting trajectories.
- frontend methods for setting commands that are either blocking or non blocking.
- frontend methods for retrieving sensory data that request the latest available information, sensor information corresponding to a specific past server iteration, or the full history of sensory information.
- client processes and the server processes that run asynchronously or synchronously.

Synchronization methods may be based on a fixed desired frequency set for the server, or may be set up by the client process ("bursting mode").

o80 library may be considered complex as it is versatile and can be used in the context of a wide range of control strategies. Yet, the objective of o80 is to provide robot users with a simple Python API. For this purpose, o80 provides convenience functions for generating application tailored Python bindings. The expected usage is that an expert developer uses o80 to design the Python API that will hide to end users the complexities related to interprocess communication and synchronization. Scientists may then use this simplified API to design new robotic experiments. In this sense, o80 aims to be a toolbox for creating customized Python APIs. Generation of Python bindings via the o80 API is based on pybind11 [@pybind11].

# Modular Implementation

o80 is based on open source packages that are maintained by the Max Planck Institute for Intelligent Systems, and which may be reused in other contexts. These packages are available on the GitHub domains "intelligent soft robots", "machines in motion", "mpi-is" and "open dynamic robot initiative". Examples of such packages are:

- synchronizer: a library for synchronizing processes
- shared memory: a wrapper over the boost interprocess library that makes exchange of serialized data over an interprocess shared memory trivial
- time series: a templated circular buffer with time stamps, supporting multiprocess access and synchronization

The complete list, the sources, the binaries as well as the documentation of theses packages can be found online [@corerobotics].


# Examples of usage

## Integration with SL

An instance of o80 backend has been integrated into the SL realtime library [@sl] used for the control of the Apollo manipulator [@apollo]. This allows scientists to program robot behavior using a simple Python interface running at a low non-realtime frequency that synchronizes with the realtime higher frequency C++ control loop of the robot. This interface was used for example for the experiments described in [@icsds].

## HYSR training

o80 has been used in the context of reinforcement learning applied to real robotic hardware. [@pam] describes a setup in which a robot arm driven by pneumatic artificial muscles learns autonomously to play table tennis using an hybrid sim and real training approach (HYSR), i.e, performing real hardware motions to interact with simulated balls. o80 was used in this context to:

- provide a Python API that has been integrated into a Gym environment to provide an interface to reinforcement algorithms [@gym]
- setting up the synchronization between the real robot control and the MuJoCo simulator [@mujoco] used for HYSR
- setting up asynchronous processes for live plotting of the robot state

The code and documentation of this project is available open source online [@pamsource].

# References


