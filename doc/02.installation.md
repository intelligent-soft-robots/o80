# Installation

o80 is a c++ library with python wrappers. It uses catkin for compilation.
It has been tested only on ubuntu 18.04 and python3.
To use o80, you will need:
- a machine installed with ubuntu 18.04, possibly with a preempt patch kernel, if your robot requires it.
- installing a few dependencies (apt and pip installation only)
- creating a catkin workspace with o80 + dependencies packages (we use the project manager [Treep](https://pypi.org/project/treep/) to make this trivial)
- creating a catkin project that will host your c++ code. Your c++ code will be the classes used for State, Observation, as well as the Driver interfacing
  o80 and your robot. o80 has been designed to that this code may be minimalist.
- create the python binders (o80 provides some high level functions to makes this trivial)

o80 is mostly a templated header library. It can thus only be installed in relation with a package which provides concrete class to be used as template. Thus the detailed installation procedure (that end up with demos you will be able to run) is available along with a package providing such concrete classe:  [o80 example](https://github.com/intelligent-soft-robots/o80_example)

- [o80 roboball2d](https://github.com/intelligent-soft-robots/o80_roboball2d)
- [o80 PAM](https://github.com/intelligent-soft-robots/o80_pam)
