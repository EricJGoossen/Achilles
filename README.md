# Rigid Body Simulator

## Overview
This project is a general-purpose rigid body dynamics simulator designed to model and simulate articulated systems. It provides a framework for representing bodies, joints, and their interactions, enabling the simulation of motion over time.

The simulator focuses on modularity and extensibility, allowing different types of joints, kinematic structures, and physical properties to be composed into complex systems such as robots, mechanisms, or multi-body assemblies.

## Features
- Representation of rigid bodies and their inertial properties  
- Support for articulated structures using joint trees  
- Modular joint implementations with configurable degrees of freedom  
- Separation of geometry, kinematics, and dynamics for flexibility  

## Architecture
At a high level, the simulator is built around a few core concepts:

- **Links**: Represent rigid bodies with mass and inertia  
- **Joints**: Define constraints and motion between links  
- **Joint Tree**: Organizes the system as a hierarchical structure  
- **Transform Tree**: Maintains relationships between coordinate frames  

The system recursively propagates physical quantities through the joint tree to compute composite properties and system behavior.

## Simulation Loop
A typical simulation step consists of:
1. Computing composite inertia
2. Computing system input from actuators
3. Propagating accelerations through the system  
4. Integrating joint states over a timestep  
5. Updating cached positions, velocities, and accelerations  

## Design Goals
- **Modularity**: Components are designed to be interchangeable and extensible  
- **Performance**: Efficient algorithms for multi-body dynamics  
- **Clarity**: Clear separation between physical concepts  
- **Scalability**: Suitable for simple systems and complex articulated structures  

## Getting Started
1. Define links and their inertial properties  
2. Connect links using joints  
3. Build a joint tree representing the system  
4. Step the simulation forward using a timestep  

## Future Work
- Collision detection and response  
- Constraint solvers  
- Visualization tools  
