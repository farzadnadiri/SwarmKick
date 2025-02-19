# SwarmKick: Swarm Intelligence for Humanoid Soccer Robots

## Overview

SwarmKick is a swarm intelligence framework designed to enhance the coordination, decision-making, and adaptability of humanoid soccer robots. By leveraging decentralized control, emergent behaviors, and biologically inspired techniques, SwarmKick enables robust teamwork in dynamic, uncertain, and partially observable environments.

## Features

- **Decentralized Control**: Eliminates single points of failure, ensuring resilience and adaptability.
- **Emergent Coordination**: Robots collaborate dynamically to optimize formation and roles.
- **Role Allocation Algorithms**: Assigns roles in real-time based on situational analysis.
- **Formation Control**: Maintains team structure while adapting to gameplay conditions.
- **Adaptive Strategies**: Responds to unexpected scenarios such as failures or opponent tactics.
- **Scalability**: Efficiently handles different team sizes and game complexities.

## Installation

Clone the repository:

`bash
git clone https://github.com/farzadnadiri/SwarmKick.git
cd SwarmKick`


## Install dependencies:

`pip install -r requirements.txt`

## Usage

Running the 2D Simulation (Pygame)

`python simulation_2d.py`

Running the 3D Simulation (Webots)

`webots simulation_3d.wbt`


Simulation Environment

SwarmKick supports two simulation environments:

- 2D Simulation: Built using Pygame, offering a lightweight, visual representation of swarm coordination.

- 3D Simulation: Runs in Webots, providing a realistic robotic simulation environment for testing advanced behaviors.

## Results & Performance

Our simulations demonstrate:

- Improved team cohesion and synchronization.
- Enhanced robustness against robot failures.
- Superior adaptability compared to centralized strategies.
