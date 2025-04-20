
# Development of Multi-Agent Control for Surface and Underwater Vehicles
The effective deployment of autonomous amphibious vehicles for surface and underwater operations re-
quires advancements in hardware, control systems, and decision-making algorithms to operate efficiently
across dynamic environments. Current systems often lack seamless state estimation, multi-agent coor-
dination, and adaptive control strategies for real-time obstacle avoidance in diverse operational settings.
To achieve robust and adaptable performance, there is a need for a unified framework that integrates
priority-based task assignment, state-aware control switching, and flexible leader-follower dynamics for
collaborative operations among surface and underwater agents. The effective deployment of autonomous
amphibious vehicles for surface and underwater operations requires advancements in hardware, control
systems, and decision-making algorithms to operate efficiently across dynamic environments. Current
systems often lack seamless state estimation, multi-agent coordination, and adaptive control strategies
for real-time obstacle avoidance in diverse operational settings. To achieve robust and adaptable perfor-
mance, there is a need for a unified framework that integrates priority-based task assignment, state-aware
control switching, and flexible leader-follower dynamics for collaborative operations among surface and
underwater agents.

# Objectives

- **Modeling of AUV, ASV**: 
    - Creating accurate models to represent both the autonomous underwater vehicle (AUV) and the autonomous surface vehicle (ASV) dynamics is crucial for simulating real-world behavior, enabling effective depth and orientation control.

- **Control with Obstacle Avoidance**: 
    - Developing control systems to maintain stability, avoid obstacles, and transition between modes ensures safe and efficient operation.

- **Multi-Agent Architecture for AUVs**:
    - **Task Switching and Priority-Based Allocation**: Enable each AUV to take on roles based on its capabilities, allowing for dynamic task switching and reallocation based on mission priorities. This will maximize resource utilization and ensure that high-priority tasks receive attention from the most capable agents.
    - **Leader-Follower Control Strategy**: Implementing a leader-follower setup will create efficient coordination in complex environments, ensuring a lead AUV can guide the group while followers maintain formation. This approach enhances mission cohesion and simplifies multi-agent movement through challenging areas.
    - **Inter-Agent Communication**: A reliable communication system among AUVs is essential for sharing real-time data on position, task status, and environmental changes. This will support adaptive decision-making, allowing agents to respond collectively to evolving mission conditions.
    - **Obstacle Avoidance and Navigation**: Equipping AUVs with autonomous obstacle detection and avoidance ensures safe operation and efficient navigation. This capability will be critical for uninterrupted multi-agent operations in dynamic underwater environments.

- **Hardware Development and Deployment**: 
    - Building and integrating reliable hardware that can withstand dual-environment operations is essential, especially given the added challenge of balancing the combined vehicle-drone system for optimal aerial and aquatic performance.


Implementation Scheme
• Modeling: Begin by developing accurate mathematical models for both surface and underwater  dynamics of your vehicles.
• Simulation: Utilize simulation environments (like MATLAB/Simulink, Gazebo, or ROS) to test control strategies in virtual scenarios before deploying on actual vehicles.
• Hardware Integration: Once simulations yield satisfactory results, implement the control strategies
on the physical vehicles, ensuring to integrate with sensors and actuators for real-time feedback.
• Testing and Validation: Conduct extensive testing in controlled environments, gradually introducing
complexity (e.g., obstacles, varying terrains) to validate the robustness of the control strategies.


### Video
[![Watch the video](https://img.youtube.com/vi/sVQAPMuBPPs/maxresdefault.jpg)](https://www.youtube.com/watch?v=sVQAPMuBPPs&list=PLeGlw_YNKerEtMuekMxKAprKBLYVwNjb-&index=4)
