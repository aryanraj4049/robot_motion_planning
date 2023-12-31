# robot_motion_planning
This project was done in Spring semester(IIT Knapur) of 2022-23 under the guidance of Professor and Head of Department Mechanical Engineering IIT Kanpur, Dr Ashish Dutta.
This project provides a comprehensive introduction to path planning and obstacle avoidance for mobile robots and serial arms. It targeted foundational concepts such as configuration space and its relation to robot path planning.It covers a range of planning techniques including road maps, cell decomposition, sampling-based, and potential field methods, alongside addressing non-holonomic and underactuated systems. By the end of the project, students will have the ability to formulate algorithms for automated path generation and obstacle avoidance, applicable to practical robotic challenges. The project uniquely integrates programming skills, enabling students to develop their path planning algorithms like A*, D* and Dijkstra's.

This project was divided among three assigment:-

1) C- space for 2DOF planer serial robot in which we have to write a program to generate the C Space for a 2 DOF (2R) planer robot arm. There should be at least 3 obstacles of different shapes, sizes and colour (e.g. square, triangle, circle,..). Plot the cartesian  space (x,y) showing the links, obstacles. Choose suitable lengths of the arms. The links may be straight lines or can occupy space (rectangles). Plot the C-space (theta1 verses theta2 ), showing the free space as white and obstacle space having the respective obstacle colour. Submit the full program and results showing points 3 and 4.
![image](https://github.com/aryanraj4049/robot_motion_planning/assets/87406447/61ecc5b4-8a33-4ae9-9b51-97f9c02992dc)![image](https://github.com/aryanraj4049/robot_motion_planning/assets/87406447/48d9d0b9-b081-4a7b-b734-ec909b9d5bb4)

2) Rapidly Expanding Random Tree :-Objective of the assignment is to write a program to find a path from the start point to the goal point using RRT. Generate a rectangular workspace with three or more obstacles. Mark the start point and the goal point in the workspace. Generate feasible points. Generate a set of random points from the start point and connect it to a tree forming branches. From the start point the next feasible points (one or more) may be made at a random direction with a fixed step length. Generate the roadmap in the form of a tree with branches. Find a path from the start point to the goal point. Submit the full program and results showing points 1-3. Results should show several figures (4-5) that show the work space with obstacles with start and goal point. Road map generated . Final path from initial point to goal point. One result should show failed cases , with suitable object position or shapes.
![image](https://github.com/aryanraj4049/robot_motion_planning/assets/87406447/2cb75065-8a4d-4311-bfcf-33f66c0223fd)
![image](https://github.com/aryanraj4049/robot_motion_planning/assets/87406447/a6722c8b-a962-4c5c-a36e-e447cb48a43d)

3) Potential field method to find a path from a start point to a goal point in a 2D workspace with obstacles:- Generate a rectangular workspace with three or more obstacles or a room with walls and passage ways (doors). Mark the start point and the goal point in the workspace. Define the potential field functions for attractive and repulsive fields. Generate feasible paths for different values of the constants in the attractive and repulsive fields. Which is the best path.  
Submit the full program and results showing points 1-4. Results should show several figures (4-5) that show the work space with obstacles with start and goal point. Final paths from initial point to goal points. One result should show failed cases , with suitable object position or shapes. Explain why it is failing.
![image](https://github.com/aryanraj4049/robot_motion_planning/assets/87406447/7bc6ed44-1cc9-438f-aba7-3b29465d7ee6)
![image](https://github.com/aryanraj4049/robot_motion_planning/assets/87406447/2c768747-c996-4206-bc82-35a20331359e)
![image](https://github.com/aryanraj4049/robot_motion_planning/assets/87406447/0141af09-5c1f-4363-9c0f-9e09792abdbe)
![image](https://github.com/aryanraj4049/robot_motion_planning/assets/87406447/b5caa701-7fbe-45c9-8e51-92f48514e9f3)






