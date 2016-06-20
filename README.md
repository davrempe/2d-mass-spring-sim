# 2D Mass-Spring Interactive Simulation
This is an interactive two-dimensional mass-spring system simulator written using OpenGL and GLUT. The core of the simulation is implemented in 'SpringMassSim\SpringMassSim\SpringMassSim.cpp'

![alt text](https://github.com/davrempe/2d-mass-spring-sim/blob/master/images/screenshot.png "Example of simulation running")

# To Run:
* The easiest way is to simply open the Visual Studio solution and run. This uses the nupengl package installed through VS.  
* If you don't have Visual Studio, make sure to have GLUT and GLEW properly set up.

# Controls:
Within the simulation you can do a number of things. You start with a blank canvas, you can:
* _LEFT-CLICK_ and hold to create a new dynamic mass. The longer you hold, the larger the mass. These can be linked via springs to any other mass and their motion will be simulated while the simulation runs.
* _RIGHT-CLICK_ and hold to create a fixed mass. Again, the mass depends on how long the mouse button is held. These can be attached to any other masses via a spring, but will they are fixed and will not move during the simulation.
* _LEFT-CLICK_ and drag between mass to insert a spring between them. The parameters of the spring are determined by the constants in the code: `DEFAULT_KS`, `DEFAULT_KD`, and `DEFAULT_R`. These are the spring constant, damping factor, and rest length.

To start the simulation, press <kbd>-></kbd>. The simulation can be paused at any time with <kbd><-</kbd>. 

While the simulation is running you can _LEFT-CLICK_ and drag on any dynamic mass to interact with the system.

# Details:
The simulation uses Euler's Method for numerical integration. The collision detection implementation is fairly simple and assumes elastic collisions. It waits until the boundaries of two masses touch and handles the collision at this time, resulting in some wonky behavior if too many particles collide at once. 

This project was inspired by [these course notes](https://graphics.stanford.edu/courses/cs448b-00-winter/papers/phys_model.pdf) on Physically Based Modeling by David Baraff and Andrew Witkin from SIGGRAPH 1999. 
