# 3D-Simulation-of-Standard-Manipulators
- The respository contains codes to generate dynamic equations for any general serial-manipulator robot given its DH parameters and link masses.
- The dynamic equation calculator is implemented to calculate dynamic equations of SCARA, PUMA and Stanford-type Manipulators and generate their 3-D simulations.  
- It also contains codes to calculate the forward/inverse kinematics of the three manipulators and implementation of 4 different types of controllers to controller these 3 robots.
- **Requirements:** numpy, sympy, matplotlib

## Interactive mode and Scripted mode
### 1. Interactive Mode
- Use to get side-by-side controllable simulation while passing commands in the command prompt.
- Starting Interactive Mode:
    1. Open the cloned folder using command prompt.
    2. Start python interactive mode using `py`/`python`/`python3`
    3. Run the following lines:
        ``` 
        from load import*
        arko, arkoCon = load(SCARAManipulator, Multivariable_Position_Controller)
        ```
        This loads SCARA Manipulator and controls it using Multivariable_Position_Controller. This will open a window with live animation for the SCARA Manipulator.
    4. Try controlling the animation using these commands:
        ```
        arko.set_ee_position((0.4,0.06,-0.3))
        arkoCon.Achieve_EE_Position((0.4,0.01,-0.4))
        ```

### 2. Scripted Mode
- Use to generate pre-coded response from the simulations, passing command through a python script.
    1. Import the load.py and use load function as done in interactive mode.
    2. Use commands in the script files in a similar manner as used in interactive mode.
    3. Run the python script file.
    4. Refer to *test.py* for reference.

## Folder Structure
```
.
│   README.md
│   load.py     # To load everything using one file
|   test.py     # A test file to ensure working of simulations and controllers
│
└───Controllers
│   │   __init__.py
│   │   ComputedTorqueFFController.py   # Computed torques feedforward controller
|   |   FeedForwardController.py        # Normal feedforward controller
|   |   MultivariableController.py      # Multivariable controller
|   |   PIDController.py                # PID Position Controller
│   
└───Robots
|   │   __init__.py
|   │   PUMA.py                         # PUMA Manipulator (RRR) 
|   |   SCARA.py                        # SCARA Manipulator (RRP)
|   |   Stanford.py                     # Stanford Manipulator (RRP)
|   |   Tools.py                        # Tools required to generate dynamic equations
|      
```

## Documentation
### Robots
 - Each file contains robot classes with their respective functions.
 - Useful Methods:
   1. **set_state**: To set the state of the robot.
   2. **get_state**: To retrieve the state of the robot.
   3. **inv_kin**: To get joint angles from the end effector position.
   4. **set_ee_position**: Set the end effector position.
   5. **get_ee_position**: Get the end effector position.
   6. **apply_constant_torques**: Apply the defined constant torques for 2 sec of simulation time.
   7. **reset**: Reset the manipulator position and time.
   8. **LiesInWorkspace**: Checks if the given point lies in the workspace of the robot.
   9. **run**: Start the animation.
   10. **stop**: Stop the animation. 
### Controllers
 - Each file contains robot controllers with their respective functions.
 - Useful Methods:
   1. **set_K**: Set the gains for the controllers.
   2. **set_target_state**: Set the target states.
   3. **achieve_target_state**: Achieve the target state.
   4. **Achieve_EE_Position**: Achieve the given end effector position.
   5. **follow_trajectory**: Follow a defined trajectory of the robot.
### Tools.py
- Contains the functions used to generate dynamic equations for any general 3D Manipulator.
- List of functions:
  1. **DHMatrix2Homo_and_Jacob** 
     - Calculates Homogeneous Transform Matrix and Jacobian from the DH parameter matrix.
     - Parameters:
       1. Hmat: 
           - DH parameter matrix (nx4); n --> Number of joints
           - Number of links automatically estimated from the size of matrix
           - Each row is of form: theta (rotation about z), d (translation about z), a(translation about x), alpha(rotation about x)
           - First columns of params are frame 0-->1 transformation and last columns are frame (n-1)-->n transformation; n is the end effector frame
       2. prismatic:
           - An array of the joints that are prismatic; Joint corresponding to angle *q<sub>i</sub>* is i<sup>th</sup> joint
     - Returns: A tuple of:
       1. H: Homogenoeous Transformation Matrix of n w.r.t. 0.
       2. J: Jacobian of n<sup>th</sup> frame for the given joints.

  2. **DV_Calculator**
     - Calculates the Inertia Matrix (D) and Potential Scalar Function (V) from DHMI Matrix.
     - Parameters:
       1. DHMI Matrix:
          - Dhmi: Dhmi (DH + Mass + Inetria Tensor) array
          ```
          [[theta1,      d1,     l1,     alpha1,    m1,     I1],
           [theta2,      d2,     l2,     alpha2,    m2,     I2],
           [theta3,      d3,     l3,     alpha3,    m3,     I3]]
          ```
          - `Ii` is 3x3 inertia tensor of i<sup>th</sup> link w.r.t. the frame attached to i<sup>th</sup> link.
       2. g_dir:
          - Direction of gravity
          - Can be 'x', 'y', 'z', '-x', '-y', '-z'
       3. prismatic:
          - An array of the joints that are prismatic: Joint corresponding to *q<sub>i</sub>* is i<sup>th</sup> joint
     - Returns:
       - A tuple of:
         1. D: Inertia Matrix of the robot.
         2. V: Potential Scalar Function of the robot. Considers gravity only.
     - IMPORTANT NOTES & ASSUMPTIONS:
       - Assumed the centre of mass of link_i to lie at the midpoint of frame_(i-1) and frame_i.
       - DHMI is a matrix with manipulator lengths, link masses passed as constants and joint variables passed as symbolic constants.
       - g_dir is used to calculate the V, specifies the direction of g; Default is taken as -z
       - Can also pass whole DHMI Matrix in terms of symbolic constants.
  3. **CG_from_DV** 
     - Calculates the Christoffel Matrix (C) and Potential Feild Matrix (g_)
     - Parameters:
       1. D: Inertia Matrix
       2. V: Scalar Potential Function
     - Returns:
       -  A tuple of:
          1. C: Chritoffel Constants Matrix
          2. g_: Potential Feild Matrix
  4. **Dynamic_Equations_from_Dhmi**
     - Prints the dynamic equations from DHMI Matrix.
     - Parameters:
       1. DHMI Matrix
       2. g_dir
       3. prismatic
     - Returns:
       1. D: Inertia Matrix
       2. C: Christoffel Matrix
       3. g_: Potential Feild Matrix
  5. **q_dot2_array**
     - Gives an array of accelerations of joint angles, expressed in terms of joint angles and joint velocities and applied torques.
     - Parameters:
       1. DHMI Matrix
       2. tau: A symbolic matrix containing symbolic constants for torques/forces at the joints.
       3. q_dot: A symbolic matrix containing symbolic constants for angular/linear velocities of the joints.
       4. prismatic
     - Returns:
       - A matrix containing joint accelerations in symbolic form.

