# QuIK: An ultra-fast and highly robust kinematics library for C++ and ROS2 using DH parameters

QuIK is a hyper-efficient C++ kinematics library for serial manipulators. It is based on the novel QuIK algorithm, [published in IEEE-TRO](http://dx.doi.org/10.1109/TRO.2022.3162954), that uses 3rd-order velocity kinematics to solve generalized inverse kinematics significantly faster, and significantly more reliably that existing inverse kinematics packages. QuIK uses the Denevit-Hartenberg convention for kinematics, which is readily available for most manipulators and results in a more computationally efficient formulation of kinematics.

Some key benchmarks over other available solvers:

| Solver         | Mean Solution Time | Error Rate |
| -------------- | ------------------ | ---------- |
| QuIK           | 21 μs              | 0.13%      |
| KDL (used in ROS, and primary base solver in TracIK)            | 148 μs (x7)       | 5.3% (x40)|
| Matlab Robotics Toolbox        | 670 μs (x32)      | 1.1% (x9) |

These benchmarks were published in the IEEE-TRO paper, and further details about them can be found there. A preprint of this paper is included in this repository:  [SLloydEtAl2022_QuIK_preprint.pdf](SLloydEtAl2022_QuIK_preprint.pdf).

> S. Lloyd, R. Irani, and M. Ahmadi, "Fast and Robust Inverse Kinematics for Serial Robots using Halley’s Method," IEEE Transactions on Robotics, vol. 38, no. 5, pp. 2768–2780, Oct. 2022. doi: [10.1109/TRO.2022.3162954](http://dx.doi.org/10.1109/TRO.2022.3162954). A preprint of this paper can be found in the current repository, [SLloydEtAl2022_QuIK_preprint.pdf](SLloydEtAl2022_QuIK_preprint.pdf).

This repository includes the code to use the QuIK algorithm in ROS2, or just in C++ in general. Examples are given in python as well.

## What this repository does and does not do

This repository allows for highly efficient robot kinematics, and in particular inverse kinematics. It is designed for serial manipulators, i.e. manipulators with a single kinematic chain that does not branch. 

 - Highly efficient and robust inverse kinematics against 6-DOF constraints in world frame. E.g. a target tool point and rotation is specified, and joint angles are returned.
 - Forward kinematics
 - Velocity kinematics/Jacobian computation

It does not do:
 - [_Planned_] Null space optimization for robots with more than 6 joints. The code works perfectly for higher-order chains, but will only return "a" solution, not necessarily a solution which is optimal. 
 - [_Planned_] Reduced-order or transformed constraints, where you perhaps care about the tool point, but not rotation, or other combinations thereof.
 - [_Planned_] Feasibility checks. This code base does not check the computed solutions against joint limits, or check whether the target pose can be reached within the robot's speed and acceleration limits.

## How to build

For optimal performance, build using the release flag. For this type of iterative code, the compiler optimizations make it run about 10x faster. It will work just fine without it, however.
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Code Description and Usage

The main C++ code is stored in `include/quik` and `src`. The code can be used directly in your C++ projects, or in C++ ros nodes, or by building and running the provided kinematics service node (see below).

**Core QuIK functionality**:
 - `include/quik/Robot.hpp`: Defines the `quik::Robot` class and forward kinematic/jacobian functions.
 - `include/quik/IKSolver.hpp`: Defines the `quik::IKSolver` class and associated inverse kinematics functions.
 - `include/quik/geometry.hpp` and `src/geometry.cpp`: Defines the `quik::geometry` namespace, which includes a number of functions for transforming from twists to homogeneous transforms to quaternion/point representations, etc.
 - `include/quik/utilities.hpp`: Defines the `quik::utilities` namespace, which includes some non-kinematic helper functions.

**ROS2 code**
 - `include/quik/ros_helpers.hpp` and `src/ros_helpers.cpp`: Defines helper functions for using QuIK in ROS, such as service handlers for forward/inverse/velocity kinematics, as well as helper functions for forming and parsing service requests.
 - `src/ros_kinematics_service_node.cpp`: A ROS2 node that defines services for forward, inverse, and velocity kinematics.

**Example code**
 - `src/sample_cpp_usage.cpp`: A simple demo showing how the codebase can be used (ROS-indenpendent).
 - `src/sample_ros_client_node.cpp`: A sample client node for the service server node provided.
 - `src/sample_ros_cpp_node.cpp`: A simple demo of using the QuIK codebase in a ROS node (directly, without using the service node).

**Python code**
 - `python/sample_quik_client_node.py`: A sample python node that calls the kinematics service node provided.
 - `python/quik_service_helpers.py`: A python module that defines helper functions for forming and parsing service requests from the service node.

### Basic Usage

This library does not use URDF like many ROS packages; instead, it builds the robot structure using Denevit-Hartenburg (DH) convention. The DH table is a method to describe the geometry of a robot or a kinematic chain. It uses four parameters - `a`, `alpha`, `d`, and `theta` - to define the spatial relationship between adjacent links in the chain. More information on the DH convention [here](https://users.cs.duke.edu/~brd/Teaching/Bio/asmb/current/Papers/chap3-forward-kinematics.pdf) or [here](https://spart.readthedocs.io/en/latest/DH.html).

#### The ``quik::Robot`` class

In this implementation, we use the Spong notation and numbering convention for the DH table. This means that:

- `a` and `alpha` are the link length and twist angle, respectively, which describe the transformation from one link to the next.
- `d` and `theta` are the link offset and joint angle, respectively, which describe the transformation due to the joint variable.

The DH parameters are arranged in a matrix where each row corresponds to one joint of the robot, and the columns correspond to the `a`, `alpha`, `d`, and `theta` parameters, in that order.

The robot properties are defined as follows:

- `DH`: A DOFx4 matrix of the Denavit-Hartenberg (DH) parameters in the following order:

    ```plaintext
    a_1  alpha_1    d_1   theta_1
    :       :        :       :
    an   alpha_n    d_n   theta_n
    ```
- `linkTypes`: A vector of link types. Each element should be `true` if the corresponding joint is a prismatic joint, `false` otherwise.
- `Qsign`: A vector of link directions. Each element should be `-1` or `1`, allowing you to change the sign of the corresponding joint variable.
- `Tbase`: The base transform of the robot. This is a 4x4 matrix representing the transformation between the world frame and the first DH frame.
- `Ttool`: The tool transform of the robot. This is a 4x4 matrix representing the transformation between the DOF'th frame and the tool frame.

For example, the following code would define the KUKA KR6 manipulator:
```c++
// Given as DOFx4 table, in the following order: a_i, alpha_i, d_i, theta_i.
Matrix<double, 6, 4> DH;
DH << 0.025,    -M_PI/2,   0.183,       0,
      -0.315,   0,         0,           0,
      -0.035,   M_PI/2,    0,           0,
      0,        -M_PI/2,   0.365,       0,
      0,        M_PI/2,    0,           0,
      0,        0,         0.08,        0;

// Second argument is a list of joint types
Vector<quik::JOINTTYPE_t,6> linkTypes;
linkTypes << quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE;

// Third argument is a list of joint directions
// Allows you to change the sign (direction) of the joints.
Vector<double,6> Qsign;
Qsign << 1, 1, 1, 1, 1, 1;

// Fourth and fifth arguments are the base and tool transforms, respectively
Matrix4d Tbase = Matrix4d::Identity(4,4);
Matrix4d Ttool = Matrix4d::Identity(4,4);

auto R = std::make_shared<quik::Robot<6>>(DH, linkTypes, Sign, Tbase, Ttool);
```

#### The ``quik::IKSolver`` class

To perform inverse kinematics, you need to build an `IKSolver` object that takes a few extra parameters, although for basic usage the default parameters are generally good:
```c++
// Define the IK options
const quik::IKSolver<6> IKS(
    R, // The robot object (pointer)
    200, // max number of iterations
    quik::ALGORITHM_QUIK, // algorithm (ALGORITHM_QUIK, ALGORITHM_NR or ALGORITHM_BFGS)
    1e-12, // Exit tolerance
    1e-14, // Minimum step tolerance
    0.05, // iteration-to-iteration improvement tolerance (0.05 = 5% relative improvement)
    10, // max consequitive gradient fails
    80, // Max gradient fails
    1e-10, // lambda2 (lambda^2, the damping parameter for DQuIK and DNR)
    0.34, // Max linear error step
    1 // Max angular error step
);
```
For the most part, these defaults above are fairly good start, so you can initialize this object just with:
```c++
const quik::IKSolver<6> IKS(R);
```

The full list of `IKSolver` parameters are:
- `max_iterations` [`int`]: Maximum number of iterations of the
  algorithm. Default: 200
- `algorithm` [`ALGORITHM_t`]: The algorithm to use. 
  Default: `ALGORITHM_QUIK`.
    - `ALGORITHM_QUIK` - QuIK
    - `ALGORITHM_NR` - Newton-Raphson or Levenberg-Marquardt
    - `ALGORITHM_BFGS` - BFGS
- `exit_tolerance` [`double`]: The exit tolerance on the norm of the error. Default: `1e-12`.
- `minimum_step_size` [`double`]: The minimum joint angle step size (normed) before the solver exits. Default: `1e-14`.
- `relative_improvement_tolerance` [double]: The minimum relative iteration-to-iteration improvement. If this threshold isn't met, a counter is incremented. If the threshold isn't met
  `max_consecutive_grad_fails` times in a row, then the algorithm exits. For example, 0.05 represents a minimum of 5% relative improvement. Default: `0.05`.
- `max_consecutive_grad_fails` [int]: The maximum number of relative improvement fails before the algorithm exits. Default: `20`.
- `lambda_squared` [double]: The square of the damping factor, `lambda`.  Only applies to the NR and QuIK methods. If given, these methods become the DNR (also known as Levenberg-Marquardt) or the DQuIK algorithm. Ignored for BFGS algorithm. Recommended to set to a small, positive but nonzero value. Default: `1e-10`.
- `max_linear_step_size` [double]: An upper limit of the error step
  in a single step. Ignored for BFGS algorithm. Default: Uses the `Robot.characteristicLength()` function to automatically estimate an appropriate value from the values in the `DH` table as `0.33*Robot.characteristicLength()`.
- `max_angular_step_size` [double]: An upper limit of the error step in a single step. Ignored for BFGS algorithm. Default: `1`.
- `armijo_sigma` [double]: The sigma value used in Armijo's rule, for line search in the BFGS method. Default: `1e-5`
- `armijo_beta` [double]: The beta value used in Armijo's rule, for line search in the BFGS method. Default: `0.5`

### Templating for fixed size
Both ``quik::Robot`` and ``quik::IKSolver`` are templated with the integer value ``DOF``, the number of degrees of freedom of the robot. The code is designed such that ``DOF`` can be set to any positive integer value, or to ``Eigen::Dynamic`` (or ``-1``) for a variable-length object. If you know the ``DOF`` beforehand, setting it properly rather than using ``-1`` will avoid dynamic memory allocation in all computations, making the code run in a more reliable and constant amount of time for real-time purposes.

## Usage in ROS

To do.

### Kinematics Service Node

The file `src/ros_kinematics_service_node.cpp` defines a service node that will load a robot structure from a parameter yaml file (use one of the provided yaml files in `config/`, or just make your own).

Running the CPP node:
```bash
ros2 run quik sample_ros_cpp_node --ros-args --params-file ./src/quik/config/ik_service_kuka_kr6.yaml 
```

Running the service:
```bash
ros2 run quik ros_kinematics_service_node --ros-args --params-file ./src/quik/config/ik_service_kuka_kr6.yaml
```

Running the sample client:
```bash
ros2 run quik sample_ros_client_node --ros-args --params-file ./src/quik/config/ik_service_kuka_kr6.yaml
```

### YAML Config Files

To do, description of parameters and sample ones

### Python Clients

To do.

## Requirements
All functions rely on the [Eigen 3.4](https://eigen.tuxfamily.org) linear algebra library, so you will also need to link to an appropriate library.

## Citations

If you use our work, please reference our publication below. Recommended citation:

[1] [S. Lloyd, R. A. Irani, and M. Ahmadi, “Fast and Robust Inverse Kinematics of Serial Robots Using Halley’s Method,” IEEE Transactions on Robotics, vol. 38, no. 5, pp. 2768–2780, Oct. 2022.](SLloydEtAl2022_QuIK_preprint.pdf) doi: [10.1109/TRO.2022.3162954](http://dx.doi.org/10.1109/TRO.2022.3162954).

## Commercial Licensing

To do. Can do it, but it costs money. Contact me.