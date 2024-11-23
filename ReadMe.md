# QuIK: A faster and more robust inverse kinematics library for ROS2

QuIK is a highly efficient C++ kinematics library for serial manipulators. It is based on the novel QuIK algorithm, [published in IEEE-TRO](http://dx.doi.org/10.1109/TRO.2022.3162954), that uses 3rd-order velocity kinematics to solve generalized inverse kinematics significantly faster, and significantly more reliably that existing inverse kinematics packages.

| Solver         | Mean Solution Time | Error Rate |
| -------------- | ------------------ | ---------- |
| QuIK           | 21 μs              | 0.13%      |
| KDL            | 148 μs (x 7)       | 5.3% (x 40)|
| Matlab         | 670 μs (x 32)      | 1.1% (x 9) |

These benchmarks were published in the IEEE-TRO paper, and further details about them can be found there. A preprint of this paper is included in this repository:  [SLloydEtAl2022_QuIK_preprint.pdf](SLloydEtAl2022_QuIK_preprint.pdf).

> S. Lloyd, R. Irani, and M. Ahmadi, "Fast and Robust Inverse Kinematics for Serial Robots using Halley’s Method," IEEE Transactions on Robotics, vol. 38, no. 5, pp. 2768–2780, Oct. 2022. doi: [10.1109/TRO.2022.3162954](http://dx.doi.org/10.1109/TRO.2022.3162954). A preprint of this paper can be found in the current repository, [SLloydEtAl2022_QuIK_preprint.pdf](SLloydEtAl2022_QuIK_preprint.pdf).

This repository includes the code to use the QuIK algorithm in ROS2, or just in C++ in general. Examples are given in python as well.

## How to build

For optimal performance, build using the release flag. For this type of iterative code, the compiler optimizations make it run about 10x faster. It will work just fine without it, however.
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Code Description and Usage

The main C++ code is stored in `include/quik` and `src`. The code can be used directly in your C++ projects, or in C++ ros nodes, or by building and running the provided kinematics service node (see below).

### Basic Usage

This library does not use URDF like many ROS packages; instead, it builds the robot structure using Denevit-Hartenburg (DH) convention. The DH table is a method to describe the geometry of a robot or a kinematic chain. It uses four parameters - `a`, `alpha`, `d`, and `theta` - to define the spatial relationship between adjacent links in the chain.

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
DH << 1./40,	-M_PI/2, 	183./1000,	0,
      -63./200,	0,        	0,			0,
      -7./200,	M_PI/2,		0,			0,
      0,			-M_PI/2,	73./200,	0,
      0,  		M_PI/2,		0,			0,
      0,  		0,			2./25,		0;

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

### Python Clients

To do.

## Requirements
All functions rely on the [Eigen 3.4](https://eigen.tuxfamily.org) linear algebra library, so you will also need to link to an appropriate library.

