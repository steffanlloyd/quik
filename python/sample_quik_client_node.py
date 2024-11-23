"""
@file sample_quik_client_node.py
@author Steffan Lloyd (steffan.lloyd@nibio.no)
@brief Demonstrates the code to call the kinematics service node define in this
package from python. Note that running the code directly in C++ will be much faster
(microseconds instead of milliseconds).

You will not be able to run this node as-is, since ros2 packages don't easily
support being both python and c++ nodes (it is possible, but not with custom 
interfaces also being defined). So to run this code, you'll need to either make
your own python node and copy this code in, or you can just run the code directly
(e.g. python3 src/quik/python/sample_quik_client_node.py).

@date 2024-11-23

@copyright Copyright (c) 2024
"""
import rclpy
from rclpy.node import Node
from quik.srv import FKService, IKService, JacobianService
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import quik_service_helpers

np.set_printoptions(precision=4)

class SampleClientNode(Node):
    def __init__(self):
        super().__init__('sample_client_node')

        # Init service clients and wait until they're available
        self.fk_client = self.create_client(FKService, 'fk_service')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('fk_service not available, waiting again...')

        self.ik_client = self.create_client(IKService, 'ik_service')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ik_service not available, waiting again...')

        self.jacobian_client = self.create_client(JacobianService, 'jacobian_service')
        while not self.jacobian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('jacobian_service not available, waiting again...')

        # self.timer_callback()
        self.q = np.zeros(5)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info("Set up kinematics loop to run every 5 seconds.")

    def fk_service_callback(self, future):
        """ Callback for FK service. Echos out the recieved robot pose.
            Then, it perturbs the original joint joint angles to form an 
            "initial guess", and gives the result of the forward kinematics
            the ik_service service to see if it can "re-find" the original
            joint angles.
        """
        
        fk_res = quik_service_helpers.fk_parse_response(future.result())
        self.get_logger().info(f"Got FK result:\nQuat: {fk_res['quat']}\nPosition: {fk_res['d']}")

        # Perturb initial joint angles
        perturbation = .1
        q_perturbed = self.q + np.random.uniform(-perturbation, perturbation)
        self.get_logger().info(f"Joint angles perturbed to: {q_perturbed}")

        # Call IK service
        ik_future = self.ik_client.call_async(quik_service_helpers.ik_make_request(fk_res['quat'], fk_res['d'], q_perturbed))
        ik_future.add_done_callback(self.ik_service_callback)


    def jacobian_service_callback(self, future):
        """ Callback for the Jacobian service. Echos out the recieved Jacobian matrix.
        """
        jacobian = quik_service_helpers.jacobian_parse_response(future.result())
        self.get_logger().info(f"Got Jacobian result:\n{jacobian}")

    def ik_service_callback(self, future):
        """ Callback for the IK service. Echos out the recieved result and algorithm parameters
        """
        ik_res = quik_service_helpers.ik_parse_response(future.result())
    
        # Log the results
        self.get_logger().info(f'IK result: {ik_res["q_star"]}')
        self.get_logger().info(f"Error on each joint: {ik_res['e_star']} (norm {np.linalg.norm(ik_res['e_star']):.4g}).")
        self.get_logger().info(f"Took {ik_res['iter']} iterations, broke because: {ik_res['break_reason']}, success {ik_res['success']}.")

    def timer_callback(self):
        """ Initiates a FK and Jacobian service call from a random set of joint angles.
            The FK service callback then initiates an IK call to try to "refind" the original
            joint angles again.
        """
        # Generate a random pose
        self.q = np.random.uniform(-1, 1, 6)
        self.get_logger().info(f"Random joint angles generated: {self.q}")

        # Call FK service and trigger backwards IK call
        fk_future = self.fk_client.call_async(quik_service_helpers.fk_make_request(self.q))
        fk_future.add_done_callback(self.fk_service_callback)

        # Call Jacobian service
        jacobian_future = self.jacobian_client.call_async(quik_service_helpers.jacobian_make_request(self.q))
        jacobian_future.add_done_callback(self.jacobian_service_callback)

def main(args=None):
    rclpy.init(args=args)
    node = SampleClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()