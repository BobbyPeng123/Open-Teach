import numpy as np

from openteach.constants import VR_FREQ, VR_TCP_HOST, VR_TCP_PORT, VR_CONTROLLER_TOPIC
from openteach.components import Component
from openteach.utils.timer import FrequencyTimer
from openteach.utils.network import create_subscriber_socket, ZMQKeypointPublisher
from openteach.components.detector.utils.controller_state import parse_controller_state

# This class is used to detect the hand keypoints from the VR and publish them.            
class OculusVRStickDetector(Component):
    def __init__(self, 
                 host, 
                controller_state_pub_port,
                noise_angle_range_deg=None,
    ):
        self.notify_component_start('vr detector')

        # Create a subscriber socket
        self.stick_socket = create_subscriber_socket(VR_TCP_HOST, VR_TCP_PORT, VR_CONTROLLER_TOPIC)
        self.last_position = np.zeros((3,))
        # self.theta_min = 15 * np.pi / 180.
        self.theta_range = None
        if noise_angle_range_deg is not None:
            self.theta_range = np.array(noise_angle_range_deg) * np.pi / 180.
        
        # Create a publisher for the controller state
        self.controller_state_publisher = ZMQKeypointPublisher(
            host = host,
            port = controller_state_pub_port
        )
        self.timer = FrequencyTimer(VR_FREQ)
    
    # Function to Publish the message
    def _publish_controller_state(self, controller_state):
        self.controller_state_publisher.pub_keypoints(
            keypoint_array = controller_state,
            topic_name = 'controller_state'
        )

    # Function to publish the left/right hand keypoints and button Feedback 
    def stream(self):

        print("oculus stick stream")
        while True:
            try:
                self.timer.start_loop()

                message = self.stick_socket.recv_string()
                if message == "oculus_controller":
                    continue

                controller_state = parse_controller_state(message)
                if self.theta_range is not None:
                    delta = controller_state.right_local_position - self.last_position
                    if np.linalg.norm(delta) > 0:
                        delta_unit = delta / np.linalg.norm(delta)
                        random_unit = np.random.uniform(-1, 1, (3,))
                        random_unit /= np.linalg.norm(random_unit)
                        curr_angle = np.arccos(np.dot(random_unit, delta_unit))
                        if curr_angle > self.theta_range[1] or curr_angle < self.theta_range[0]:
                            sampled_angle = np.random.uniform(*self.theta_range)
                        # sampled_angle = self.theta_max
                            m = (np.cos(sampled_angle) - np.cos(curr_angle)) / (1. - np.cos(curr_angle))
                            random_unit = m * delta_unit + (1 - m) * random_unit
                            curr_angle = np.arccos(np.dot(random_unit, delta_unit))
                        random_vec = np.linalg.norm(delta) * random_unit
                    controller_state.right_local_position = self.last_position + random_vec
                    self.last_position = controller_state.right_local_position
                # Publish message
                self._publish_controller_state(controller_state)

                self.timer.end_loop()

            except KeyboardInterrupt:
                break

        self.controller_state_publisher.stop()
        print('Stopping the oculus keypoint extraction process.')