import rospy

from .pid import PIDController
from .common import compute_theta, angles_difference

from std_msgs.msg import Float32


class PIDSpeedController(object):
    def __init__(self, frame):
        self._frame = frame
        speed_controller_params = rospy.get_param(
            '/planner/move_controller/speed_controller/speed_coefs',
            {}
        )
        position_controller_params = rospy.get_param(
            '/planner/move_controller/speed_controller/position_coefs',
            {}
        )
        self.theta_threshold = rospy.get_param(
            '/planner/move_controller/speed_controller/theta_threshold',
            0.1
        )

        self.speed_pid = PIDController(speed_controller_params)
        self.position_pid = PIDController(position_controller_params)

        self.debug_publisher = rospy.Publisher(
            'planner/debug/ang_dif',
            Float32,
            queue_size=10
        )

    def execute(self, target):
        current_pos = self._frame.get_robot_pose()
        yaw = current_pos[2]
        target_pos = target.get_target_point()

        current_speed = self._frame.get_robot_speed()
        target_speed = target.get_target_speed()

        theta = compute_theta(current_pos, target_pos)

        if abs(angles_difference(yaw, theta)) > self.theta_threshold:
            print("BIGG DIFF BTWN ANGLES: {0}, yaw: {1}, theta: {2}".format(
                angles_difference(yaw, theta), yaw, theta
            ))
            self.position_pid.reset()
            self.speed_pid.reset()
            s = 0.0
        else:
            s1 = self.speed_pid.execute(target_speed - current_speed)

            longitudinal_distance = target.get_robot_to_target_distance()
            s2 = self.position_pid.execute(longitudinal_distance)
            s = s1 + s2

        msg = Float32()
        msg.data = abs(angles_difference(yaw, theta))
        self.debug_publisher.publish(msg)

        return s
