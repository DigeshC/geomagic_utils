import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from sensor_msgs.msg import Joy, JointState
from geomagic_control.msg import DeviceButtonEvent, DeviceFeedback
from PyKDL import Frame, Vector, Rotation
import sys


# Utilities
def kdl_frame_to_msg_pose(kdl_pose):
    ps = PoseStamped()
    p = ps.pose
    p.position.x = kdl_pose.p[0]
    p.position.y = kdl_pose.p[1]
    p.position.z = kdl_pose.p[2]

    p.orientation.x = kdl_pose.M.GetQuaternion()[0]
    p.orientation.y = kdl_pose.M.GetQuaternion()[1]
    p.orientation.z = kdl_pose.M.GetQuaternion()[2]
    p.orientation.w = kdl_pose.M.GetQuaternion()[3]

    return ps


def msg_pose_to_kdl_frame(msg_pose):
    pose = msg_pose.pose
    f = Frame()
    f.p[0] = pose.position.x
    f.p[1] = pose.position.y
    f.p[2] = pose.position.z
    f.M = Rotation.Quaternion(pose.orientation.x,
                              pose.orientation.y,
                              pose.orientation.z,
                              pose.orientation.w)

    return f


# Init Relevant MTM
class ProxyMTM:
    def __init__(self, arm_name):
        pose_str = '/dvrk/' + arm_name + '/position_cartesian_current'
        wrench_str = '/dvrk/' + arm_name + '/set_wrench_body'
        gripper_str = '/dvrk/' + arm_name + '/state_gripper_current'


        self._mtm_arm_type = None

        if arm_name == 'MTMR':
            self._mtm_arm_type = 0
        elif arm_name == 'MTML':
            self._mtm_arm_type = 1
        else:
            print('WARNING, MTM ARM TYPE NOT UNDERSTOOD, SHOULD BE MTMR or MTML')
        pass

        self.pose = Frame()
        self.pose.p = Vector(0, 0, 0)
        self.pose.M = Rotation.Quaternion(0, 0, 0, 1)
        self.buttons = 0

        self._commanded_force = Vector(0, 0, 0)

        self._gripper_min_angle = -3.16
        self._gripper_max_angle = 1.2
        self._gripper_angle = JointState()
        self._gripper_angle.position.append(0)

        self._pose_pub = rospy.Publisher(pose_str, PoseStamped, queue_size=1)
        self._gripper_pub = rospy.Publisher(gripper_str, JointState, queue_size=1)

        self._force_sub = rospy.Subscriber(wrench_str, WrenchStamped, self.force_cb, queue_size=10)

        self._foot_pedal = ProxyButtons()
        pass

    def force_cb(self, msg):
        self._commanded_force[0] = msg.wrench.force.x
        self._commanded_force[1] = msg.wrench.force.y
        self._commanded_force[2] = msg.wrench.force.z

    def set_pose(self, pose):
        self.pose = pose
        msg = kdl_frame_to_msg_pose(self.pose)
        self._pose_pub.publish(msg)

    def set_gripper_angle(self, angle):
        if angle == 0:
            self._gripper_angle.position[0] = self._gripper_min_angle
        elif angle == 1:
            self._gripper_angle.position[0] = self._gripper_max_angle

        self._gripper_pub.publish(self._gripper_angle)

    def get_commanded_force(self):
        return self._commanded_force

    def set_foot_pedal(self, btn):
        if self._mtm_arm_type == 0:
            self._foot_pedal.set_clutch_footpedals(btn)
        elif self._mtm_arm_type == 1:
            self._foot_pedal.set_cam_footpedals(btn)


# Init one instance of MTM FootPedals
class ProxyButtons:
    def __init__(self):
        self.buttons = 0
        base_str = '/dvrk/footpedals/'
        clutch_str = base_str + 'clutch'
        cam_str = base_str + 'cam'

        self._clutch = Joy()
        self._clutch.buttons.append(0)
        self._cam = Joy()
        self._cam.buttons.append(0)

        self._clutch_pub = rospy.Publisher(clutch_str, Joy, queue_size=1)
        self._cam_pub = rospy.Publisher(cam_str, Joy, queue_size=1)
        pass

    def set_clutch_footpedals(self, btn):
        self._clutch.buttons[0] = btn
        self._clutch_pub.publish(self._clutch)
        pass

    def set_cam_footpedals(self, btn):
        self._cam.buttons[0] = btn
        self._cam_pub.publish(self._cam)
        pass

    # Use this function and to set the value of both footpedals, Use afInputDevices
    # mapping to map one device's clutch to camera, and one to clutch
    def set_cam_clutch_footpedals(self, btn):
        self.set_clutch_footpedals(btn)
        self.set_cam_footpedals(btn)


# Init everthing related to Geomagic
class GeomagicDevice:
    # The name should include the full qualified prefix. I.e. '/Geomagic/', or '/omniR_' etc.
    def __init__(self, name, mtm_name):
        pose_str = name + 'pose'
        button_str = name + 'button'
        force_str = name + 'force_feedback'

        self._active = False
        self._scale = 0.001
        self.pose = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.base_frame = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.tip_frame = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.grey_button_pressed = False
        self.white_button_pressed = False
        self._force = DeviceFeedback()
        self._force.force.x = 0
        self._force.force.y = 0
        self._force.force.z = 0

        self._pose_sub = rospy.Subscriber(pose_str, PoseStamped, self.pose_cb, queue_size=10)
        self._button_sub = rospy.Subscriber(button_str, DeviceButtonEvent, self.buttons_cb, queue_size=10)
        self._force_pub = rospy.Publisher(force_str, DeviceFeedback, queue_size=1)

        self._mtm_handle = ProxyMTM(mtm_name)

    def pose_cb(self, msg):

        cur_frame = msg_pose_to_kdl_frame(msg)
        # Convert the position based on the scale
        cur_frame.p = cur_frame.p * self._scale

        self.pose = self.base_frame.Inverse() * cur_frame * self.tip_frame
        # Mark active as soon as first message comes through
        self._active = True
        pass

    def buttons_cb(self, msg):
        self.grey_button_pressed = msg.grey_button
        self.white_button_pressed = msg.white_button
        pass

    def command_force(self, force):
        pass

    def process_commands(self):
        if self._active:
            self._mtm_handle.set_pose(self.pose)
            self._mtm_handle.set_gripper_angle(self.white_button_pressed)
            self._mtm_handle.set_foot_pedal(self.grey_button_pressed)

            force = self._mtm_handle.get_commanded_force()
            force = self.base_frame.M.Inverse() * force
            self._force.force.x = force[0]
            self._force.force.y = force[1]
            self._force.force.z = force[2]
            # self._force_pub.publish(self._force)


def main():
    _pair_one_specified = True
    _pair_two_specified = False
    rospy.init_node('geomagic_mtm_proxy_node')

    _geomagic_one_name = '/Geomagic/'
    _geomagic_two_name = '/Geomagic/'

    _device_pairs = []

    _mtm_one_name = 'MTMR'
    _mtm_two_name = 'MTML'

    if len(sys.argv) > 1:
        _geomagic_one_name = sys.argv[1]

    if len(sys.argv) > 2:
        _mtm_one_name = sys.argv[2]
        _geomagic_two_name = True

    if _pair_one_specified:
        geomagic_one = GeomagicDevice(_geomagic_one_name, _mtm_one_name)
        _device_pairs.append(geomagic_one)

    if _pair_two_specified:
        geomagic_two = GeomagicDevice(_geomagic_two_name, _mtm_two_name)
        _device_pairs.append(geomagic_two)

    rate = rospy.Rate(500)
    msg_index = 0

    while not rospy.is_shutdown():
        for dev in _device_pairs:
            dev.process_commands()
        rate.sleep()
        msg_index = msg_index + 1
        if msg_index % 500 == 0:
            print('MTM MIMIC VIA GEOMAGIC ACTIVE...')
        if msg_index >= 5000:
            msg_index = 0


if __name__ == "__main__":
    main()
