import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Joy
from geomagic_control.msg import DeviceButtonEvent, DeviceFeedback
from PyKDL import Frame, Vector, Rotation


# Utilities
def kdl_frame_to_msg_pose(kdl_pose):
    p = Pose()
    p.position.x = kdl_pose.p[0]
    p.position.y = kdl_pose.p[1]
    p.position.z = kdl_pose.p[2]

    p.orientation.x = kdl_pose.M.GetQuaternion()[0]
    p.orientation.y = kdl_pose.M.GetQuaternion()[1]
    p.orientation.z = kdl_pose.M.GetQuaternion()[2]
    p.orientation.w = kdl_pose.M.GetQuaternion()[3]

    return p


def msg_pose_to_kdl_frame(msg_pose):
    f = Frame()
    f.p.x = msg_pose.position.x
    f.p.y = msg_pose.position.y
    f.p.z = msg_pose.position.z

    f.M.Quaternion(msg_pose.orientation.x,
                   msg_pose.orientation.y,
                   msg_pose.orientation.z,
                   msg_pose.orientation.w)

    return f


# Init Relevant MTM
class ProxyMTM:
    def __init__(self, arm_name):
        pose_str = '/dvrk/' + arm_name + '/position_cartesian_current'

        self.pose = Frame()
        self.pose.p(0, 0, 0)
        self.pose.M.Quaternion(0, 0, 0, 1)
        self.buttons = 0

        self._pose_pub = rospy.Publisher(pose_str, PoseStamped)
        pass

    def set_pose(self, pose):
        self.pose = pose
        msg = kdl_frame_to_msg_pose(self.pose)
        self._pose_pub.publish(msg)


# Init one instance of MTM FootPedals
class ProxyFootPedal:
    def __init__(self):
        self.buttons = 0
        base_str = '/dvrk/footpedals/'
        clutch_str = base_str + 'clutch'
        cam_str = base_str + 'cam'
        self._clutch_pub = rospy.Publisher(clutch_str, Joy)
        self._cam_pub = rospy.Publisher(cam_str, Joy)
        pass


# Init everthing related to Geomagic
class GeomagicDevice:
    # The name should include the full qualified prefix. I.e. '/Geomagic/', or '/omniR_' etc.
    def __init__(self, name, mtm_name):
        pose_str = name + 'pose'
        button_str = name + 'button'
        force_str = name + 'force_feedback'

        self.pose = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.base_frame = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.tip_frame = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.buttons = 0

        self._pose_sub = rospy.Subscriber(pose_str, PoseStamped, self.pose_cb, queue_size=10)
        self._button_sub = rospy.Subscriber(button_str, DeviceButtonEvent, self.buttons_cb, queue_size=10)
        self._force_pub = rospy.Publisher(force_str, DeviceFeedback)

        self._mtm_handle = ProxyMTM(mtm_name)
        pass

    def pose_cb(self, msg):
        self.pose = msg_pose_to_kdl_frame(msg.pose)
        pass

    def buttons_cb(self, msg):
        pass

    def command_force(self, force):
        pass


rospy.init_node('geomagic_mtm_proxy_node')
