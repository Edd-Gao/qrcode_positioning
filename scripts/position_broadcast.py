#!/usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header

pub = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=1)

sequence = 0
i = 0


def gazeboCallBack(data, model_name):
    # print data.pose[1].position
    global sequence
    global i

    i += 1

    if i % 5 == 0:

        pose = data.pose[data.name.index(model_name)]

        temp = pose.position.x
        pose.position.x = -pose.position.y

        pose.position.y = temp

        header = Header()

        header.seq = sequence
        sequence += 1
        header.stamp = rospy.Time.now()

        pub.publish(header, pose)


def positionBroadcast():
    rospy.init_node('position_broadcast', anonymous=True)

    while not rospy.has_param('~model_name'):
        pass

    model_name = rospy.get_param('~model_name')

    assert isinstance(model_name, str)
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazeboCallBack, model_name)

    rospy.spin()


if __name__ == '__main__':
    positionBroadcast()
