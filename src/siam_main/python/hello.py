import rospy
import std_msgs.msg

node = rospy.init_node('siamsim')
pub = rospy.Publisher('/god/insert', std_msgs.msg.String, queue_size=10, tcp_nodelay=True)
file = open("/opt/ros/noetic/share/siam_sim/src/siam_main/models/dronechallenge_models/drone2/model.sdf")
model = file.read()
msg = std_msgs.msg.String(model)

#msg.data = model.replace('\n','').replace('\t','').replace(' ','')

pub.publish(msg)
r = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(msg)
    r.sleep()