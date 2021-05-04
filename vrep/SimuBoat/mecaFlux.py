import rospy
import numpy as np
import simu
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Wrench
from scipy.spatial.transform import Rotation



R = np.array([[1,0,0],[0,1,0],[0,0,1]])
pos     = np.zeros(3)
vitesse = np.zeros(3)
omega   = np.zeros(3)

F = np.zeros(3)
M = np.zeros(3)

def get_Pose(data):
	global pos
	global R
	pos = np.array([data.position.x,data.position.y,data.position.z-0.4])
	R = Rotation.from_quat([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]).as_matrix()

def get_Speed(data):
	global vitesse
	global omega
	vitesse = np.array([data.position.x,data.position.y,data.position.z])
	omega   = np.array([data.orientation.x, data.orientation.y, data.orientation.z])


def talker():
	pub = rospy.Publisher('Torseur', Wrench, queue_size=10)
	rate = rospy.Rate(20) # 50hz
	msg = Wrench()
	while not rospy.is_shutdown():
		F,M = simu.Calcul(R,pos,vitesse,omega)
		msg.force.x = F[0];msg.force.y = F[1];msg.force.z = F[2];
		msg.torque.x = M[0];msg.torque.y = M[1];msg.torque.z = M[2];
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	rospy.init_node('mecaFlux', anonymous=True)
	rospy.Subscriber("pose", Pose, get_Pose)
	rospy.Subscriber("speed", Pose, get_Speed)
	
	talker()
	
	rospy.spin()

