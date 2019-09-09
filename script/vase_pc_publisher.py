import rospy 
import csv
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

csv_file = open("../model/vase.csv", "r" )
f = csv.reader(csv_file, delimiter = ',')

msg_pc = PointCloud()
idx = 0
for row in f:
    scaler = 0.001
    pt = Point32()
    pt.x = float(row[0]) * scaler
    pt.y = float(row[1]) * scaler
    pt.z = float(row[2]) * scaler
    msg_pc.points.append(pt)

rospy.init_node("vase_pc_pub", anonymous = True)
pub = rospy.Publisher("vase_pc", PointCloud, queue_size = 10)
r = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(msg_pc)
    r.sleep()



