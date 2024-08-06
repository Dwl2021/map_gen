#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

points = [
    [5, 3],
    [0, 0],
    [-5, 3],
    [-5, -3]
]

def publish_points(points):
    rospy.init_node('point_publisher', anonymous=True)
    marker_pub = rospy.Publisher('reference_points', Marker, queue_size=10)
    rate = rospy.Rate(10)

    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "points"
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    for point in points:
        p = Point()
        p.x, p.y = point
        p.z = 0
        marker.points.append(p)

    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_points(points)
    except rospy.ROSInterruptException:
        pass
