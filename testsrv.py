#!/usr/bin/python

# Utility program for testing the heightmap node.
# Interactive use is suggested: import this module from a Python REPL,
# after calling matplotlib.pyplot.ion().

import rospy
import heightmap.srv as srv
import numpy as np
import matplotlib.pyplot as plt
import geometry_msgs.msg

service_name = 'heightmap_query'

rospy.wait_for_service(service_name)
heightmap_query = rospy.ServiceProxy(service_name, srv.Query)
    
def run(x, y, frame_id, **kwargs):
    corner = geometry_msgs.msg.PointStamped()
    corner.point.x = x
    corner.point.y = y
    corner.header.frame_id = frame_id
    
    response = heightmap_query(corner=corner, **kwargs)
    # NOTE NOTE NOTE The order of x and y appears wrong, yet gives
    # correct result. MUST INVESTIGATE
    shape = (response.y_size, response.x_size)
    A = np.array(response.map, dtype=np.float64).reshape(shape)
    A = np.nan_to_num(A)
    plt.imshow(A)
