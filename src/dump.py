import argparse
import numpy as np
import sys
import time
from PIL import Image

from geometry_msgs.msg import PointStamped
import heightmap.srv
import rospy

def parse_tuple(s, conv=int):
    return map(conv, s.split(','))

ap = argparse.ArgumentParser(
    description='Dump a section of the heightmap from a heightmap_node to an image file')
ap.add_argument('--frame', '-f', metavar='FRAME_ID',
                action='store', dest='tf_frame', default='map',
                help="request's TF frame of reference (default is 'map')")
ap.add_argument('--corner', '-c', metavar='X,Y',
                action='store', default='0,0',
                help='position of the captured rectangle\'s corner')
ap.add_argument('--size', '-s', metavar='W,H',
                action='store', required=True,
                help='size of the captured rectangle (default is 0.0,0.0)')
ap.add_argument('--res', '-r', metavar='N,M',
                action='store', required=True,
                help='sampling resolution (and resolution of the image), '
                'in samples/unit-of-distance')
ap.add_argument('--output', '-o', metavar='FILENAME',
                action='store', required=True,
                dest='output_filename', help='image file to write')
args = ap.parse_args()

corner_pos = parse_tuple(args.corner, float)
size = parse_tuple(args.size, float)
res = parse_tuple(args.res, int)
n_samples = res[0] * size[0], res[1] * size[1]

print 'Waiting for service "heightmap"...'
rospy.wait_for_service('heightmap')
query = rospy.ServiceProxy('heightmap', heightmap.srv.Query)

corner = PointStamped()
corner.header.frame_id = args.tf_frame
corner.point.x, corner.point.y = corner_pos

print 'Request sent'
req_time = time.time()
response = query(corner=corner,
                 x_size=size[0], y_size=size[1],
                 x_samples=n_samples[0], y_samples=n_samples[1])
print 'Got response in', (time.time() - req_time), 'secs'
data = np.array(response.map).reshape(response.y_samples, response.x_samples)
print 'Exporting image: size', data.shape

# PIL expects floats to be in the range 0.0-255.0 (even though they
# aren't bytes)
Image.fromarray(data*255).convert('L').save(args.output_filename)
