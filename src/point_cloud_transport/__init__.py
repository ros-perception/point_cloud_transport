# Copyright (c) 2023, Czech Technical University in Prague
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Encoding and decoding of point clouds compressed with any point cloud transport.

Example usage:

.. code-block:: python

    from point_cloud_transport import decode, encode
    from sensor_msgs.msg import PointCloud2

    raw = PointCloud2()
    ... # fill the cloud
    compressed, err = encode(raw, 'draco')
    if compressed is None:
      rospy.logerr("Error encoding point cloud: " + err)
      return False
    # work with the compressed instance in variable compressed

    # beware, for decoding, we do not specify "raw", but the codec used for encoding
    raw2, err = decode(compressed, 'draco')
    if raw2 is None:
      rospy.logerr("Error decoding point cloud: " + err)
      return False
    # work with the PointCloud2 instance in variable raw2
"""

from point_cloud_transport.decoder import decode
from point_cloud_transport.encoder import encode
from point_cloud_transport.publisher import Publisher
from point_cloud_transport.subscriber import Subscriber


if __name__ == '__main__':
    def main():
        import rospy
        import time
        from sensor_msgs.msg import PointCloud2, PointField

        raw = PointCloud2()
        raw.header.stamp = rospy.Time(10)
        raw.header.frame_id = 'test'
        raw.width = raw.height = 2
        raw.is_bigendian = False
        raw.is_dense = True
        raw.point_step = 8
        raw.row_step = raw.point_step * 2
        raw.fields = [
            PointField(name='x', offset=0, datatype=PointField.INT32, count=1),
            PointField(name='y', offset=0, datatype=PointField.INT32, count=1),
        ]

        import struct
        data = struct.pack('i', 0) + struct.pack('i', 1000) + \
            struct.pack('i', 5000) + struct.pack('i', 9999)
        data += struct.pack('i', 0) + struct.pack('i', -1000) + \
            struct.pack('i', -5000) + struct.pack('i', -9999)
        import sys
        if sys.version_info[0] == 2:
            raw.data = map(ord, data)

        rospy.init_node('test_node')
        rospy.loginfo('start')
        time.sleep(1)

        start = time.time()
        for i in range(1):
            compressed, err = encode(raw, 'draco', {'encode_speed': 1})
            # compressed, err = encode(raw, 'draco')
            if compressed is None:
                print(err)
                break

            # raw2, err = decode(compressed, 'draco', {"SkipDequantizationPOSITION": True})
            raw2, err = decode(compressed, 'draco')

        end = time.time()
        print(end - start)
        print(bool(raw))
        print(bool(compressed))
        print(err)
        print(raw.fields == raw2.fields)
        print(raw == raw2)

        def cb(msg):
            print(msg == raw)  # they are not equal exactly...

        def cb_ros(msg):
            print(msg == compressed)

        sub = Subscriber('test', cb, default_transport='draco', queue_size=1)
        sub  # prevent unused variable warning
        sub2 = rospy.Subscriber('test', PointCloud2, cb, queue_size=1)
        sub2  # prevent unused variable warning
        sub3 = rospy.Subscriber('test/draco', type(compressed), cb_ros, queue_size=1)
        sub3  # prevent unused variable warning
        pub = Publisher('test', queue_size=10)
        time.sleep(0.2)
        pub.publish(raw)
        rospy.spin()

    main()
