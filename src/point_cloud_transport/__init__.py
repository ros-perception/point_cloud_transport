# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Encoding and decoding of point clouds compressed with any point cloud transport.

Example usage:

.. code-block:: python

    from point_cloud_transport import decode, encode
    from sensor_msgs.msg import PointCloud2

    raw = PointCloud2()
    ... # fill the cloud
    compressed, err = encode(raw, "draco")
    if compressed is None:
      rospy.logerr("Error encoding point cloud: " + err)
      return False
    # work with the compressed instance in variable compressed

    # beware, for decoding, we do not specify "raw", but the codec used for encoding
    raw2, err = decode(compressed, "draco")
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
        raw.header.frame_id = "test"
        raw.width = raw.height = 2
        raw.is_bigendian = False
        raw.is_dense = True
        raw.point_step = 8
        raw.row_step = raw.point_step * 2
        raw.fields = [
            PointField(name="x", offset=0, datatype=PointField.INT32, count=1),
            PointField(name="y", offset=0, datatype=PointField.INT32, count=1),
        ]

        import struct
        data = struct.pack('i', 0) + struct.pack('i', 1000) + struct.pack('i', 5000) + struct.pack('i', 9999)
        data += struct.pack('i', 0) + struct.pack('i', -1000) + struct.pack('i', -5000) + struct.pack('i', -9999)
        import sys
        if sys.version_info[0] == 2:
            raw.data = map(ord, data)

        rospy.init_node("test_node")
        rospy.loginfo("start")
        time.sleep(1)

        start = time.time()
        for i in range(1):
            compressed, err = encode(raw, "draco", {"encode_speed": 1})
            # compressed, err = encode(raw, "draco")
            if compressed is None:
                print(err)
                break

            # raw2, err = decode(compressed, "draco", {"SkipDequantizationPOSITION": True})
            raw2, err = decode(compressed, "draco")

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

        sub = Subscriber("test", cb, default_transport="draco", queue_size=1)
        sub2 = rospy.Subscriber("test", PointCloud2, cb, queue_size=1)
        sub3 = rospy.Subscriber("test/draco", type(compressed), cb_ros, queue_size=1)
        pub = Publisher("test", queue_size=10)
        time.sleep(0.2)
        pub.publish(raw)
        rospy.spin()

    main()
