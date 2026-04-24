#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


class CompressedToRawBridge:
  def __init__(self):
    self.bridge = CvBridge()
    self.output_encoding = rospy.get_param("~output_encoding", "bgr8")
    input_topic = rospy.get_param("~input_topic", "/camera/image/compressed")
    output_topic = rospy.get_param("~output_topic", "/camera/image_raw")
    queue_size = int(rospy.get_param("~queue_size", 10))

    self.pub = rospy.Publisher(output_topic, Image, queue_size=queue_size)
    self.sub = rospy.Subscriber(
        input_topic, CompressedImage, self.callback, queue_size=queue_size
    )
    rospy.loginfo(
        "compressed_to_raw_bridge active: in='%s' out='%s' encoding='%s'",
        input_topic,
        output_topic,
        self.output_encoding,
    )

  def callback(self, msg):
    try:
      cv_img = self.bridge.compressed_imgmsg_to_cv2(
          msg, desired_encoding=self.output_encoding
      )
      out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding=self.output_encoding)
      out_msg.header = msg.header
      self.pub.publish(out_msg)
    except CvBridgeError as err:
      rospy.logwarn_throttle(2.0, "compressed_to_raw_bridge decode failed: %s", err)


def main():
  rospy.init_node("compressed_to_raw_bridge")
  CompressedToRawBridge()
  rospy.spin()


if __name__ == "__main__":
  main()
