#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2022 Hibikino-Musashi@Home
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Hibikino-Musashi@Home nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import os
import sys
import roslib
import rospy
import threading
import cv2
import message_filters

from cv_bridge import CvBridge
from gazebo_ros import gazebo_interface

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo


GP_DYNAMIC_LOOP_RATE = False
GP_LOOP_RATE = 30.0
camera_model_database_template = """<sdf version="1.4">
  <world name="default">
    <model name='task1_camera'>
      <static>true</static>
      <pose>-1.4411584138870239 -3.010014533996582 2.9286668300628662 0 0.7916429 1.5881942</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </visual>
        <sensor name='my_camera' type='camera'>
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image>
              <width>1920</width>
              <height>1080</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>0.0</updateRate>
            <cameraName>camera</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>camera_link</frameName>
            <hackBaseline>0.07</hackBaseline>
            <distortionK1>0.0</distortionK1>
            <distortionK2>0.0</distortionK2>
            <distortionK3>0.0</distortionK3>
            <distortionT1>0.0</distortionT1>
            <distortionT2>0.0</distortionT2>
        </plugin>
        </sensor>
      </link>
    </model>
  </world>
</sdf>"""

camera1_model_database_template = """<sdf version="1.4">
  <world name="default">
    <model name='task1_camera1'>
      <static>true</static>
      <pose>0.1 0.4 2.0 3.14 2.0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </visual>
        <sensor name='my_camera' type='camera'>
          <camera>
            <horizontal_fov>1.2</horizontal_fov>
            <image>
              <width>1920</width>
              <height>1080</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>0.0</updateRate>
            <cameraName>camera1</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>camera_link</frameName>
            <hackBaseline>0.07</hackBaseline>
            <distortionK1>0.0</distortionK1>
            <distortionK2>0.0</distortionK2>
            <distortionK3>0.0</distortionK3>
            <distortionT1>0.0</distortionT1>
            <distortionT2>0.0</distortionT2>
        </plugin>
        </sensor>
      </link>
    </model>
  </world>
</sdf>"""


class Shooting:
    def __init__(self, run_enable=True):
        """コンストラクタ.
        
        Args:
            run_enable (bool, optional): 実行許可
        """
        self._lock = threading.Lock()

        self._init_ros_time = rospy.Time.now()
        self._update_ros_time = {}
        self._prev_ros_time = self._init_ros_time

        self._lib = {}

        self._run_enable = Bool()
        self._image0 = Image()
        self._image1 = Image()
        self._cb = CvBridge()

        self._run_enable.data = run_enable
        self._save_path = roslib.packages.get_pkg_dir("hma_hsr_sim_pkg") + "/io/images"
        self._cnt = 0

        # ROSインタフェース
        self._sub_run_enable = rospy.Subscriber(
            "/manage_task_time_node/run_enable", Bool, self.subfRunEnable, queue_size=1)

        self._sub_image0 = message_filters.Subscriber(
            "/camera/image_raw", Image)

        self._sub_image1 = message_filters.Subscriber(
            "/camera1/image_raw", Image)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_image0, self._sub_image1],
            1,  # Queueサイズ
            0.1,  # 許容時間ずれ
            allow_headerless=True
        )
        self._sync.registerCallback(self.subfImage)

        return

    def delete(self):
        """デストラクタ."""
        for key in self._lib.keys():
            self._lib[key].delete()

        return

    def subfRunEnable(self, run_enable):
        """実行許可サブスクライブ関数.
        
        Args:
            run_enable (Bool): 実行許可
        """
        self._run_enable = run_enable

        if self._run_enable.data is True:
            rospy.loginfo("[" + rospy.get_name() + "]: SHOOTING START!!")

        return

    def subfImage(self, image0, image1):
        """Imageサブスクライブ関数.
        
        Args:
            image0 (Image()): 入力画像
            image1 (Image()): 入力画像
        """
        try:
            self.proc(image0, image1)
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        return

    def convSMI2CV(self, smi, encode, is_depth=False):
        """smiからcvに変換する関数.
        
        Args:
            smi (Image()): 画像（SMI）
            encode (str): 画像エンコード
            is_depth (bool, optional): depth画像判定
        
        Returns:
            array or int: 画像 or 1(失敗時)
        """
        try:
            if is_depth:
                cv_tmp = self._cb.imgmsg_to_cv2(smi, encode)
                cv = numpy.array(cv_tmp, dtype=numpy.uint16)
            else:
                cv = self._cb.imgmsg_to_cv2(smi, encode)
        except:
            rospy.logwarn("[" + rospy.get_name() + "]: CV bridge FAILURE")
            return 1
        return cv

    def drop_camera(self):
        rospy.loginfo('Drop {0}'.format("task1_camera"))
        initial_pose = Pose()
        initial_pose.position.x = -1.4411584138870239
        initial_pose.position.y = -3.010014533996582
        initial_pose.position.z = 2.9286668300628662
        initial_pose.orientation = Quaternion(
            -0.27499783039093018,
            0.27025458216667175,
            0.65808409452438354,
            0.646733283996582
        )

        model_xml = camera_model_database_template.replace('MODEL_NAME', "task1_camera")

        gazebo_interface.spawn_sdf_model_client("camera", model_xml, rospy.get_namespace(),
                                                initial_pose, "", "/gazebo")

        rospy.loginfo('Drop {0}'.format("task1_camera1"))

        model_xml = camera1_model_database_template.replace('MODEL_NAME', "task1_camera1")

        gazebo_interface.spawn_sdf_model_client("camera", model_xml, rospy.get_namespace(),
                                                initial_pose, "", "/gazebo")

    def proc(self, image0, image1):
        """処理関数.
        
        Args:
            image0 (Image): 入力画像
            image1 (Image): 入力画像
        """
        # 実行許可の検証
        if self._run_enable.data == False:
            return

        # proc
        cv_img0 = self.convSMI2CV(image0, "bgr8")
        cv_img1 = self.convSMI2CV(image1, "bgr8")
        cv2.imwrite(self._save_path + "/0/" + str(self._cnt).zfill(7) + ".jpg", cv_img0)
        cv2.imwrite(self._save_path + "/1/" + str(self._cnt).zfill(7) + ".jpg", cv_img1)
        self._cnt += 1
        # cv2.imshow("test", cv_img)
        # cv2.waitKey(1)
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = Shooting(False)

    rospy.on_shutdown(cls.delete)

    cls.drop_camera()

    while not rospy.is_shutdown():
        try:
            # cls.proc()
            pass
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()
