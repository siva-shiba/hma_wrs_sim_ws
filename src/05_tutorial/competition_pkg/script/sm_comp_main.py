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


"""sample state machine class for competition.

Init       : Initialization state. Set initial values for rosparam, etc.
Wait4Start : Wait for seconds as defined in the launch file. (default is 0)
Start      : Start the countdown for the clean up task.
Task1      : Sample state machine. Wait 30 seconds for simulator time.
End        : Process when tasks end normally (e.g., when a task is completed).
Except     : Process when exit in case of some abnormality.
"""

import rospy
import smach

from std_msgs.msg import Bool


class Init(smach.State):
    """Initialize state class.
        
    Set initial values for rosparam, etc., but not done this time.
    """
    
    def __init__(self, lib={}):
        """Initialize Init class & state machine pass.
        
        Args:
            lib (dict[str,instance], optional): library instances
        """
        smach.State.__init__(self, outcomes=["next", "except"])
        self.lib = lib
        return

    def __del__(self):
        """Delete the object."""
        return 

    def execute(self, userdata):
        """Initialize execute code."""
        return "next"


class Wait4Start(smach.State):
    """Wait for start state class.
       
    Wait for seconds as defined in the launch file. (default is 0)
    """
    
    def __init__(self, lib={}):
        """Initialize Wait4Start class.

            Read the seconds from rosparam. 
        
        Args:
            lib (dict[str,instance], optional): library instances
        """
        smach.State.__init__(self, outcomes=["next", "except"])
        self.lib = lib

        # ROS I/F
        self.p_wait_time = rospy.get_param(rospy.get_name() + "/start_wait_time", 0.0)

        return

    def __del__(self):
        """Delete the object."""
        return

    def execute(self, userdata):
        """Wait for seconds as defined in the launch file (default is 0)."""
        rospy.loginfo("[" + rospy.get_name() + "]: Wait for start: " + str(self.p_wait_time) + " sec")
        rospy.sleep(float(self.p_wait_time))
        rospy.loginfo("[" + rospy.get_name() + "]: Let's GO!!")
        return "next"


class Start(smach.State):
    """Start state machine class.
        
    Publish the countdown flag for the clean up task.
    """
    
    def __init__(self, lib={}):
        """Initialize State class.
        
        Args:
            lib (dict[str,instance], optional): library instances
        """
        smach.State.__init__(self, outcomes=["next", "except"])
        self.lib = lib

        # ROS I/F
        self.pub_time_supervisor = rospy.Publisher(
            "/manage_task_time_node/run_enable", Bool, queue_size=1)

        return

    def __del__(self):
        """Delete the object."""
        return

    def execute(self, userdata):
        """Start task timer."""
        self.pub_time_supervisor.publish(Bool(True))
        return "next"


class Task1(smach.State):
    """Sample state machine class.
    
    Wait 30 seconds for simulator time
    """

    def __init__(self, lib={}):
        """Initialize class.
        
        Args:
            lib (dict[str,instance], optional): library instances
        """
        smach.State.__init__(self, outcomes=["next", "except"])

        self.lib = lib
        return

    def __del__(self):
        """Delete the object."""
        return

    def execute(self, userdata):
        """Sample execute code."""
        rospy.sleep(30.0)
        return "next"


class End(smach.State):
    """End state.

    Process when tasks end normally (e.g., when a task is completed).
    """

    def __init__(self, lib={}):
        """Initialize End class.
        
        Args:
            lib (dict, optional): library
        """
        smach.State.__init__(self, outcomes=["end"])
        self.lib = lib

        return

    def __del__(self):
        """Delete the object."""
        return

    def execute(self, userdata):
        """Exit program at normal."""
        return "end"


class Except(smach.State):
    """Except state.

    Process when exit in case of some abnormality.
    """

    def __init__(self, lib={}):
        """Initialize Except class.
        
        Args:
            lib (dict, optional): library
        """
        smach.State.__init__(self, outcomes=["except"])
        self.lib = lib

        return

    def __del__(self):
        """Delete the object."""
        return

    def execute(self, userdata):
        """Exit program in case of error."""
        return "except"
