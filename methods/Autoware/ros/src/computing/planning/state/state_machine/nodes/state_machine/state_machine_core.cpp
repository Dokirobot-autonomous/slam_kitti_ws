/*
 *  Copyright (c) 2015, Nagoya University

 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "state_machine_core.h"

namespace state_machine
{
// Constructor
StateMachineNode::StateMachineNode() : private_nh_("~"), sc_()
{
  initForROS();
}

// Destructor
StateMachineNode::~StateMachineNode()
{
}

void StateMachineNode::initForROS()
{
  // ros parameter settings
  private_nh_.param<bool>("is_manual_light_detection", is_manual_light_detection_, true);

  // setup subscriber
  sub1_ = nh_.subscribe("light_color", 100, &StateMachineNode::callbackFromLightColor, this);
  sub2_ = nh_.subscribe("light_color_managed", 100, &StateMachineNode::callbackFromLightColorManaged, this);

  sub3_ = nh_.subscribe("change_flag", 100, &StateMachineNode::callbackFromChangeFlag, this);

  // setup publisher
  pub_ = nh_.advertise<std_msgs::String>("state", 10);
}

void StateMachineNode::run()
{
  ros::spin();
}

void StateMachineNode::publish() const
{
  /*
  std_msgs::Int32 msg;
  msg.data = sc_.getCurrentState();
  ROS_INFO("Current State: %d",sc_.getCurrentState());
  pub_.publish(msg);
  */

  std_msgs::String msg;
  msg.data = *sc_.getCurrentStateString();
  ROS_INFO_STREAM("Current State String : " << msg.data);
  pub_.publish(msg);
}

void StateMachineNode::callbackFromLightColor(const autoware_msgs::traffic_lightConstPtr& msg)
{
  ROS_INFO("Light color callback");
  if (is_manual_light_detection_)
    return;

  sc_.setLightColor(msg->traffic_light);
  sc_.update();
  publish();
}

void StateMachineNode::callbackFromLightColorManaged(const autoware_msgs::traffic_lightConstPtr& msg)
{
  ROS_INFO("Light color managed callback");
  if (!is_manual_light_detection_)
    return;

  sc_.setLightColor(msg->traffic_light);
  sc_.update();
  publish();
}

void StateMachineNode::callbackFromChangeFlag(const std_msgs::Int32ConstPtr& msg)
{
  ROS_INFO("Change flag callback: %d",msg->data);
  sc_.setChangeFlag(msg->data);
  sc_.update();
  publish();
}
}  // state_machine
