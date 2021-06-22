/*
Copyright (c) 2019 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetModelState.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include <string>

int main(int argc, char **argv)
{
    gazebo::msgs::Pose pose_standard;
    /*
    position {
      x: 19.688961029052734
      y: -6.0323991775512695
      z: 7.5681877136230469
    }
    orientation {
      x: -0.2802603542804718
      y: 0.12004674226045609
      z: 0.87545603513717651
      w: 0.37499290704727173
    }
    */
    pose_standard.mutable_position()->set_x(0.0);
    pose_standard.mutable_position()->set_y(0.0);
    pose_standard.mutable_position()->set_z(7.5681877136230469);
    pose_standard.mutable_orientation()->set_x(-0.2802603542804718);
    pose_standard.mutable_orientation()->set_y(0.12004674226045609);
    pose_standard.mutable_orientation()->set_z(0.87545603513717651);
    pose_standard.mutable_orientation()->set_w(0.37499290704727173);
    const double x_offset = 19.688961029052734 - 14.0;
    const double y_offset = -6.0323991775512695;

    // initialize ROS node
    ros::init(argc, argv, "hhcc_move_camera_controller");
    ros::NodeHandle n("~");
    
    ros::Rate rate(10);
    
    ros::service::waitForService("/gazebo/get_model_state");
    ros::ServiceClient getModelState = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState model_state;
    model_state.request.model_name = "hsrb";

    // subscription to contacts info on gazebo transport
    gazebo::client::setup(argc, argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");

    while (ros::ok()) {
        getModelState.call(model_state);
        double x = model_state.response.pose.position.x;
        double y = model_state.response.pose.position.y;
        pose_standard.mutable_position()->set_x(x + x_offset);
        pose_standard.mutable_position()->set_y(y + y_offset);
        pub->Publish(pose_standard);
        ros::spinOnce();
        rate.sleep();
    }
  
    gazebo::client::shutdown();
}
