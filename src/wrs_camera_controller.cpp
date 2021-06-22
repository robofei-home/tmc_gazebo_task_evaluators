/*
Copyright (c) 2019 TOYOTA MOTOR CORPORATION
Copyright (c) 2020 MID Academic Promotions, Inc.
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
    std::string robot_name;

    gazebo::msgs::Pose pose_room1;
    /*
    position {
    x: -1.4411584138870239
    y: -3.010014533996582
    z: 2.9286668300628662
    }
    orientation {
    x: -0.27499783039093018
    y: 0.27025458216667175
    z: 0.65808409452438354
    w: 0.646733283996582
    }
    */
    pose_room1.mutable_position()->set_x(-1.4411584138870239);
    pose_room1.mutable_position()->set_y(-3.010014533996582);
    pose_room1.mutable_position()->set_z(2.9286668300628662);
    pose_room1.mutable_orientation()->set_x(-0.27499783039093018);
    pose_room1.mutable_orientation()->set_y(0.27025458216667175);
    pose_room1.mutable_orientation()->set_z(0.65808409452438354);
    pose_room1.mutable_orientation()->set_w(0.646733283996582);

    gazebo::msgs::Pose pose_between_rooms;
    /*
    position {
    x: -0.0679052323102951
    y: -1.5629496574401855
    z: 2.90983510017395
    }
    orientation {
    x: -0.4410625696182251
    y: 0.43693691492080688
    z: 0.5569225549697876
    w: 0.55171316862106323
    }
    */
    pose_between_rooms.mutable_position()->set_x(-0.0679052323102951);
    pose_between_rooms.mutable_position()->set_y(-1.5629496574401855);
    pose_between_rooms.mutable_position()->set_z(2.90983510017395);
    pose_between_rooms.mutable_orientation()->set_x(-0.4410625696182251);
    pose_between_rooms.mutable_orientation()->set_y(0.43693691492080688);
    pose_between_rooms.mutable_orientation()->set_z(0.5569225549697876);
    pose_between_rooms.mutable_orientation()->set_w(0.55171316862106323);

    gazebo::msgs::Pose pose_room2;
    /*
    position {
    x: 0.21855369210243225
    y: -2.6537826061248779
    z: 2.1867930889129639
    }
    orientation {
    x: -0.12114474177360535
    y: 0.21551588177680969
    z: 0.4747948944568634
    w: 0.84465771913528442
    }
    */
    pose_room2.mutable_position()->set_x(0.21855369210243225);
    pose_room2.mutable_position()->set_y(-2.6537826061248779);
    pose_room2.mutable_position()->set_z(2.1867930889129639);
    pose_room2.mutable_orientation()->set_x(-0.12114474177360535);
    pose_room2.mutable_orientation()->set_y(0.21551588177680969);
    pose_room2.mutable_orientation()->set_z(0.4747948944568634);
    pose_room2.mutable_orientation()->set_w(0.84465771913528442);

    // initialize ROS node
    ros::init(argc, argv, "wrs_camera_controller");
    ros::NodeHandle n("~");

    if (n.getParam("robot_name", robot_name)) {
        ROS_INFO("box_name is defined as: %s", robot_name.c_str());
    } else {
        robot_name = "hsrb";
        ROS_ERROR("Failed to get param 'robot_name' use default '%s'", robot_name.c_str());
    }

    ros::Rate rate(1);

    ros::service::waitForService("/gazebo/get_model_state");
    ros::ServiceClient getModelState = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState model_state;
    model_state.request.model_name = robot_name;

    // subscription to contacts info on gazebo transport
    gazebo::client::setup(argc, argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");

    while (ros::ok()) {
        getModelState.call(model_state);
        double x = model_state.response.pose.position.x;
        double y = model_state.response.pose.position.y;
        if (fabs(x) < 1.0 && y < -0.8) {
            pub->Publish(pose_between_rooms);
        } else if (x < 0.0) {
            pub->Publish(pose_room1);
        } else {
            pub->Publish(pose_room2);
        }
        ros::spinOnce();
        rate.sleep();
    }

    gazebo::client::shutdown();
}
