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
      x: -2.9806835651397705
      y: 1.5702841281890869
      z: 7.8411393165588379
    }
    orientation {
      x: -0.0015904847532510757
      y: 0.58484375476837158
      z: 0.0022059017792344093
      w: 0.81114143133163452
    }
    */
    pose_standard.mutable_position()->set_x(-2.98068356513977085);
    pose_standard.mutable_position()->set_y(1.5702841281890869);
    pose_standard.mutable_position()->set_z(7.8411393165588379);
    pose_standard.mutable_orientation()->set_x(-0.0015904847532510757);
    pose_standard.mutable_orientation()->set_y(0.58484375476837158);
    pose_standard.mutable_orientation()->set_z(0.0022059017792344093);
    pose_standard.mutable_orientation()->set_w(0.81114143133163452);

    gazebo::msgs::Pose pose_wagon;
    /*
    position {
      x: 1.2904144525527954
      y: -0.45991659164428711
      z: 4.2858314514160156
    }
    orientation {
      x: 0.00083756691310554743
      y: 0.65387088060379028
      z: -0.00096916290931403637
      w: 0.75660508871078491
    }
    */
    pose_wagon.mutable_position()->set_x(1.2904144525527954);
    pose_wagon.mutable_position()->set_y(-0.45991659164428711);
    pose_wagon.mutable_position()->set_z(4.2858314514160156);
    pose_wagon.mutable_orientation()->set_x(0.00083756691310554743);
    pose_wagon.mutable_orientation()->set_y(0.65387088060379028);
    pose_wagon.mutable_orientation()->set_z(-0.00096916290931403637);
    pose_wagon.mutable_orientation()->set_w(0.75660508871078491);

    gazebo::msgs::Pose pose_dishwasher;
    /*
    position {
      x: 1.8237259387969971
      y: 2.8617923259735107
      z: 3.0747940540313721
    }
    orientation {
      x: 0.00083756691310554743
      y: 0.65387088060379028
      z: -0.00096916290931403637
      w: 0.75660508871078491
    }
    */
    pose_dishwasher.mutable_position()->set_x(1.8237259387969971);
    pose_dishwasher.mutable_position()->set_y(2.8617923259735107);
    pose_dishwasher.mutable_position()->set_z(3.0747940540313721);
    pose_dishwasher.mutable_orientation()->set_x(0.00083756691310554743);
    pose_dishwasher.mutable_orientation()->set_y(0.65387088060379028);
    pose_dishwasher.mutable_orientation()->set_z(-0.00096916290931403637);
    pose_dishwasher.mutable_orientation()->set_w(0.75660508871078491);

    // initialize ROS node
    ros::init(argc, argv, "hhcc_tidy_camera_controller");
    ros::NodeHandle n("~");
    
    ros::Rate rate(1);
  
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
        if (x > 1.0 && y < 0.5) {
            pub->Publish(pose_wagon);
        } else if (x > 1.0 && y > 1.5) {
            pub->Publish(pose_dishwasher);
        } else {
            pub->Publish(pose_standard);
        }
        ros::spinOnce();
        rate.sleep();
    }
  
    gazebo::client::shutdown();
}
