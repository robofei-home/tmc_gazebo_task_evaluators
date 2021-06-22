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
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetModelState.h>

#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <cmath>

std::string target_model_name;
double radius = 1.0;
double radius2 = 1.0;
const double max_area_size = 50.0;  // max area size is 50.0[meter] square
double step_size = 0.1;
int step_count = 0;
bool first_loop = true;
double area_offset = 0.0;

Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> reached_region;

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "reached_region_calculator");
    ros::NodeHandle n("~");
    
    if (n.getParam("target_model_name", target_model_name)) {
        ROS_INFO("target_model_name is defined as: %s", target_model_name.c_str());
    } else {
        ROS_ERROR("Failed to get param 'target_model_name' use default 'hsrb'");
        target_model_name = "hsrb";
    }
    
    if (n.getParam("radius", radius)) {
        ROS_INFO("radius is defined as: %f", radius);
    } else {
        ROS_ERROR("Failed to get param 'radius' use default %f", radius);
    }
    radius2 = radius * radius;
    
    if (n.getParam("step_size", step_size)) {
        ROS_INFO("step size is defined as: %f", step_size);
    } else {
        ROS_ERROR("Failed to get param 'step_size' use default %f", step_size);
    }

    // initialise reached region map
    step_count = (int)round(max_area_size * 2.0 / step_size);
    reached_region.resize(step_count, step_count);
    for (int y = 0; y < step_count; y++) {
        for (int x = 0; x < step_count; x++) {
            reached_region(x, y) = false;
        }
    }

    ros::Publisher pub = n.advertise<std_msgs::Float32>("area_size", 1000);
    ros::Rate rate(10);
  
    ros::service::waitForService("/gazebo/get_model_state");
    ros::ServiceClient getModelState = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState model_state;
    model_state.request.model_name = target_model_name;

    while (ros::ok()) {
        getModelState.call(model_state);
        double x = model_state.response.pose.position.x;
        double y = model_state.response.pose.position.y;
        // fill the reached region map with circle
        for (double dy = -radius; dy <= radius; dy += step_size) {
            for (double dx = -radius; dx <= radius; dx += step_size) {
                if (dx * dx + dy * dy <= radius2) {
                    int ix = (int)round((x + dx + max_area_size) / step_size);
                    if (ix < 0) ix = 0;
                    if (ix >= step_count) ix = step_count - 1;
                    int iy = (int)round((y + dy + max_area_size) / step_size);
                    if (iy < 0) iy = 0;
                    if (iy >= step_count) iy = step_count - 1;
                    reached_region(ix, iy) = true;
                }
            }
        }
        int count = 0;
        for (int y = 0; y < step_count; y++) {
            for (int x = 0; x < step_count; x++) {
                if (reached_region(x, y) == true) {
                    count++;
                }
            }
        }
        double area_size = ((double)count) * step_size * step_size - area_offset;
        if (first_loop) {
            area_offset = area_size;
            first_loop = false;
        } else {
            std_msgs::Float32 msg;
            msg.data = area_size;
            pub.publish(msg);
        }
        ros::spinOnce();
        rate.sleep();
    }
}
