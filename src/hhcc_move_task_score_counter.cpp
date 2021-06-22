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
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <string>

double per_call_and_come;
double collision_per_second;
double per_area_size;

double collision_score;
double call_and_come_score;
double area_size_score;

int exit_on_end;

std::string current_state;

ros::Time prev_detect_cb;

void cb_detect(const std_msgs::BoolConstPtr& detect)
{
    ros::Time now = ros::Time::now();
    double dt = fabs(now.toSec() - prev_detect_cb.toSec());
    if (detect->data && dt < 10) {
        double score = collision_per_second * dt;
        collision_score += score;
        ROS_WARN("[HHCC] Detect collision!");
    }
    prev_detect_cb = now;
}

void cb_state(const std_msgs::StringConstPtr& state)
{
    current_state = state->data;
}

void cb_count(const std_msgs::Float32Ptr& count)
{
    static double prev_call_and_come_score = 0.0;
    call_and_come_score = per_call_and_come * count->data;
    if (call_and_come_score != prev_call_and_come_score) {
        ROS_WARN("[HHCC] New success of call_and_come!");
        if (area_size_score < 100.0) {
            ROS_WARN("[HHCC] However, system has detected very low reached area size score (you probably haven't moved at all...). You cannot earn score if you just stay at the entrance.");
            call_and_come_score = 0.0;
        }
        prev_call_and_come_score = call_and_come_score;
    }
}

void cb_area(const std_msgs::Float32Ptr& area_size)
{
    if (current_state == "empty") {
        static double prev_area_size_score = 0.0;
        area_size_score = area_size->data * per_area_size;
        if (area_size_score != prev_area_size_score) {
            ROS_WARN("[HHCC] Detect increase of reached area size!");
            prev_area_size_score = area_size_score;
        }
    }
}

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "hhcc_move_task_score_counter");
    ros::NodeHandle n("~");

    if (n.getParam("per_call_and_come", per_call_and_come)) {
        ROS_INFO("per_call_and_come is defined as: %f", per_call_and_come);
    } else {
        per_call_and_come = 3000.0;
        ROS_ERROR("Failed to get param 'per_call_and_come' use default '%f'", per_call_and_come);
    }
    
    if (n.getParam("collision_per_second", collision_per_second)) {
        ROS_INFO("collision_per_second is defined as: %f", collision_per_second);
    } else {
        collision_per_second = -1.0;
        ROS_ERROR("Failed to get param 'collision_per_second' use default '%f'", collision_per_second);
    }
    
    if (n.getParam("per_area_size", per_area_size)) {
        ROS_INFO("per_area_size is defined as: %f", per_area_size);
    } else {
        per_area_size = 10.0;
        ROS_ERROR("Failed to get param 'per_area_size' use default '%f'", per_area_size);
    }
    
    if (n.getParam("exit_on_end", exit_on_end)) {
        ROS_INFO("exit_on_end is defined as: %i", exit_on_end);
    } else {
        exit_on_end = 1;
        ROS_ERROR("Failed to get param 'exit_on_end' use default '%i'", exit_on_end);
    }
    
    ros::Time::init();

    call_and_come_score = 0.0;
    collision_score = 0.0;
    area_size_score = 0.0;
    prev_detect_cb = ros::Time::now();

    ros::Publisher pub = n.advertise<std_msgs::Float32>("/score", 1000);
    ros::Rate rate(10);
    ros::Subscriber sub = n.subscribe("/undesired_contact_detector/detect", 1, cb_detect);
    ros::Subscriber sub2 = n.subscribe("/call_and_come_evaluator/count", 1, cb_count);
    ros::Subscriber sub3 = n.subscribe("/call_and_come_evaluator/state", 1, cb_state);
    ros::Subscriber sub4 = n.subscribe("/reached_region_calculator/area_size", 1, cb_area);

    double prev_score = 0.0;
    double score_offset = 0.0;
    int end_count = 0;
    while (ros::ok()) {
        std_msgs::Float32 msg;
        double score = collision_score + call_and_come_score + area_size_score + score_offset;
        if (score < 0.0) {
            score_offset += fabs(score);
            score = 0.0;
        }
        if (fabs(score - prev_score) > 0.1) {
            ROS_WARN("[HHCC] Your score has been changed to %i (%+i).", (int)score, (int)(score - prev_score));
            prev_score = score;
        }
        msg.data = score;
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
        if (current_state == "success" || current_state == "fail") {
            end_count++;
            if (end_count > 30 && exit_on_end != 0) {
                break;
            }
        }
    }
}
