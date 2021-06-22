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

double per_cleaned_block;
double per_cleaned_toy;
double per_cleaned_dish;
double per_cleaned_stationary;
double collision_per_second;

double collision_score;
double clean_score;
double clean_score2;
double clean_score3;
double clean_score4;
double clean_score5;
double clean_score6;
double clean_score7;

ros::Time prev_detect_cb;

void cb_detect(const std_msgs::BoolConstPtr& detect)
{
    ros::Time now = ros::Time::now();
    if (detect->data) {
        double score = collision_per_second * (now.toSec() - prev_detect_cb.toSec());
        collision_score += score;
        ROS_WARN("[HHCC] Detect collision!");
    }
    prev_detect_cb = now;
}

void cb_count_block(const std_msgs::Int16Ptr& count)
{
    static double prev_clean_score = 0.0;
    clean_score = per_cleaned_block * (double)count->data;
    if (clean_score != prev_clean_score) {
        ROS_WARN("[HHCC] Found new cleaned object!");
        prev_clean_score = clean_score;
    }
}

void cb_count_toy(const std_msgs::Int16Ptr& count)
{
    static double prev_clean_score = 0.0;
    clean_score2 = per_cleaned_toy * (double)count->data;
    if (clean_score2 != prev_clean_score) {
        ROS_WARN("[HHCC] Found new cleaned object!");
        prev_clean_score = clean_score2;
    }
}

void cb_count_dish(const std_msgs::Int16Ptr& count)
{
    static double prev_clean_score = 0.0;
    clean_score3 = per_cleaned_dish * (double)count->data;
    if (clean_score3 != prev_clean_score) {
        ROS_WARN("[HHCC] Found new cleaned object!");
        prev_clean_score = clean_score3;
    }
}

void cb_count_teacup(const std_msgs::Int16Ptr& count)
{
    static double prev_clean_score = 0.0;
    clean_score4 = per_cleaned_dish * (double)count->data;
    if (clean_score4 != prev_clean_score) {
        ROS_WARN("[HHCC] Found new cleaned object!");
        prev_clean_score = clean_score4;
    }
}

void cb_count_cellphone(const std_msgs::Float32Ptr& count)
{
    static double prev_clean_score = 0.0;
    clean_score5 = per_cleaned_stationary * count->data;
    if (fabs(clean_score5 - prev_clean_score) > 0.1) {
        ROS_WARN("[HHCC] Found new cleaned object!");
        prev_clean_score = clean_score5;
    }
}

void cb_count_remocon(const std_msgs::Float32Ptr& count)
{
    static double prev_clean_score = 0.0;
    clean_score6 = per_cleaned_stationary * count->data;
    if (fabs(clean_score6 - prev_clean_score) > 0.1) {
        ROS_WARN("[HHCC] Found new cleaned object!");
        prev_clean_score = clean_score6;
    }
}

void cb_count_stapler(const std_msgs::Float32Ptr& count)
{
    static double prev_clean_score = 0.0;
    clean_score7 = per_cleaned_stationary * count->data;
    if (fabs(clean_score7 - prev_clean_score) > 0.1) {
        ROS_WARN("[HHCC] Found new cleaned object!");
        prev_clean_score = clean_score7;
    }
}

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "hhcc_score_counter");
    ros::NodeHandle n("~");

    if (n.getParam("per_cleaned_block", per_cleaned_block)) {
        ROS_INFO("per_cleaned_block is defined as: %f", per_cleaned_block);
    } else {
        per_cleaned_block = 50.0;
        ROS_ERROR("Failed to get param 'per_cleaned_block' use default '%f'", per_cleaned_block);
    }
    
    if (n.getParam("per_cleaned_toy", per_cleaned_toy)) {
        ROS_INFO("per_cleaned_toy is defined as: %f", per_cleaned_toy);
    } else {
        per_cleaned_toy = 100.0;
        ROS_ERROR("Failed to get param 'per_cleaned_toy' use default '%f'", per_cleaned_toy);
    }
    
    if (n.getParam("per_cleaned_dish", per_cleaned_dish)) {
        ROS_INFO("per_cleaned_dish is defined as: %f", per_cleaned_dish);
    } else {
        per_cleaned_dish = 250.0;
        ROS_ERROR("Failed to get param 'per_cleaned_dish' use default '%f'", per_cleaned_dish);
    }
    
    if (n.getParam("per_cleaned_stationary", per_cleaned_stationary)) {
        ROS_INFO("per_cleaned_stationary is defined as: %f", per_cleaned_stationary);
    } else {
        per_cleaned_stationary = 500.0;
        ROS_ERROR("Failed to get param 'per_cleaned_stationary' use default '%f'", per_cleaned_stationary);
    }
    
    if (n.getParam("collision_per_second", collision_per_second)) {
      ROS_INFO("collision_per_second is defined as: %f", collision_per_second);
    } else {
      collision_per_second = -1.0;
      ROS_ERROR("Failed to get param 'collision_per_second' use default '%f'", collision_per_second);
    }
    
    ros::Time::init();

    clean_score = 0.0;
    collision_score = 0.0;
    prev_detect_cb = ros::Time::now();

    ros::Publisher pub = n.advertise<std_msgs::Float32>("/score", 1000);
    ros::Publisher pubmsg = n.advertise<std_msgs::String>("message", 1000);
    ros::Rate rate(10);
    ros::Subscriber sub = n.subscribe("/undesired_contact_detector/detect", 1, cb_detect);
    ros::Subscriber sub2 = n.subscribe("/object_in_box_detector/count", 1, cb_count_block);
    ros::Subscriber sub3 = n.subscribe("/toys_in_trofase_detector/count", 1, cb_count_toy);
    ros::Subscriber sub4 = n.subscribe("/dishes_in_sink_detector/count", 1, cb_count_dish);
    ros::Subscriber sub5 = n.subscribe("/teacups_in_sink_detector/count", 1, cb_count_teacup);
    ros::Subscriber sub6 = n.subscribe("/cellphone_in_wagon_detector/similarity", 1, cb_count_cellphone);
    ros::Subscriber sub7 = n.subscribe("/remocon_in_wagon_detector/similarity", 1, cb_count_remocon);
    ros::Subscriber sub8 = n.subscribe("/stapler_in_wagon_detector/similarity", 1, cb_count_stapler);

    double prev_score = 0.0;
    while (ros::ok()) {
        std_msgs::Float32 msg;
        double score = collision_score + clean_score + clean_score2 + clean_score3 + clean_score4 + clean_score5 + clean_score6 + clean_score7;
        if (fabs(score - prev_score) > 0.1) {
            ROS_WARN("[HHCC] Your score has been changed to %i (%+i).", (int)score, (int)(score - prev_score));
          prev_score = score;
        }
        msg.data = score;
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}
