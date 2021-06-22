/*
Copyright (c) 2020 TOYOTA MOTOR CORPORATION
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
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>

#include <iostream>
#include <string>
#include <map>
#include <boost/bind.hpp>

#include <Poco/RegularExpression.h>

using Poco::RegularExpression;
RegularExpression task2_re("task2_[0-9]+_ycb_[0-9]+_");

double task1_per_delivery;
double task1_per_correct_category;
double task1_per_correct_orientation;

double task1_start_time;
double task1_delivery_score;
double task1_category_score;
double task1_orientation_score;
double task1_time_bonus;
double task1_draweropen_bonus;

double task2a_score;
bool task2a_collision_detected;

double task2_start_time;
double task2_score;
double task2_time_bonus;
std::string task2_target;
std::string task2_target_person;
std::map<std::string, bool> task2_checked_objects;

double overall_time_bonus;

int seed;

ros::ServiceClient getWorldProperties;
ros::ServiceClient getModelState;
ros::Publisher pubmsg;

ros::Time prev_detect_cb;

void get_objects_in_shelf(std::vector<std::string> &objects)
{
    gazebo_msgs::GetWorldProperties world_properties;
    getWorldProperties.call(world_properties);
    objects.resize(0);
    for (auto name: world_properties.response.model_names) {
        gazebo_msgs::GetModelState model_state;
        model_state.request.model_name = name;
        model_state.request.relative_entity_name = "wrc_bookshelf::link";
        getModelState.call(model_state);
        auto p = model_state.response.pose;
        if (name != "wrc_bookshelf" && 
            fabs(p.position.x) < 0.8 / 2 &&
            fabs(p.position.y) < 0.28 / 2 ) {
            objects.push_back(name);
        }
    }
}

void get_objects_in_humanfront(std::vector<std::string> &objects)
{
    gazebo_msgs::GetWorldProperties world_properties;
    getWorldProperties.call(world_properties);
    objects.resize(0);
    for (auto name: world_properties.response.model_names) {
        gazebo_msgs::GetModelState model_state;
        model_state.request.model_name = name;
        model_state.request.relative_entity_name = "wrc_frame::link";
        getModelState.call(model_state);
        auto p = model_state.response.pose;
        if (name.find("task2_") == 0 &&
            fabs(p.position.x - 1.5) < 3.0 / 2 &&
            fabs(p.position.y - 1.2) < 1.6 / 2 ) {
            objects.push_back(name);
        }
    }
}

void get_task1_objects_on_tables(std::vector<std::string> &objects)
{
    gazebo_msgs::GetWorldProperties world_properties;
    getWorldProperties.call(world_properties);
    objects.resize(0);
    for (auto name: world_properties.response.model_names) {
        gazebo_msgs::GetModelState model_state;
        model_state.request.model_name = name;
        model_state.request.relative_entity_name = "wrc_frame::link";
        getModelState.call(model_state);
        auto p = model_state.response.pose;
        if (name.find("task1_") == 0 &&
            fabs(p.position.x + 0.7) < 1.4 / 2 &&
            fabs(p.position.y) < 4.0 / 2 ) {
            objects.push_back(name);
        }
    }
}

void cb_detect(const std_msgs::BoolConstPtr& detect)
{
    static bool first_time = true;
    if (detect->data) {
        if (first_time) {
            task2a_collision_detected = true;
            task2a_score = 0.0;
            first_time = false;
            ROS_WARN("[WRS] You hit the object on the floor!");
        }
    }
}

std::string random_object_in_shelf()
{
    std::vector<std::string> objects_in_shelf;
    get_objects_in_shelf(objects_in_shelf);
    std::string obj = objects_in_shelf[rand() % objects_in_shelf.size()];
    task2_re.subst(obj, "");
    return obj;
}

std::string random_person()
{
    std::vector<std::string> persons;
    persons.push_back("left");
    persons.push_back("right");
    std::string obj = persons[rand() % persons.size()];
    return obj;
}

void count_task2_score()
{
    std::vector<std::string> objects_in_humanfront;
    get_objects_in_humanfront(objects_in_humanfront);
    bool correct = false;
    bool delivered = false;
    for (auto obj: objects_in_humanfront) {
        if (task2_checked_objects.count(obj) == 0) {
            task2_checked_objects[obj] = true;
            task2_re.subst(obj, "");
            if (task2_target == obj) {
                correct = true;
            }
            delivered = true;
        }
    }
    if (correct) {
        // bonus score
        ROS_WARN("[WRS] You have delivered object and the object is correct!");
        task2_score += 100.0;
    } else if (delivered) {
        // regular score
        ROS_WARN("[WRS] You have delivered object and the object is incorrect!");
        task2_score += 40.0;
    } else {
        ROS_WARN("[WRS] No object delivered...");
    }
}

void cb_hsrb_in_room2(const std_msgs::Int16::ConstPtr& count)
{
    static bool first_time = true;
    if (count->data > 0) {
        if (first_time) {
            first_time = false;
            if (task2a_collision_detected == false) {
                task2a_score = 100.0;
            } else {
                task2a_score = 0.0;
            }
            ROS_WARN("[WRS] Entered room 2!");
            task2_start_time = ros::Time::now().toSec();

            // calculate time bonus when all the objects are cleaned
            std::vector<std::string> remaining_task1_objects;
            get_task1_objects_on_tables(remaining_task1_objects);
            ROS_WARN("[WRS] Number of remaining task1 objects %i.", (int)remaining_task1_objects.size());
            if (remaining_task1_objects.size() == 0) {
                double task1_duration = task2_start_time - task1_start_time;
                if (task1_duration <= 15*60+15) {
                    task1_time_bonus = 50;
                    ROS_WARN("[WRS] Congratulations! You have finished task1 within given time.");
                }
            }

            // publish first request
            task2_target = random_object_in_shelf();
            task2_target_person = random_person();
            std_msgs::String msg;
            msg.data = task2_target + " to person " + task2_target_person;
            pubmsg.publish(msg);
            ROS_WARN("[WRS] Asked to take %s", msg.data.c_str());
        }
    }
}

void cb_hsrb_in_humanfront(const std::string place, const std_msgs::Int16::ConstPtr& count)
{
    static int times = 0;
    static std::map<std::string, int> places;

    int prevdata = 0;
    int total_count = 0;
    for(auto itr = places.begin(); itr != places.end(); ++itr) {
        if (itr->second > 0) {
            prevdata = itr->second;
        }
    }
    if (prevdata != count->data && count->data > 0) {
        if (times == 0 && place == task2_target_person) {
            times = 1;
            // count score
            ROS_WARN("[WRS] Delivered object to human!");
            count_task2_score();
            double task2_end_time = ros::Time::now().toSec();
            double task2_duration = task2_end_time - task2_start_time;
            std::vector<std::string> remaining_task1_objects;
            get_task1_objects_on_tables(remaining_task1_objects);
            std::vector<std::string> objects_in_humanfront;
            get_objects_in_humanfront(objects_in_humanfront);
            if (task2_duration <= 5*60.0+15.0 && objects_in_humanfront.size() == 1) {
                task2_time_bonus = 50.0;
                ROS_WARN("[WRS] Congratulations! You have finished task2 within given time.");
            }
            double overall_time = task2_end_time - task1_start_time;
            ROS_WARN("[WRS] Overall time %f [seconds].", overall_time);
            double remaining_time = 20*60.0 - overall_time;
            double remaining_time_in_minute = floor(remaining_time / 60.0);
            ROS_WARN("[WRS] Remaining time %.0f [minute].", remaining_time_in_minute);
            if (remaining_time_in_minute > 0.0) {
                if (remaining_task1_objects.size() == 0 && objects_in_humanfront.size() == 1) {
                    ROS_WARN("[WRS] Congratulations! You have finished all the tasks within given time.");
                    overall_time_bonus = 20.0 * remaining_time_in_minute;
                } else {
                    ROS_WARN("[WRS] No remaining time bonus due to unfinished tasks.");
                }
            }
            std_msgs::String msg;
            msg.data = "done";
            pubmsg.publish(msg);
            ROS_WARN("[WRS] All done!");
        }
    }
    places[place] = count->data;
}

void cb_count_delivery(const std::string place, const std_msgs::Int16::ConstPtr& count)
{
    static double prev_delivery_score = 0.0;
    static std::map<std::string, int> places;

    places[place] = count->data;
    int total_count = 0;
    for(auto itr = places.begin(); itr != places.end(); ++itr) {
        total_count += itr->second;
    }
    task1_delivery_score = task1_per_delivery * (double)total_count;
    if (task1_delivery_score != prev_delivery_score) {
        ROS_WARN("[WRS] Found new delivered object!");
        prev_delivery_score = task1_delivery_score;
    }
}

void cb_count_correct_category(const std::string place, const std_msgs::Int16::ConstPtr& count)
{
    static double prev_category_score = 0.0;
    static std::map<std::string, int> places;

    places[place] = count->data;
    int total_count = 0;
    for(auto itr = places.begin(); itr != places.end(); ++itr) {
        total_count += itr->second;
    }
    task1_category_score = task1_per_correct_category * (double)total_count;
    if (task1_category_score != prev_category_score) {
        ROS_WARN("[WRS] Found new delivered object with correct category!");
        prev_category_score = task1_category_score;
    }
}

void cb_count_draweropen(const std_msgs::Int16::ConstPtr& count)
{
    static double prev_draweropen_score = 0.0;

    if (count->data >= 3) {
        task1_draweropen_bonus = 50.0;
    }
    if (task1_draweropen_bonus != prev_draweropen_score) {
        ROS_WARN("[WRS] Opened all the drawers!");
        prev_draweropen_score = task1_draweropen_bonus;
    }
}

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "wrc_score_counter");
    ros::NodeHandle n("~");

    if (n.getParam("task1_per_delivery", task1_per_delivery)) {
        ROS_INFO("task1_per_delivery is defined as: %f", task1_per_delivery);
    } else {
        task1_per_delivery = 10.0;
        ROS_INFO("Failed to get param 'task1_per_delivery' use default '%f'", task1_per_delivery);
    }

    if (n.getParam("task1_per_correct_category", task1_per_correct_category)) {
        ROS_INFO("task1_per_correct_category is defined as: %f", task1_per_correct_category);
    } else {
        task1_per_correct_category = 10.0;
        ROS_INFO("Failed to get param 'task1_per_correct_category' use default '%f'", task1_per_correct_category);
    }
    
    if (n.getParam("seed", seed)) {
        ROS_INFO("seed is defined as: %i", seed);
    } else {
        seed = 0;
        ROS_INFO("Failed to get param 'seed' use default '%i'", seed);
    }

    srand(seed);

    ros::service::waitForService("/gazebo/get_world_properties");
    getWorldProperties = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

    ros::service::waitForService("/gazebo/get_model_state");
    getModelState = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    ros::Time::init();

    //collision_score = 0.0;
    prev_detect_cb = ros::Time::now();

    ros::Publisher pub = n.advertise<std_msgs::Float32>("/score", 1000, true);
    pubmsg = n.advertise<std_msgs::String>("/message", 1000, true);
    ros::Rate rate(10);

    ros::Subscriber sub_drawerleft_any = n.subscribe<std_msgs::Int16>("/any_in_drawerleft_detector/count", 1, boost::bind(&cb_count_delivery, "drawerleft", _1));
    ros::Subscriber sub_drawerleft_cat = n.subscribe<std_msgs::Int16>("/shapeitems_in_drawerleft_detector/count", 1, boost::bind(&cb_count_correct_category, "drawerleft", _1));

    ros::Subscriber sub_drawertop_any = n.subscribe<std_msgs::Int16>("/any_in_drawertop_detector/count", 1, boost::bind(&cb_count_delivery, "drawertop", _1));
    ros::Subscriber sub_drawertop_cat = n.subscribe<std_msgs::Int16>("/tools_in_drawertop_detector/count", 1, boost::bind(&cb_count_correct_category, "drawertop", _1));

    ros::Subscriber sub_drawerbottom_any = n.subscribe<std_msgs::Int16>("/any_in_drawerbottom_detector/count", 1, boost::bind(&cb_count_delivery, "drawerbottom", _1));
    ros::Subscriber sub_drawerbottom_cat = n.subscribe<std_msgs::Int16>("/tools_in_drawerbottom_detector/count", 1, boost::bind(&cb_count_correct_category, "drawerbottom", _1));

    ros::Subscriber sub_drawer_open = n.subscribe<std_msgs::Int16>("/drawer_in_drawerfront_detector/count", 1, cb_count_draweropen);

    ros::Subscriber sub_containera_any = n.subscribe<std_msgs::Int16>("/any_in_containera_detector/count", 1, boost::bind(&cb_count_delivery, "containera", _1));
    ros::Subscriber sub_containera_cat = n.subscribe<std_msgs::Int16>("/kitchenitems_in_containera_detector/count", 1, boost::bind(&cb_count_correct_category, "containera", _1));

    ros::Subscriber sub_containerb_any = n.subscribe<std_msgs::Int16>("/any_in_containerb_detector/count", 1, boost::bind(&cb_count_delivery, "containerb", _1));
    ros::Subscriber sub_containerb_largemarker = n.subscribe<std_msgs::Int16>("/largemarker_in_containerb_detector/count", 1, boost::bind(&cb_count_correct_category, "containerb_largemarker", _1));
    ros::Subscriber sub_containerb_fork = n.subscribe<std_msgs::Int16>("/fork_in_containerb_detector/count", 1, boost::bind(&cb_count_correct_category, "containerb_fork", _1));
    ros::Subscriber sub_containerb_spoon = n.subscribe<std_msgs::Int16>("/spoon_in_containerb_detector/count", 1, boost::bind(&cb_count_correct_category, "containerb_spoon", _1));

    ros::Subscriber sub_traya_any = n.subscribe<std_msgs::Int16>("/any_in_traya_detector/count", 1, boost::bind(&cb_count_delivery, "traya", _1));
    ros::Subscriber sub_traya_cat = n.subscribe<std_msgs::Int16>("/foods_in_traya_detector/count", 1, boost::bind(&cb_count_correct_category, "traya", _1));

    ros::Subscriber sub_trayb_any = n.subscribe<std_msgs::Int16>("/any_in_trayb_detector/count", 1, boost::bind(&cb_count_delivery, "trayb", _1));
    ros::Subscriber sub_trayb_cat = n.subscribe<std_msgs::Int16>("/foods_in_trayb_detector/count", 1, boost::bind(&cb_count_correct_category, "trayb", _1));

    ros::Subscriber sub_bina_any = n.subscribe<std_msgs::Int16>("/any_in_bina_detector/count", 1, boost::bind(&cb_count_delivery, "bina", _1));
    ros::Subscriber sub_bina_cat = n.subscribe<std_msgs::Int16>("/taskitems_in_bina_detector/count", 1, boost::bind(&cb_count_correct_category, "bina", _1));

    ros::Subscriber sub_binb_any = n.subscribe<std_msgs::Int16>("/any_in_binb_detector/count", 1, boost::bind(&cb_count_delivery, "binb", _1));
    ros::Subscriber sub_binb_cat = n.subscribe<std_msgs::Int16>("/taskitems_in_binb_detector/count", 1, boost::bind(&cb_count_correct_category, "binb", _1));

    ros::Subscriber sub = n.subscribe("/undesired_contact_detector/detect", 1, cb_detect);
    ros::Subscriber sub_hsrb_in_room2 = n.subscribe<std_msgs::Int16>("/hsrb_in_room2_detector/count", 1, cb_hsrb_in_room2);
    ros::Subscriber sub_hsrb_in_humanleftfront = n.subscribe<std_msgs::Int16>("/hsrb_in_humanleftfront_detector/count", 1, boost::bind(&cb_hsrb_in_humanfront, "left", _1));
    ros::Subscriber sub_hsrb_in_humanrightfront = n.subscribe<std_msgs::Int16>("/hsrb_in_humanrightfront_detector/count", 1, boost::bind(&cb_hsrb_in_humanfront, "right", _1));

    task1_delivery_score = 0.0;
    task1_category_score = 0.0;
    task1_time_bonus = 0.0;
    task1_draweropen_bonus = 0.0;
    task2a_score = 0.0;
    task2a_collision_detected = false;
    task2_score = 0.0;
    task2_time_bonus = 0.0;
    overall_time_bonus = 0.0;

    task1_start_time = 0.0;

    double prev_score = 0.0;
    while (ros::ok()) {
        std_msgs::Float32 msg;
        double score = task1_delivery_score + task1_category_score + task1_time_bonus + task1_draweropen_bonus + task2a_score + task2_score + task2_time_bonus + overall_time_bonus;
        if (fabs(score - prev_score) > 0.1) {
            ROS_WARN("[WRS] Your score has been changed to %i (%+i).", (int)score, (int)(score - prev_score));
            prev_score = score;
        }
        msg.data = score;
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}
