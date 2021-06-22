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
#include <std_msgs/Bool.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include <string>
#include <unordered_map>
#include <thread>

#include <Poco/Glob.h>

bool detect_contact = false;
std::string contact_with = "";

typedef std::shared_ptr<Poco::Glob> GlobPtr;

std::string target_model_name;
std::vector<GlobPtr> glob_filters;
std::unordered_map<std::string, bool> seen_models;
std::unordered_map<std::string, bool> except_models;

boost::shared_ptr<gazebo::msgs::Contacts const> msg(new gazebo::msgs::Contacts());
std::mutex msg_mtx;

std::string extract_model_name(const std::string &col)
{
    return col.substr(0, col.find_first_of(":"));
}

void check_glob(const std::string &obj)
{
    ROS_INFO("check name of new object: %s", obj.c_str());
    for (GlobPtr g : glob_filters) {
        if (g->match(obj)) {
            ROS_INFO("match");
            except_models[obj] = true;
            break;
        }
    }
}

void cb(ConstContactsPtr &_msg)
{
    std::lock_guard<std::mutex> lock(msg_mtx);
    msg = _msg;
}

void do_detect_contact()
{
    std::lock_guard<std::mutex> lock(msg_mtx);
    bool has_contact = false;
    std::string objname = "";
    for (unsigned int i = 0; i < msg->contact_size(); ++i) {
        const gazebo::msgs::Contact &contact = msg->contact(i);

        std::string obj1 = extract_model_name(contact.collision1());
        std::string obj2 = extract_model_name(contact.collision2());
        
        if (seen_models.find(obj1) == seen_models.end()) {
            check_glob(obj1);
            seen_models[obj1] = true;
        }
        if (seen_models.find(obj2) == seen_models.end()) {
            check_glob(obj2);
            seen_models[obj2] = true;
        }

        if (except_models.find(obj1) != except_models.end()) continue;
        if (except_models.find(obj2) != except_models.end()) continue;

        if (obj1 == target_model_name) {
            objname = obj2;
            has_contact = true;
        }
        if (obj2 == target_model_name) {
            objname = obj1;
            has_contact = true;
        }
    }
    detect_contact = has_contact;
    contact_with = objname;
}

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "undesired_contact_detector");
    ros::NodeHandle n("~");
    
    if (n.getParam("target_model_name", target_model_name)) {
        ROS_INFO("target_model_name is defined as: %s", target_model_name.c_str());
    } else {
        ROS_ERROR("Failed to get param 'target_model_name' use default 'hsrb'");
        target_model_name = "hsrb";
    }
    
    std::vector<std::string> except_models_list;
    if (n.getParam("except_model_names", except_models_list)) {
        for (std::string e : except_models_list) {
            ROS_INFO("except for model: %s", e.c_str());
            GlobPtr g;
            g.reset(new Poco::Glob(e));
            glob_filters.push_back(g);
        }
    }
    
    ros::Publisher pub = n.advertise<std_msgs::Bool>("detect", 1000);
    ros::Rate rate(10);
  
    // subscription to contacts info on gazebo transport
    gazebo::client::setup(argc, argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/physics/contacts", cb);

    while (ros::ok()) {
        do_detect_contact();
        if (detect_contact) {
            ROS_INFO("detect contact with %s", contact_with.c_str());
        }
        std_msgs::Bool msg;
        msg.data = detect_contact;
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
  
    gazebo::client::shutdown();
}
