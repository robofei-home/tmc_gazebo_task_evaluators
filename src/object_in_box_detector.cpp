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
#include <tf/tf.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>

#include <iostream>
#include <unordered_map>
#include <string>
#include <sstream>
#include <map>

#include <Poco/Glob.h>

typedef std::shared_ptr<Poco::Glob> GlobPtr;

std::string box_name;
std::vector<double> box_size;
std::vector<double> box_pose;

tf::Vector3 object_axes;
tf::Vector3 target_axes;
bool use_object_axes;
int both_direction;
double cosine_similarity;

tf::Vector3 target_position;
bool use_object_position;
double max_distance;

std::vector<GlobPtr> glob_filters;
std::unordered_map<std::string, bool> seen_models;
std::unordered_map<std::string, bool> target_objects;
std::vector<std::string> objects_list;
std::unordered_map<std::string, geometry_msgs::Pose> object_pose;

void check_glob(const std::string &obj)
{
    ROS_DEBUG("check name of new object: %s", obj.c_str());
    for (GlobPtr g : glob_filters) {
        if (g->match(obj)) {
            ROS_DEBUG("match");
            target_objects[obj] = true;
            objects_list.push_back(obj);
            break;
        }
    }
}

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "object_in_box_detector");
    ros::NodeHandle n("~");
    
    if (n.getParam("box_name", box_name)) {
        ROS_INFO("box_name is defined as: %s", box_name.c_str());
    } else {
        box_name = "box";
        ROS_ERROR("Failed to get param 'box_name' use default '%s'", box_name.c_str());
    }

    if (n.getParam("box_size", box_size)) {
        ROS_INFO("box_size is defined as: %f, %f, %f", box_size[0], box_size[1], box_size[2]);
    } else {
        box_size = {1, 1, 1};
        ROS_ERROR("Failed to get param 'box_size' use default '1 1 1'");
    }
    
    if (n.getParam("box_pose", box_pose)) {
        ROS_INFO("box_pose is defined as: %f, %f, %f", box_pose[0], box_pose[1], box_pose[2]);
    } else {
        box_pose = {0, 0, 0};
        ROS_ERROR("Failed to get param 'box_pose' use default '0 0 0'");
    }
    
    std::vector<double> tmp;
    if (n.getParam("target_axes", tmp)) {
        use_object_axes = true;
        target_axes = tf::Vector3(tmp[0], tmp[1], tmp[2]);
        target_axes.normalize();
        ROS_INFO("target_axes is defined as: %f, %f, %f", target_axes.getX(), target_axes.getY(), target_axes.getZ());
    } else {
        use_object_axes = false;
        ROS_INFO("Failed to get param 'target_axes' will ignore orientation");
    }

    if (n.getParam("object_axes", tmp)) {
        object_axes = tf::Vector3(tmp[0], tmp[1], tmp[2]);
        object_axes.normalize();
        ROS_INFO("object_axes is defined as: %f, %f, %f", object_axes.getX(), object_axes.getY(), object_axes.getZ());
    } else {
        if (use_object_axes)  {
            object_axes = tf::Vector3(0, 0, 1);
            ROS_INFO("Failed to get param 'object_axes' use default Z-axis");
        }
    }
    
    if (n.getParam("both_direction", both_direction)) {
        ROS_INFO("both_direction is defined as: %i", both_direction);
    } else {
        if (use_object_axes)  {
            both_direction = 1;
            ROS_ERROR("Failed to get param 'both_direction' use default %i", both_direction);
        }
    }

    double allow_degree = 30;
    if (n.getParam("allow_degree", allow_degree)) {
        ROS_INFO("allow_degree is defined as: %f", allow_degree);
    } else {
        if (use_object_axes)  {
            ROS_ERROR("Failed to get param 'allow_degree' use default %f", allow_degree);
        }
    }
    cosine_similarity = allow_degree * 2 * M_PI / 360.0;
    
    if (n.getParam("target_position", tmp)) {
        use_object_position = true;
        target_position = tf::Vector3(tmp[0], tmp[1], tmp[2]);
        ROS_INFO("target_position is defined as: %f, %f, %f", target_position.getX(), target_position.getY(), target_position.getZ());
    } else {
        use_object_position = false;
        ROS_INFO("Failed to get param 'target_position' will ignore position");
    }

    max_distance = 0.2;
    if (n.getParam("max_distance", max_distance)) {
        ROS_INFO("max_distance is defined as: %f", max_distance);
    } else {
        if (use_object_position)  {
            ROS_ERROR("Failed to get param 'max_distance' use default %f", max_distance);
        }
    }

    std::vector<std::string> filter_list;
    if (n.getParam("object_names", filter_list)) {
        for (std::string f : filter_list) {
            ROS_INFO("target object: %s", f.c_str());
            GlobPtr g;
            g.reset(new Poco::Glob(f));
            glob_filters.push_back(g);
        }
    }

    ros::service::waitForService("/gazebo/get_world_properties");
    ros::ServiceClient getWorldProperties = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
    gazebo_msgs::GetWorldProperties world_properties;

    ros::service::waitForService("/gazebo/get_model_state");
    ros::ServiceClient getModelState = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState model_state;

    ros::Publisher pub = n.advertise<std_msgs::Int16>("count", 1000);
    ros::Publisher pub_similarity;
    if (use_object_axes || use_object_position) {
        pub_similarity = n.advertise<std_msgs::Float32>("similarity", 1000);
    }
    ros::Rate rate(1);
    
    ROS_INFO("enter main loop");
    int prev_count = -1;
    while (ros::ok()) {
        getWorldProperties.call(world_properties);
        for (auto name: world_properties.response.model_names) {
            if (seen_models.find(name) == seen_models.end()) {
                check_glob(name);
                seen_models[name] = true;
            }
            if (target_objects.find(name) != target_objects.end()) {
                model_state.request.model_name = name;
                model_state.request.relative_entity_name = box_name;
                getModelState.call(model_state);
                object_pose[name] = model_state.response.pose;
            }
        }
        int count = 0;
        double similarity = M_PI;
        double distance = 100000.0;
        for (auto o: objects_list) {
            auto it2 = object_pose.find(o);
            if (it2 != object_pose.end()) {
                const geometry_msgs::Pose p = it2->second;
                // relative pose of the object from the box
                ROS_DEBUG("%s-rel %f %f %f", o.c_str(), p.position.x, p.position.y, p.position.z);
                if (use_object_axes) {
                    tf::Quaternion q;
                    tf::quaternionMsgToTF(p.orientation, q);
                    auto t = tf::Transform(q, tf::Vector3(0, 0, 0));
                    auto v = t * object_axes;
                    v.normalize();
                    // calculate cosine similarity
                    similarity = target_axes.angle(v);
                    ROS_DEBUG("%s-axes %f %f %f, similarity %f", o.c_str(), v.getX(), v.getY(), v.getZ(), similarity);
                }
                // check relative pose of the object is within the bounding box of the box
                if (fabs(p.position.x - box_pose[0]) < box_size[0] / 2 &&
                    fabs(p.position.y - box_pose[1]) < box_size[1] / 2 &&
                    fabs(p.position.z - box_pose[2]) < box_size[2] / 2) {
                    count++;
                    if (use_object_axes) {
                        if (both_direction > 0) {
                            if (M_PI - similarity < similarity) {
                                similarity = M_PI - similarity;
                            }
                        }
                        if (similarity <= cosine_similarity) {
                            count++;
                        }
                    }
                    if (use_object_position) {
                        distance = target_position.distance(tf::Vector3(p.position.x, p.position.y, p.position.z));
                        if (distance <= max_distance) {
                            count++;
                        }
                    }
                }
            }
        }
        if (prev_count != count) {
            ROS_INFO("object in box: %i", count);
            prev_count = count;
        }
        std_msgs::Int16 msg;
        msg.data = count;
        pub.publish(msg);
        double total_similarity = 1.0;
        if (use_object_axes) {
            if (similarity <= cosine_similarity) {
                total_similarity *= (cosine_similarity - similarity) / cosine_similarity;
            } else {
                total_similarity = 0.0;
            }
        }
        if (use_object_position) {
            if (distance <= max_distance) {
                total_similarity *= (max_distance - distance) / max_distance;
            } else {
                total_similarity = 0.0;
            }
        }
        if (count == 0) {
            total_similarity = 0.0;
        }
        std_msgs::Float32 msg2;
        if (use_object_axes || use_object_position) {
            msg2.data = total_similarity;
            pub_similarity.publish(msg2);
        }
        ros::spinOnce();
        rate.sleep();
    }
}
