#include <bits/stdc++.h>
#include <cmath>
#include <cstdlib>
#include <iarc_wind_plugin/wind_plugin.hpp>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

namespace gazebo {

void CustomWindPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    model_ = parent;
    sdf_ = sdf;

    gzmsg << "Wind plugin loaded for " << model_->GetName() << std::endl;

    if (sdf->HasElement("namespace")) {
        namespace_ = sdf->GetElement("namespace")->Get<std::string>();
        gzmsg << namespace_ << std::endl;
    } else {
        gzerr << "Element <namespace> is not specified. Aborting.";
        return;
    }
    if (sdf->HasElement("link_name")) {
        link_name_ = sdf->GetElement("link_name")->Get<std::string>();
    } else {
        gzerr << "Element <link_name> is not specified. Aborting.";
        return;
    }
    if (sdf->HasElement("windspeed")) {
        windspeed_ = sdf->GetElement("windspeed")->Get<double>();
    } else {
        windspeed_ = 1.0;
    }
    if (sdf->HasElement("windangle")) {
        wind_angle_ = sdf->GetElement("windangle")->Get<double>();
    } else {
        wind_angle_ = 0;  // in radians
    }
    if (sdf->HasElement("precision")) {
        precision_ = sdf->GetElement("precision")->Get<double>();
    } else {
        gzerr << "Precision is not set. Aborting.";
        return;
    }

    link_ = model_->GetLink(link_name_);

    if (!ros::isInitialized()) {
        int argc = 0;
        
        char** argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }

    std::string path = ros::package::getPath("iarc_wind_plugin");
    YAML::Node config = YAML::LoadFile(path + "/config/params.yaml");
    forcex_list_ = config["force_x"].as<std::vector<double>>();
    forcey_list_ = config["force_y"].as<std::vector<double>>();
    wind_speeds_ = config["wind_speeds"].as<std::vector<double>>();
    wind_angles_ = config["wind_angles"].as<std::vector<double>>();
    rows_ = config["rows"].as<double>();
    columns_ = config["columns"].as<double>();

    // Input values from cfd analysis
    initializeFieldTable();

    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
    this->rosSub = this->rosNode->subscribe("/firefly/ground_truth/odometry", 1, &CustomWindPlugin::odometryCallback_, this);
    this->rosQueueThread = std::thread(std::bind(&CustomWindPlugin::QueueThread, this));
    // force_direction_.Set(cos(wind_angle_), sin(wind_angle_), 0);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&CustomWindPlugin::onUpdate, this, _1));
}

void CustomWindPlugin::odometryCallback_(const nav_msgs::Odometry::ConstPtr msg) {
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    relative_angle_ = wind_angle_ - yaw;
}

void CustomWindPlugin::QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

ignition::math::Vector3d CustomWindPlugin::interpolateWindDynamics(WindParams params) {
    params.angle /= precision_;
    params.speed /= precision_;

    double temp, sub_angle, sub_speed;
    sub_angle = modf(params.angle, &temp);
    sub_speed = modf(params.speed, &temp);

    WindParams ceil = params;  // impl getUpperBounds
    ceil.angle += 1 - sub_angle;
    ceil.speed += 1 - sub_speed;
    WindParams floor = params;  // impl getLowerBounds
    floor.angle -= sub_angle;
    floor.speed -= sub_speed;

    ceil.angle *= precision_;
    ceil.speed *= precision_;
    floor.angle *= precision_;
    floor.speed *= precision_;
    params.angle *= precision_;
    params.speed *= precision_;

    CustomWindPlugin::DynParams bot_left = field_table_[CustomWindPlugin::WindParams(floor.angle, floor.speed)];
    CustomWindPlugin::DynParams bot_right = field_table_[CustomWindPlugin::WindParams(ceil.angle, floor.speed)];
    CustomWindPlugin::DynParams top_left = field_table_[CustomWindPlugin::WindParams(floor.angle, ceil.speed)];
    CustomWindPlugin::DynParams top_right = field_table_[CustomWindPlugin::WindParams(ceil.angle, ceil.speed)];

    interp_force_ = (bot_left.force * (ceil.angle - params.angle) * (ceil.speed - params.speed) +
                     bot_right.force * (params.angle - floor.angle) * (ceil.speed - params.speed) +
                     top_left.force * (ceil.angle - params.angle) * (params.speed - floor.speed) +
                     top_right.force * (params.angle - floor.angle) * (params.speed - floor.speed));
    interp_force_ /= (ceil.angle - floor.angle) * (ceil.speed - floor.speed);

    return interp_force_;
};

void CustomWindPlugin::onUpdate(const common::UpdateInfo& _info) {
    WindParams test_params(abs(relative_angle_), windspeed_);  // Input angle and speed
    interp_force_ = interpolateWindDynamics(test_params);

    if (relative_angle_ < 0) {
        interp_force_ *= -1;
    }
    // link_->AddForce(windspeed_ * force_direction_);  // if want to apply gust of constant wind
    link_->AddForce(interp_force_);  // considering the cfd analysis
}
void CustomWindPlugin::initializeFieldTable() {
    // dummy initialization for testing - filling of the hash table will change as per cfd data - also will be a function of precision sdf parameter
    for (int i = 0; i < rows_; i++) {
        for (int j = 0; j < columns_; j++) {
            field_table_[WindParams(wind_angles_[i], wind_speeds_[j])] =
                CustomWindPlugin::DynParams(ignition::math::Vector3d(forcex_list_[i] + forcex_list_[j], forcey_list_[i] + forcey_list_[j], 0),
                    ignition::math::Vector3d(i + 2, i + 2, i + 2));
        }
    }
}

GZ_REGISTER_MODEL_PLUGIN(CustomWindPlugin);

}  // namespace gazebo
