#pragma once

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <bits/stdc++.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iarc_sim_test_tools/common.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <unordered_map>

namespace gazebo {

static const std::string kdefualtFrameId = "world";
static const std::string kdefaultLinkName = "base_link";

class CustomWindPlugin : public ModelPlugin {
  public:
    CustomWindPlugin()
        : ModelPlugin()
        , namespace_(kDefaultNamespace)
        , link_name_(kDefaultLinkName) {
    }

    typedef struct WindParams {
        double angle;
        double speed;

        WindParams(const double& angle, const double& speed)
            : angle(angle)
            , speed(speed){};

        // comparison operator to check hash collisions
        bool operator==(const WindParams& wind_params) const {
            return (angle == wind_params.angle) && (speed == wind_params.speed);
        }
    } WindParams;

    typedef struct DynParams {
        ignition::math::Vector3d force;
        ignition::math::Vector3d torque;

        DynParams(const ignition::math::Vector3d& force, const ignition::math::Vector3d& torque)
            : force(force)
            , torque(torque){};

        DynParams() {
            force = ignition::math::Vector3d::Zero;
            torque = ignition::math::Vector3d::Zero;
        };
    } DynParams;

    virtual ~CustomWindPlugin(){};

  protected:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    void onUpdate(const common::UpdateInfo& _info);
    ignition::math::Vector3d interpolateWindDynamics(WindParams params);
    void odometryCallback_(const nav_msgs::Odometry::ConstPtr msg);
    void QueueThread();

  private:
    // hashing function for wind parameters
    typedef struct WindHasher {
        std::size_t operator()(const WindParams& params) const {
            // Compute individual hash values for angle & speed then combine
            // http://stackoverflow.com/a/1646913/126995
            std::size_t hash = 1009;
            hash = hash * 9176 + std::hash<double>()(params.angle);
            hash = hash * 9176 + std::hash<double>()(params.speed);
            return hash;
        }
    } WindHasher;

    void initializeFieldTable();

    event::ConnectionPtr update_connection_;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber rosSub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;

    physics::ModelPtr model_;
    sdf::ElementPtr sdf_;
    physics::LinkPtr link_;

    std::string namespace_;
    std::string link_name_;

    ignition::math::Vector3d xyz_offset_;
    ignition::math::Vector3d force_direction_;
    ignition::math::Vector3d interp_force_;

    std::vector<double> forcex_list_;
    std::vector<double> forcey_list_;
    std::vector<double> wind_speeds_;
    std::vector<double> wind_angles_;

    int rows_;
    int columns_;
    double precision_;
    double windspeed_;
    double wind_angle_;
    double relative_angle_;
    std::unordered_map<WindParams, DynParams, WindHasher> field_table_;
};
}  // namespace gazebo
