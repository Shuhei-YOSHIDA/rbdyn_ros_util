/**
 * @file rbdyn_ros_util.h
 */

#ifndef INCLUDE_RBDYN_ROS_UTIL_RBDYN_ROS_UTIL_H
#define INCLUDE_RBDYN_ROS_UTIL_RBDYN_ROS_UTIL_H

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>

namespace rbdyn_ros_util
{

/**
 * @brief Output value of MultBodyConfig
 */
void printMBC(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc);

/**
 * @param mb
 * @param mbc
 * @param[out] msg
 */
void jointStateFromMBC(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc,
                       sensor_msgs::JointState& msg);

/**
 * @brief Put value of JointState to MultiBodyConfig
 * @details Other values of mbc which is not in name of msg are unchanged.
 *          And if msg includes name that mbc doesn't possess, Unsafe.
 * @param mb
 * @param msg
 * @param[out] mbc
 */
void jointStateToMBC(rbd::MultiBody mb, sensor_msgs::JointState msg,
                     rbd::MultiBodyConfig& mbc);

/**
 * @brief set position limit to mbc
 * @param mb
 * @param q_limit Position limit that is gotten from RBDynUrdf::Limit
 * @param[out] mbc_limit
 */
//void setPosLimitsToMBC(rbd::MultiBody mb,
//                       std::map<std::string, double> q_limit, rbd::MultiBodyConfig& mbc_limit);

/**
 * @brief Put value of sva::PTransformd to geometry_msgs::Pose
 */
geometry_msgs::Pose geoPoseFromPTd(const sva::PTransformd& pt);

/**
 * @brief Put value of geometry_msgs::Pose to sva::PTransformd
 */
sva::PTransformd geoPoseToPTd(const geometry_msgs::Pose& pose);

/**
 * @param mb
 * @param mbc
 * @param mb_root_frame_id
 * @param[out] msg
 */
visualization_msgs::MarkerArray makeMarkerArrayFromMBC(
    const rbd::MultiBody& mb,
    const rbd::MultiBodyConfig& mbc,
    const std::string& mb_root_frame_id="world");

/**
 * @param transform Marker's pose, X_O_{marker}
 * @param marker_id For header of msg
 * @param frame_id For header of msg
 * @param[out] msg
 */
visualization_msgs::Marker makePTdMarker(
    const sva::PTransformd& transform,
    const int& marker_id = 0,
    const std::string& frame_id = "world");

/**
 * @brief return Marker msg filled by points data
 */
visualization_msgs::Marker makePTdsPointsMarker(
    const std::vector<sva::PTransformd>& transforms,
    const int& marker_id = 0,
    const std::string& frame_id="world");

}

#endif
