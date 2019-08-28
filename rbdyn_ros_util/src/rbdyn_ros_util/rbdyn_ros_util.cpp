/**
 * @file rbdyn_ros_util.cpp
 */

#include <eigen_conversions/eigen_msg.h>
#include <RBDyn/FK.h>

#include "rbdyn_ros_util/rbdyn_ros_util.h"
#include "easy_marker/easy_marker.h"

using namespace std;
using namespace Eigen;
using namespace tf;
using namespace sva;
using namespace rbd;
using namespace color_names;
using namespace easy_marker;
using namespace sensor_msgs;
using namespace visualization_msgs;
using namespace geometry_msgs;

namespace rbdyn_ros_util
{

void printMBC(const MultiBody& mb, const MultiBodyConfig& mbc,
              const map<string, PTransformd>& mbToBase)
{
  auto mbc_ = mbc;
  forwardKinematics(mb, mbc_);

  // mbc.q
  for (int i = 0; i < mb.nrJoints(); i++)
  {
    cout << mb.joint(i).name() << ": [";
    for (auto&& q : mbc_.q[i])
    {
      cout << q << ", ";
    }
    cout << "]" << endl;
  }

  // mbc.bodyPosW
  for (int i = 0; i < mb.nrBodies(); i++)
  {
    auto name = mb.body(i).name();
    auto X_O_b = mbToBase.at(name)*mbc_.bodyPosW[i]; // X_j_b*X_O_j
    cout << "------" << endl;
    cout << name << ": " << endl;
    cout << "translation" << endl
         << X_O_b.translation().transpose() << endl;
    cout << "rotation" << endl
         << X_O_b.rotation() << endl;
  }

}

void jointStateFromMBC(const MultiBody& mb, const MultiBodyConfig& mbc,
                       JointState& msg)
{
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear(); /// @todo set velocity
  msg.effort.clear();

  for (int count = 0; count < mbc.q.size(); count++)
  {
    //if (mb.joint(count).type() == rbd::Joint::Type::Rev ||
    //  mb.joint(count).type() == rbd::Joint::Type::Prism) // 1dof joint
    if (mb.joint(count).dof() == 1) // Only for idof joint
    {
      msg.name.push_back(mb.joint(count).name());
      msg.position.push_back(mbc.q[count][0]);
    }
  }
  msg.header.stamp = ros::Time::now();
}

void jointStateToMBC(const MultiBody& mb, const JointState& msg,
                     MultiBodyConfig& mbc)
{
  /// @todo set velocity
  for (int i = 0; i < msg.name.size(); i++)
  {
    int index = mb.jointIndexByName(msg.name[i]);
    if (mb.joint(index).dof() == 1) // Only for 1dof joint
    {
      mbc.q[index][0] = msg.position[i];
    }
  }
}

void poseToMBCFreeJointPos(const MultiBody& mb, const Pose& msg,
                           MultiBodyConfig& mbc, int joint_index)
{
  if (mb.sJoint(joint_index).type() != Joint::Type::Free)
  {
    cerr << "joint_index is invalid @rbdyn_ros_util::poseToMBCFreeJointPos" << endl;
    return;
  }
  Vector3d p;
  Quaterniond q;
  pointMsgToEigen(msg.position, p);
  quaternionMsgToEigen(msg.orientation, q);
  q = q.inverse(); /// @todo is it ok?
  mbc.q[joint_index][0] = q.w();
  mbc.q[joint_index][1] = q.x();
  mbc.q[joint_index][2] = q.y();
  mbc.q[joint_index][3] = q.z();
  mbc.q[joint_index][4] = p[0];
  mbc.q[joint_index][5] = p[1];
  mbc.q[joint_index][6] = p[2];
}

Pose geoPoseFromPTd(const PTransformd& pt)
{
  /// @note PTransformd.rotation == ^{After}R_{Before}
  ///       geometry_msgs::Quaternion => ^{Before}R_{After}
  ///       Between SVA and ROS, rotation matrix should be inverted
  Pose p;
  Vector3d pos = pt.translation();
  Quaterniond qua(pt.rotation().transpose());

  pointEigenToMsg(pos, p.position);
  quaternionEigenToMsg(qua, p.orientation);

  return p;
}

PTransformd geoPoseToPTd(const Pose& pose)
{
  Vector3d pos;
  Quaterniond qua;

  pointMsgToEigen(pose.position, pos);
  quaternionMsgToEigen(pose.orientation, qua);

  return PTransformd(qua.inverse(), pos);
}

MarkerArray makeMarkerArrayFromMBC(
    const MultiBody& mb,
    const MultiBodyConfig& mbc,
    const map<string, PTransformd>& mbToBase,
    const string& mb_root_frame_id)
{
  MarkerArray msg;
  int id = 0;
  auto stamp = ros::Time::now();
  string frame_id = mb_root_frame_id;

  for (int i = 0; i < mb.nrBodies(); i++)
  {
    PTransformd X_O_l = mbToBase.at(mb.body(i).name())*mbc.bodyPosW[i];
    Marker mrk = makeMarkerMESH_RESOURCETemplate();
    mrk.header.stamp = stamp;
    mrk.header.frame_id = frame_id;
    mrk.id = id;
    id++;
    mrk.pose = geoPoseFromPTd(X_O_l);
    mrk.color = makeColorMsg("coral");
    mrk.color.a = 0.5;
    mrk.scale.x *= 0.1;
    mrk.scale.y *= 0.1;
    mrk.scale.z *= 0.1;
    msg.markers.push_back(mrk);

    mrk = makeMarkerTEXT_VIEW_FACINGTemplate(mb.body(i).name());
    mrk.header.stamp = stamp;
    mrk.header.frame_id = frame_id;
    mrk.id = id;
    id++;
    mrk.pose = geoPoseFromPTd(X_O_l);
    mrk.color = makeColorMsg("aqua");
    mrk.scale.x *= 0.1;
    mrk.scale.y *= 0.1;
    mrk.scale.z *= 0.1;
    msg.markers.push_back(mrk);
  }

  return msg;
}

Marker makePTdMarker(
    const sva::PTransformd& transform,
    const int& marker_id,
    const std::string& frame_id)
{
  Marker msg = makeMarkerMESH_RESOURCETemplate(
      "package://easy_marker/meshes/xyz_marker.stl", true,
      1.0, "crimson", frame_id);
  msg.header.stamp = ros::Time::now();
  msg.id = marker_id;
  msg.pose = geoPoseFromPTd(transform);

  return msg;
}

Marker makePTdsPointsMarker(
    const std::vector<sva::PTransformd>& transforms,
    const int& marker_id,
    const std::string& frame_id)
{
  vector<Point> points;
  vector<string> color_names;

  for (auto&& t : transforms)
  {
    geometry_msgs::Point p;
    pointEigenToMsg(t.translation(), p);
    points.push_back(p);
    color_names.push_back("lime");
  }

  Marker msg = makeMarkerPOINTSTemplate(points, color_names, 1.0, frame_id);
  msg.header.stamp = ros::Time::now();
  msg.id = marker_id;

  return msg;
}

}
