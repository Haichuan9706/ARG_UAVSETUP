#ifndef __LINEAR_CONTROL_H__
#define __LINEAR_CONTROL_H__

#include <Eigen/Geometry>
#include <Eigen/Core>
#include "PX4CtrlParam.h"
#include "input.h"
#include "controller_utils.h"
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>

class LinearControl
{
public:
  LinearControl(Parameter_t &);
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu, 
      Controller_Output_t &u);
  bool estimateThrustModel(const Eigen::Vector3d &est_v,
      const Parameter_t &param);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

  // Thrust-accel mapping params
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
  void resetThrustMapping(void);
  double fromQuaternion2yaw(Eigen::Quaterniond q);
};

#endif
