#include "linear_control.h"

#include <ros/ros.h>

#include <iostream>

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q) {
  double yaw =
      atan2(2 * (q.x() * q.y() + q.w() * q.z()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param) {
  resetThrustMapping();
}

Eigen::Quaterniond ToQuaternion(Eigen::Vector3d Euler) {
  double cy = std::cos(Euler[0] * 0.5);
  double sy = std::sin(Euler[0] * 0.5);
  double cp = std::cos(Euler[1] * 0.5);
  double sp = std::sin(Euler[1] * 0.5);
  double cr = std::cos(Euler[2] * 0.5);
  double sr = std::sin(Euler[2] * 0.5);

  Eigen::Quaterniond q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;

  return q;
}

Eigen::Vector3d ToEuler(Eigen::Quaterniond q) {
  Eigen::Vector3d angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  angles[2] = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1)
    angles[1] =
        std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    angles[1] = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  angles[0] = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}

/*
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/
quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl(
    const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu,
    Controller_Output_t &u) {
  /* WRITE YOUR CODE HERE */

  // SE(3)
  Eigen::Vector3d des_acc(0.0, 0.0, 0.0);

  bool use_try_se3_control(true);
  if (use_try_se3_control) {
    Eigen::Vector3d ypr_des = ToEuler(des.q);

    Eigen::Vector3d z_B(imu.a[0], imu.a[1], imu.a[2] + param_.gra);
    z_B.normalize();
    Eigen::Vector3d z_W(0, 0, 1);

    Eigen::Vector3d e_p, e_v;
    e_p = odom.p - des.p;
    e_v = odom.v;
    Eigen::Vector3d F_des;
    double K_p(1.0), K_v(1.0);
    F_des = param_.mass * param_.gra * z_W - K_p * e_p - K_v * e_v;
    des_acc = Eigen::Vector3d(0, 0, F_des.dot(z_B) / param_.mass);
    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);

    Eigen::Vector3d z_B_des = F_des.normalized();
    Eigen::Vector3d x_C_des(std::cos(ypr_des[0]), std::sin(ypr_des[0]), 0);
    Eigen::Vector3d y_B_des = (z_B_des.cross(x_C_des)).normalized();
    Eigen::Vector3d x_B_des = (y_B_des.cross(z_B_des)).normalized();

    Eigen::Matrix<double, 3, 3> R_B_des;
    R_B_des << x_B_des, y_B_des, z_B_des;
    u.q = Eigen::Quaterniond(R_B_des);
    u.q = imu.q * odom.q.inverse() * u.q;
  } else {
    Eigen::Vector3d Kp, Kv;
    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
    des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) +
              Kp.asDiagonal() * (des.p - odom.p);
    des_acc += Eigen::Vector3d(0, 0, param_.gra);

    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
    double roll, pitch, yaw, yaw_imu;
    double yaw_odom = fromQuaternion2yaw(odom.q);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;
    pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;
    yaw = fromQuaternion2yaw(des.q);
    yaw_imu = fromQuaternion2yaw(imu.q);
    // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
    //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
    //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    u.q = imu.q * odom.q.inverse() * q;
  }

  /* WRITE YOUR CODE HERE */

  // used for debug
  debug_msg_.des_p_x = des.p(0);
  debug_msg_.des_p_y = des.p(1);
  debug_msg_.des_p_z = des.p(2);

  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);

  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);

  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();

  debug_msg_.des_thr = u.thrust;

  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100) {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage
*/
double LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc) {
  double throttle_percentage(0.0);

  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool LinearControl::estimateThrustModel(const Eigen::Vector3d &est_a,
                                        const Parameter_t &param) {
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1) {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045)  // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035)  // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();

    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    // fflush(stdout);

    debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void LinearControl::resetThrustMapping(void) {
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}

