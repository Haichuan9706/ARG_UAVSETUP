#ifndef __CONTROLLER_UTILS_H__
#define __CONTROLLER_UTILS_H__

#include <Eigen/Geometry>
#include <uav_utils/utils.h>
#include <sensor_msgs/Imu.h>


struct Desired_State_t
{
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  Eigen::Quaterniond q;

  Desired_State_t(){};

  Desired_State_t(Odom_Data_t &odom)
	: p(odom.p),
		v(Eigen::Vector3d::Zero()),
		a(Eigen::Vector3d::Zero()),
		j(Eigen::Vector3d::Zero()),
		q(odom.q){};
};

struct Controller_Output_t
{
	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Collective mass normalized thrust
	double thrust;

};

struct Gain
{
	double Kp0, Kp1, Kp2;
	double Kv0, Kv1, Kv2;
	double Kvi0, Kvi1, Kvi2;
	double Kvd0, Kvd1, Kvd2;
	double KAngR, KAngP, KAngY;
};

#endif
