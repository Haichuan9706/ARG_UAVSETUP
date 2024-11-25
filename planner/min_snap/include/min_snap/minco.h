#ifndef _MINCO_H_
#define _MINCO_H_

#include <Eigen/Eigen>
#include <iostream>

using std::vector;
using namespace std;

namespace my_planner
{
    class Minco
    {
    private:
        vector<Eigen::Vector3d> wps;
        int ctrl_order, di, di_;       // 控制量次数，中间状态维数
        int n_order, n_seg, n_per_seg; //次数，段数，每段变量数
        double mean_vel, Tsigma;       // ?, 轨迹总时间
        Eigen::VectorXd ts;
        Eigen::VectorXd poly_coef_x, poly_coef_y, poly_coef_z;
        Eigen::MatrixXd F_0, E_M, M, b, c;
        Eigen::MatrixXd poly_coef;
        vector<Eigen::MatrixXd> Ei, Fi;

        int
        fact(int n);
        void init_ts(int init_type);

        Eigen::MatrixXd betaD0D(double t, int N, int d);
        Eigen::MatrixXd betaD(double t, int N, int d);
        void calEi();
        void calFi();
        void calM();
        void calB();
        void dispMatrix(const Eigen::MatrixXd &matrix);

    public:
        Minco(){};
        ~Minco(){};
        Minco(const vector<Eigen::Vector3d> &waypoints, double meanvel = 1.0);
        void Init(const vector<Eigen::Vector3d> &waypoints, double meanvel = 1.0);
        void MincoConstruct();
        Eigen::MatrixXd getPolyCoef();
        Eigen::MatrixXd getDecVel();
        Eigen::VectorXd getTime();
    };
}
#endif