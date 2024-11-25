#include <min_snap/minco.h>

namespace my_planner
{
    void Minco::dispMatrix(const Eigen::MatrixXd &matrix)
    {
        for (int i = 0; i < matrix.rows(); i++)
        {
            for (int j = 0; j < matrix.cols(); j++)
            {
                //std::cout << matrix(i, j) << " ";
            }
            //std::cout << std::endl;
        }
    }
    Minco::Minco(const vector<Eigen::Vector3d> &waypoints, double meanvel)
    {
        ctrl_order = 4;
        di = 1;
        Tsigma = 15.0;

        n_order = 2 * ctrl_order - 1;
        di_ = 2 * ctrl_order - di;

        wps = waypoints;
        n_seg = int(wps.size()) - 1;
        n_per_seg = n_order + 1;
        mean_vel = meanvel;
    }

    void Minco::Init(const vector<Eigen::Vector3d> &waypoints, double meanvel)
    {
        ctrl_order = 4;
        di = 1;
        Tsigma = 15.0;

        n_order = 2 * ctrl_order - 1;
        di_ = 2 * ctrl_order - di;
        wps = waypoints;
        n_seg = int(wps.size()) - 1;
        n_per_seg = n_order + 1;
        mean_vel = meanvel;
    }

    // 0 means each segment time is one second.
    void Minco::init_ts(int init_type)
    {
        const double dist_min = 2.0;
        ts = Eigen::VectorXd::Zero(n_seg);
        if (init_type)
        {
            Eigen::VectorXd dist(n_seg);
            double dist_sum = 0, t_sum = 0;
            for (int i = 0; i < n_seg; i++)
            {
                dist(i) = 0;
                for (int j = 0; j < 3; j++)
                {
                    dist(i) += pow(wps[i + 1](j) - wps[i](j), 2);
                }
                dist(i) = sqrt(dist(i));
                if ((dist(i)) < dist_min)
                {
                    dist(i) = sqrt(dist(i)) * 2;
                }
                dist_sum += dist(i);
            }
            dist(0) += 1;
            dist(n_seg - 1) += 1;
            dist_sum += 2;
            double T = dist_sum / mean_vel;
            std::cout << "mean_vel:" << mean_vel << ",T: " << T << std::endl;
            for (int i = 0; i < n_seg - 1; i++)
            {
                ts(i) = dist(i) / dist_sum * T;
                t_sum += ts(i);
            }
            ts(n_seg - 1) = T - t_sum;
        }
        else
        {
            for (int i = 0; i < n_seg; i++)
            {
                ts(i) = 1;
            }
        }
        cout << "ts: " << ts.transpose() << endl;
    }

    int Minco::fact(int n)
    {
        if (n < 0)
        {
            cout << "ERROR fact(" << n << ")" << endl;
            return 0;
        }
        else if (n == 0)
        {
            return 1;
        }
        else
        {
            return n * fact(n - 1);
        }
    }

    Eigen::MatrixXd Minco::betaD(double t, int N, int d)
    {
        // //std::cout << "minco::betaD 0" << std::endl;

        Eigen::MatrixXd beta_d = Eigen::MatrixXd::Zero(N + 1, 1);
        // //std::cout << "minco::betaD 1:" << d << " " << N << std::endl;

        for (int i = 0; i < N + 1; i++)
        {
            // //std::cout << "minco::betaD  for:" << i << " " << d << std::endl;

            if (i - d >= 0)
            {
                // //std::cout << "minco::betaD  beta_d 0:" << i << " " << fact(i) << " " << fact(i - d) << " " << t << " " << i - d << " " << pow(t, i - d) << std::endl;

                beta_d(i, 0) = fact(i) / fact(i - d) * pow(t, i - d);
                // //std::cout << "minco::betaD  beta_d 1:" << beta_d(i, 0) << std::endl;
            }
            else
            {
                // //std::cout << "minco::betaD  beta_d 2.1:" << beta_d(0, 0) << std::endl;

                beta_d(i, 0) = 0;
                // //std::cout << "minco::betaD  beta_d 2.2:" << beta_d(i, 1) << std::endl;
            }
            // dispMatrix(beta_d);
        }
        // //std::cout << "minco::betaD 2" << std::endl;

        return beta_d;
    }
    Eigen::MatrixXd Minco::betaD0D(double t, int N, int d)
    {
        // //std::cout << "minco::betaD0D 0" << std::endl;
        Eigen::MatrixXd beta_d0_d = Eigen::MatrixXd::Zero(N + 1, d + 1);
        // //std::cout << "minco::betaD0D 1" << std::endl;

        for (int i = 0; i <= d; i++)
        {
            beta_d0_d.col(i) = betaD(t, N, i);
        }
        // //std::cout << "minco::betaD0D 2" << std::endl;
        return beta_d0_d;
    }

    void Minco::calEi()
    {
        for (int j = 0; j < n_seg - 1; j++)
        {
            Eigen::MatrixXd Ei_tmp = Eigen::MatrixXd::Zero(2 * ctrl_order, 2 * ctrl_order);
            Ei_tmp.topRows(di) = betaD0D(ts(j), n_order, di - 1).transpose();
            Ei_tmp.bottomRows(di_) = betaD0D(ts(j), n_order, di_ - 1).transpose();
            //std::cout << "Ei_tmp size" << Ei_tmp.rows() << " " << Ei_tmp.cols() << std::endl;
            dispMatrix(Ei_tmp);
            Ei.push_back(Ei_tmp);
        }
    }
    void Minco::calFi()
    {
        for (int j = 0; j < n_seg - 1; j++)
        {
            Eigen::MatrixXd Fi_tmp = Eigen::MatrixXd::Zero(2 * ctrl_order, 2 * ctrl_order);
            Fi_tmp.bottomRows(di_) = -1 * betaD0D(0, n_order, di_ - 1).transpose();

            //std::cout << "Fi_tmp size" << Fi_tmp.rows() << " " << Fi_tmp.cols() << std::endl;
            dispMatrix(Fi_tmp);
            Fi.push_back(Fi_tmp);
        }
    }

    void Minco::calM()
    {
        //std::cout << "calM0" << std::endl;
        M = Eigen::MatrixXd::Zero(2 * ctrl_order * n_seg, 2 * ctrl_order * n_seg);
        //std::cout << F_0.rows() << " " << F_0.cols() << std::endl;
        //std::cout << ctrl_order << " " << ctrl_order * 2 << std::endl;
        M.block(0, 0, ctrl_order, ctrl_order * 2) = F_0.transpose();
        //std::cout << "calM1" << std::endl;

        M.bottomRightCorner(ctrl_order, ctrl_order * 2) = E_M.transpose();
        //std::cout << "calM2" << std::endl;

        for (int i = 1; i < n_seg; i++)
        {
            M.block(i * 2 * ctrl_order - ctrl_order, i * 2 * ctrl_order, 2 * ctrl_order, 2 * ctrl_order) = Fi.at(i - 1);
            M.block(i * 2 * ctrl_order - ctrl_order, (i - 1) * 2 * ctrl_order, 2 * ctrl_order, 2 * ctrl_order) = Ei.at(i - 1);
        }
        //std::cout << "calM3" << std::endl;
    }
    void Minco::calB()
    {
        // B只添加了路径点，没有其他状态
        b = Eigen::MatrixXd::Zero(2 * ctrl_order * n_seg, 3);
        b.row(0) << wps[0](0), wps[0](1), wps[0](2);

        for (int i = 1; i < n_seg + 1; i++)
        {
            b(i * ctrl_order * 2 - ctrl_order, 0) = wps[i](0);
            b(i * ctrl_order * 2 - ctrl_order, 1) = wps[i](1);
            b(i * ctrl_order * 2 - ctrl_order, 2) = wps[i](2);
        }
        b.row(2 * ctrl_order * n_seg - ctrl_order) << (*(wps.end() - 1))(0), (*(wps.end() - 1))(1), (*(wps.end() - 1))(2);
    }
    void Minco::MincoConstruct()
    {
        init_ts(1);
        Ei.clear();
        Fi.clear();
        //std::cout << "minco 0" << std::endl;

        // Each matrix is given the final size, for reference only.
        //std::cout << "minco n_seg, n_order" << n_seg << n_order << std::endl;

        //std::cout << "minco 1" << std::endl;
        F_0 = betaD0D(0, n_order, ctrl_order - 1);
        //std::cout << "minco 2: " << n_seg - 1 << " " << ts(n_seg - 1) << std::endl;
        dispMatrix(F_0);

        E_M = betaD0D(ts(n_seg - 1), n_order, ctrl_order - 1);
        //std::cout << "minco 3" << std::endl;
        dispMatrix(E_M);

        calEi();
        //std::cout << "minco 4" << std::endl;

        calFi();
        //std::cout << "minco 5" << std::endl;

        calM();
        dispMatrix(M);
        //std::cout << "minco 6" << std::endl;

        calB();
        dispMatrix(b);

        //std::cout << "minco 7, M" << M.rows() << " " << M.cols() << " b:" << b.rows() << " " << b.cols() << std::endl;
        auto M_inv = M.inverse();
        dispMatrix(M_inv);
        poly_coef = M_inv * b;
        dispMatrix(poly_coef);
    }

    Eigen::MatrixXd Minco::getPolyCoef()
    {

        return poly_coef;
    }

    Eigen::VectorXd Minco::getTime()
    {
        return ts;
    }

}