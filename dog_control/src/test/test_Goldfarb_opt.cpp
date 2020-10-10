#include "dog_control/optimization/QuadSolver.h"
#include <iostream>
#include <sys/time.h>

using namespace dog_control;

int test_Goldfarb_opt()
{
    constexpr int n_v = 150;
    constexpr int n_ce = 25;
    constexpr int n_ce_sel = 30; // n_ce_sel > n_ce
    constexpr int n_ci = 300;

    // set a hessian matrix
    Eigen::MatrixXf H = Eigen::MatrixXf::Random(n_v, n_v);
    H = H.transpose() * H;
    Eigen::VectorXf g = Eigen::VectorXf::Random(n_v);

    // span an available area by several corners
    std::array<Eigen::VectorXf, n_ci> x0;

    for (int i = 0; i < n_ci; i++)
        x0[i] = Eigen::VectorXf::Random(n_v) * 10;

    // set constraints. keep all x0 available to ensure
    // there exists a solution.

    Eigen::MatrixXf ci;
    Eigen::VectorXf li;
    ci = Eigen::MatrixXf::Random(n_ci, n_v);
    li.resize(n_ci);
    // ci * x >= li

    for (int i = 0; i < n_ci; i++)
    {
        double min = 100000;

        for (int j = 0; j < n_ci; j++)
        {
            double res = ci.row(i).dot(x0[j]);

            if (min > res)
                min = res;
        }

        li(i) = min;
    }

    // use pca to compute ce, le.
    Eigen::MatrixXf cov = Eigen::MatrixXf::Zero(n_v, n_v);
    Eigen::VectorXf sum = Eigen::VectorXf::Zero(n_v);

    // only use the first n_ce_sel elements
    for (int i = 0; i < n_ce_sel; i++)
    {
        sum += x0[i];
        cov += x0[i] * x0[i].transpose();
    }

    cov -= sum * sum.transpose() / n_ce_sel;
    Eigen::EigenSolver<Eigen::MatrixXf> es(cov);
    Eigen::MatrixXf vec = es.pseudoEigenvectors();
    Eigen::VectorXf val = es.pseudoEigenvalueMatrix().diagonal();

    Eigen::MatrixXf ce;
    ce.resize(n_ce, n_v);

    for (int i = 0; i < n_ce; i++)
    {
        int max_pos = 0;

        val.maxCoeff(&max_pos);

        val(max_pos) = -1;
        ce.row(i) = vec.col(max_pos).transpose();
    }

    Eigen::VectorXf le = ce * x0[0];

    Eigen::VectorXf res_g = Eigen::VectorXf::Zero(n_v);

    struct timeval t1, t2;
    int timeuse;
    gettimeofday(&t1, NULL);
//    (void) timeuse;
//    (void) t2;

    optimization::SolveQuadProg(H, g,
                   ce.transpose(), - le,
                   ci.transpose(), - li,
                   res_g);
//    res_g = res_gf.cast<double>();

    gettimeofday(&t2, NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) * 1000000
            + (t2.tv_usec - t1.tv_usec);

    std::cout << "goldfarb elapsed: " << timeuse << "us" << std::endl;

    return 0;
}
