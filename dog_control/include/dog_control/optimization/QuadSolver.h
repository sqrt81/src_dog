/*
    QuadSolver.h
    This file implements a light weight Quadratic Programming problem solver,
    by means of an active-set dual method.

    The problem is in the form:

    min 0.5 * x G x + g0 x
        s.t.
        CE^T x + ce0 = 0
        CI^T x + ci0 >= 0

    The matrix and vectors dimensions are as follows:
        G: n * n
        g0: n

        CE: n * p
        ce0: p

        CI: n * m
        ci0: m

        x: n

    The function will return the cost of the solution written in the x vector or
    std::numeric_limits::infinity() if the problem is infeasible.
    In the latter case, the value of the x vector is not correct.

    References:
    D. Goldfarb, A. Idnani. A numerically stable dual method for solving
    strictly convex quadratic programs.
    Mathematical Programming 27 (1983) pp. 1-33.


    Author: Luca Di Gaspero
            DIEGM - University of Udine, Italy
                luca.digaspero@uniud.it
                http://www.diegm.uniud.it/digaspero/

    The author will be grateful if the researchers using this software will
    acknowledge the contribution of this function in their research papers.

    Copyright (c) 2007-2016 Luca Di Gaspero

    This software may be modified and distributed under the terms
    of the MIT license.  See the LICENSE file for details.
*/

#ifndef QUADSOLVER_H
#define QUADSOLVER_H

#include <Eigen/Eigen>

namespace dog_control
{

namespace optimization
{

float SolveQuadProg(const Eigen::MatrixXf &G, const Eigen::VectorXf &g0,
                    const Eigen::MatrixXf &CE, const Eigen::VectorXf &ce0,
                    const Eigen::MatrixXf &CI, const Eigen::VectorXf &ci0,
                    Eigen::VectorXf &x);

} /* optimization */

} /* dog_control */

#endif /* QUADSOLVER_H */
