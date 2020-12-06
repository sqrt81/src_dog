#include "dog_control/optimization/QuadSolver.h"

#include "dog_control/utils/MiniLog.h"

namespace dog_control
{

namespace optimization
{

namespace
{

constexpr int MAX_ITERATION = 100;

inline double fabs_f(double x)
{
    return x > 0 ? x : -x;
}

inline void compute_d(Eigen::VectorXd& d, const Eigen::MatrixXd& J,
                      const Eigen::VectorXd& np)
{
    /* compute d = H^T * np */
    d.noalias() = J.transpose() * np;
}

inline void update_z(Eigen::VectorXd& z, const Eigen::MatrixXd& J,
                     const Eigen::VectorXd& d, int iq)
{
    const int sz = d.size() - iq;
    /* setting of z = H * d */
    z.noalias() = J.rightCols(sz) * d.bottomRows(sz);
}

inline void update_r(const Eigen::MatrixXd& R, Eigen::VectorXd& r,
                     const Eigen::VectorXd& d, int iq)
{
    /* setting of r = R^-1 d */
    r.topRows(iq) = R.topLeftCorner(iq, iq).triangularView<Eigen::Upper>()
            .solve(d.topRows(iq));
}

bool add_constraint(Eigen::MatrixXd& R, Eigen::MatrixXd& J,
                    Eigen::VectorXd& d, int& iq, double& R_norm)
{
    const int n = d.size();

    /*
      we have to find the Givens rotation which will reduce the element
      d[j] to zero.
      if it is already zero we don't have to do anything, except of
      decreasing j
    */
    for (int j = n - 1; j >= iq + 1; j--)
    {
        /*
          The Givens rotation is done with the matrix (cc cs, cs -cc).
          If cc is one, then element (j) of d is zero compared with element
          (j - 1). Hence we don't have to do anything.
          If cc is zero, then we just have to
          switch column (j) and column (j - 1) of J.
          Since we only switch columns in J, we have to be careful how we
          update d depending on the sign of gs.
          Otherwise we have to apply the Givens rotation to these columns.
          The i - 1 element of d has to be updated to h.
        */
        double cc = d[j - 1];
        double ss = d[j];
        double h = sqrt(cc * cc + ss * ss);

        if (h < std::numeric_limits<double>::epsilon()) // h == 0
            continue;

        d[j] = 0.0;
        ss = ss / h;
        cc = cc / h;

        if (cc < 0.0)
        {
            cc = - cc;
            ss = - ss;
            d[j - 1] = - h;
        }
        else
            d[j - 1] = h;

//        const double xny = ss / (1.0 + cc);

        const Eigen::VectorXd t1 = J.col(j - 1);
        const Eigen::VectorXd t2 = J.col(j);
        J.col(j - 1) = cc * t1 + ss * t2;
        J.col(j)     = ss * t1 - cc * t2;
//        J.col(j) = xny * (t1 + J.col(j - 1)) - t2;
    }

    /* update the number of constraints added*/
    iq++;

    /*
      To update R we have to put the iq components of the d vector
      into column iq - 1 of R
     */
    R.block(0, iq - 1, iq, 1) = d.topRows(iq);

    if (fabs(d[iq - 1]) <= std::numeric_limits<double>::epsilon() * R_norm)
    {
        // problem degenerate
        return false;
    }

    R_norm = std::max<double>(R_norm, fabs(d[iq - 1]));
    return true;
}

void delete_constraint(Eigen::MatrixXd& R, Eigen::MatrixXd& J,
                       Eigen::VectorXi& A, Eigen::VectorXd& u,
                       int p, int& iq, int l)
{
    int qq = -1; // just to prevent warnings from smart compilers

    /* Find the index qq for active constraint l to be removed */
    for (int i = p; i < iq; i++)
        if (A[i] == l)
        {
            qq = i;
            break;
        }

    /* remove the constraint from the active set and the duals */
    for (int i = qq; i < iq - 1; i++)
    {
        A[i] = A[i + 1];
        u[i] = u[i + 1];
        R.col(i) = R.col(i + 1);
    }

    A[iq - 1] = A[iq];
    u[iq - 1] = u[iq];
    A[iq] = 0;
    u[iq] = 0.0;
    R.col(iq - 1).setZero();
    /* constraint has been fully removed */
    iq--;

    if (iq == 0)
        return;

    for (int j = qq; j < iq; j++)
    {
        double cc = R(j, j);
        double ss = R(j + 1, j);
        double h = sqrt(cc * cc + ss * ss);

        if (h < std::numeric_limits<double>::epsilon()) // h == 0
            continue;

        cc = cc / h;
        ss = ss / h;
        R(j + 1, j) = 0.0;

        if (cc < 0.0)
        {
            R(j, j) = - h;
            cc = - cc;
            ss = - ss;
        }
        else
            R(j, j) = h;

//        double xny = ss / (1.0 + cc);

        Eigen::MatrixXd::ColsBlockXpr R_interest(
                    R.middleCols(j + 1, iq - j - 1));
        Eigen::RowVectorXd t1 = R_interest.row(j);
        Eigen::RowVectorXd t2 = R_interest.row(j + 1);
        R_interest.row(j)     = cc * t1 + ss * t2;
        R_interest.row(j + 1) = ss * t1 - cc * t2;
//        R_interest.row(j + 1) = xny * (t1 + R_interest.row(j)) - t2;

        t1 = J.col(j);
        t2 = J.col(j + 1);
        J.col(j)     = cc * t1 + ss * t2;
        J.col(j + 1) = ss * t1 - cc * t2;
//        J.col(j + 1) = xny * (t1 + J.col(j)) - t2;
    }
}

} /* anonymous */

double SolveQuadProg(const Eigen::MatrixXd& G, const Eigen::VectorXd& g0,
                    const Eigen::MatrixXd& CE, const Eigen::VectorXd& ce0,
                    const Eigen::MatrixXd& CI, const Eigen::VectorXd& ci0,
                    Eigen::VectorXd& x)
{
    /* p is the number of equality constraints */
    /* m is the number of inequality constraints */
    const int n = G.cols();
    const int p = CE.cols();
    const int m = CI.cols();

    CHECK(G.rows() == n);
    CHECK(g0.size() == n);
    CHECK(CE.rows() == n);
    CHECK(ce0.size() == p);
    CHECK(CI.rows() == n);
    CHECK(ci0.size() == m);

    x.resize(n);

    int i; /* index */
    int iter = 0; /* expire counter */

    /* this is the index of the constraint to be added to the active set */
    int ip;

    Eigen::VectorXd s(m + p), r(m + p), u(m + p), u_old(m + p);
    Eigen::VectorXd np(n), z(n), x_old(n);

    double f_value, R_norm;

    constexpr double inf = std::numeric_limits<double>::has_infinity ?
                std::numeric_limits<double>::infinity() : 1E30;

    /* t is the step lenght, which is the minimum of
     * the partial step length t1
     * and the full step length t2 */
    double t, t1, t2;

    Eigen::VectorXi A(m + p), A_old(m + p), iai(m + p);
    Eigen::VectorXi iaexcl(m + p);

    /*
     * Preprocessing phase
     */

    /* decompose the matrix G in the form L L^T */
    Eigen::MatrixXd J;

    if (G.isDiagonal())
        J = G.diagonal().cwiseSqrt().cwiseInverse().asDiagonal();
    else
        J = G.llt().matrixU()
            .solve(Eigen::MatrixXd::Identity(n, n));

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd d = Eigen::VectorXd::Zero(n);

    R_norm = 1.0; /* this variable will hold the norm of the matrix R */

    /* An estimate for cond(G) */
    const double cond = G.trace() * J.trace();

    /*
     * Find the unconstrained minimizer of the quadratic form
     *  0.5 * x^T G x + g0^T x
     * this is a feasible point in the dual space
     *  x = - G^-1 * g0
     */
    x.noalias() = - J * (J.transpose() * g0);
    f_value = g0.dot(x) / 2;

    /* Add equality constraints to the working set A */
    int iq = 0;

    for (i = 0; i < p; i++)
    {
        np = CE.col(i);
        compute_d(d, J, np);
        update_z(z, J, d, iq);
        update_r(R, r, d, iq);

        /* compute full step length t2:
         * i.e., the minimum step in primal space
         * s.t. the contraint becomes feasible */
        t2 = 0.0;

        if (z.norm() > std::numeric_limits<double>::epsilon()) // i.e. z != 0
            t2 = (- np.dot(x) - ce0[i]) / np.dot(z);

        /* set x = x + t2 * z */
        x += t2 * z;

        /* set u = u+ */
        u[iq] = t2;
        u.topRows(iq) -= t2 * r.topRows(iq);

        /* compute the new solution value */
        f_value += 0.5 * (t2 * t2) * np.dot(z);
        A[i] = - i - 1;

        if (!add_constraint(R, J, d, iq, R_norm))
        {
            // Equality constraints are linearly dependent
            throw std::runtime_error("Constraints are linearly dependent");
            return f_value;
        }
    }

    /* set iai = K \ A */
    for (i = 0; i < m; i++)
      iai[i] = i;

    /* q is size of the active set A
     * (containing the indices of the active constraints) */
//    int q = 0;

l1: /* step 1: choose a violated constraint */
    for (i = p; i < iq; i++)
    {
        ip = A[i];
        iai[ip] = -1;
    }

    ip = 0; /* ip will be the index of the chosen violated constraint */

    iaexcl.topRows(m).setOnes();

    /* compute s[x] = ci^T * x + ci0 for all elements of K \ A */
    s.head(m).noalias() = CI.transpose() * x + ci0;

    /* psi will contain the sum of all infeasibilities */
    double psi = s.cwiseMin(0).sum();

    // note that the right side would be
    // m * std::numeric_limits<float>::epsilon() * cond * 0.01
    // if use float instead of double.
    if (- psi <= m * std::numeric_limits<double>::epsilon() * cond * 100.)
    {
      /* numerically there are not infeasibilities anymore */
//      q = iq;

      return f_value;
    }

    /* save old values for u and A */
    u_old.head(iq) = u.head(iq);
    A_old.head(iq) = A.head(iq);
    /* and for x */
    x_old = x;

    double ss = 0.0;

l2: /* Step 2: check for feasibility and determine a new S-pair */
    for (i = 0; i < m; i++)
    {
      if (s[i] < ss && iai[i] != -1 && iaexcl[i])
      {
        ss = s[i];
        ip = i;
      }
    }

    if (ss >= 0.0)
    {
//        q = iq;

        return f_value;
    }

    /* set np = n[ip] */
    np = CI.col(ip);
    /* set u = [u 0]^T */
    u[iq] = 0.0;
    /* add ip to the active set A */
    A[iq] = ip;

l2a:/* Step 2a: determine step direction */

    iter++;

    if (iter > MAX_ITERATION)
        return inf;

    /* compute z = H np: the step direction in the primal space
     * (through J, see the paper) */
    compute_d(d, J, np);
    update_z(z, J, d, iq);
    /* compute N* np (if q > 0): the negative of the step direction
     * in the dual space */
    update_r(R, r, d, iq);

    /* Step 2b: compute step length */
    int l = 0;
    /* Compute t1: partial step length
     * (maximum step in dual space without violating dual feasibility) */
    t1 = inf; /* +inf */

    /* find the index l s.t. it reaches the minimum of u+[x] / r */
    for (int k = p; k < iq; k++)
    {
        if (r[k] > 0.0)
        {
            if (u[k] / r[k] < t1)
            {
                t1 = u[k] / r[k];
                l = A[k];
            }
        }
    }

    /* Compute t2: full step length
     * (minimum step in primal space such that
     *  the constraint ip becomes feasible) */
    if (z.norm() > std::numeric_limits<double>::epsilon()) // i.e. z != 0
    {
        t2 = - s[ip] / np.dot(z);

        /* patch suggested by Takano Akio
         * for handling numerical inconsistencies */
        if (t2 < 0)
            t2 = inf;
    }
    else
      t2 = inf; /* +inf */

    /* the step is chosen as the minimum of t1 and t2 */
    t = std::min(t1, t2);

    /* Step 2c: determine new S-pair and take step: */

    /* case (i): no step in primal or dual space */
    if (t >= inf)
    {
        /* QPP is infeasible */
        // FIXME: unbounded to raise
//        q = iq;
        return inf;
    }

    /* case (ii): step in dual space */
    if (t2 >= inf)
    {
        /* set u = u +  t * [-r 1] and drop constraint l from the active set A */
        u.topRows(iq) -= t * r.topRows(iq);
        u[iq] += t;
        iai[l] = l;
        delete_constraint(R, J, A, u, p, iq, l);
        goto l2a;
    }

    /* case (iii): step in primal and dual space */

    /* set x = x + t * z */
    x += t * z;
    /* update the solution value */
    f_value += t * np.dot(z) * (0.5 * t + u[iq]);
    /* u = u + t * [-r 1] */
    u.topRows(iq) -= t * r.topRows(iq);
    u[iq] += t;

    if (fabs_f(t - t2) < std::numeric_limits<double>::epsilon())
    {
        /* full step has taken */
        /* add constraint ip to the active set*/
        if (!add_constraint(R, J, d, iq, R_norm))
        {
            iaexcl[ip] = false;
            delete_constraint(R, J, A, u, p, iq, ip);

            for (i = 0; i < m; i++)
                iai[i] = i;

            A.segment(p, iq - p) = A_old.segment(p, iq - p);
            u.segment(p, iq - p) = u_old.segment(p, iq - p);

            for (i = p; i < iq; i++)
            {
                iai[A[i]] = -1;
            }

            x = x_old;

            goto l2; /* go to step 2 */
        }
        else
            iai[ip] = -1;

        goto l1;
    }

    /* a patial step has taken */
    /* drop constraint l */
    iai[l] = l;
    delete_constraint(R, J, A, u, p, iq, l);

    /* update s[ip] = CI^T * x + ci0 */
    s[ip] = CI.col(ip).dot(x) + ci0[ip];

    goto l2a;
}

} /* optimization */

} /* dog_control */
