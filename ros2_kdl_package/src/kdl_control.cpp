#include "kdl_control.h"
#include <iostream>
#include <iomanip>
#include <sstream>


KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{

}


Eigen::VectorXd KDLController::velocity_ctrl_null(const Eigen::Vector3d& pos_err,
                                     const Eigen::Vector3d& ori_err,
                                     const Eigen::Vector3d& des_lin_vel,
                                     double Kp,
                                     const KDL::JntArray& q_min,
                                     const KDL::JntArray& q_max,
                                     double lambda)
{

    const Eigen::MatrixXd J = robot_->getEEJacobian().data;  // 6 x n
    unsigned int nj = robot_->getNrJnts();

    Eigen::VectorXd vel(6);
    vel.head<3>() = des_lin_vel + Kp*pos_err;
    vel.tail<3>() = ori_err;

    const Eigen::MatrixXd Jpinv = pseudoinverse(J);          // n x 6
  
    // qdot0 
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd qdot0(nj);
    for (int i = 0; i < nj; ++i) {
        const double qmin = q_min(i);
        const double qmax = q_max(i);
        const double Delta = qmax - qmin;
        const double a = qmax - q(i);
        const double b = q(i) - qmin;
        const double denom = (a*a) * (b*b);
        // dU/dq_i = 1/λ * (Δ^2) * (2q - qmax - qmin) / [(qmax - q)^2 (q - qmin)^2]
        qdot0(i) = -(1.0 / lambda) * (Delta*Delta) * (2.0*q(i) - qmax - qmin) / denom;
    }

    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nj, nj);
    Eigen::VectorXd qdot = Jpinv * vel + (I - Jpinv * J) * qdot0;

    return qdot;
}


Eigen::VectorXd KDLController::vision_ctrl(const Eigen::Vector3d& p_cO,
                                           const KDL::Frame&     F_ec,
                                           const Eigen::Vector3d& s_des,
                                           double Kv,
                                           const KDL::JntArray& q_min,
                                           const KDL::JntArray& q_max,
                                           double lambda)
{
    const unsigned int nj = robot_->getNrJnts();
    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(nj);

    const double eps = 1e-6;
    const double normP = std::max(p_cO.norm(), eps);

    // ---- s e L(s) ----------------------------------------------------------

    Eigen::Vector3d s = p_cO / normP;
    Eigen::Matrix<double,3,6> L;
    L.setZero();
    // L(s) = [ -(I - s s^T)/||p|| ,  skew(s) ]
    L.block<3,3>(0,0) = -(Eigen::Matrix3d::Identity() - s*s.transpose()) / normP;
    L.block<3,3>(0,3) = skew(s);

    // ---- Jacobian of camera in camera frame ----------------------------
    // J_ee in WORLD
    KDL::Jacobian J_ee(nj);
    J_ee.data = robot_->getEEJacobian().data;     // ^w J_ee

    
    const KDL::Frame F_we = robot_->getEEFrame(); // world->ee
    const KDL::Frame F_wc = F_we * F_ec;          // world->camera

    const Eigen::Vector3d r_ce_w = toEigen(F_we.M) * toEigen(F_ec.p); // r_ce espresso in WORLD
    const Matrix6d X_ce_w = adjoint(r_ce_w, Eigen::Matrix3d::Identity());
    const Eigen::MatrixXd Jc_world = X_ce_w * J_ee.data;   // ^w J_c

    // (2) ROTAZIONE WORLD -> CAMERA (stesso punto)
    const Matrix6d R_cw_blk = spatialRotation(F_wc.M.Inverse()); // blkdiag(R_cw,R_cw)
    const Eigen::MatrixXd Jc = R_cw_blk * Jc_world;        // ^c J_c

    // (3) Task matrix
    const Eigen::MatrixXd A = L * Jc;                      // 3×n

 

    const Eigen::MatrixXd A_pinv = pseudoinverse(A);    
    const Eigen::VectorXd qdot_vis =  (A_pinv * s_des);

    // ---- Null-space -------------------
    Eigen::VectorXd qdot0(nj);
    const Eigen::VectorXd q = robot_->getJntValues();
    for (unsigned int i = 0; i < nj; ++i) {
        const double qmin = q_min(i), qmax = q_max(i);
        const double Delta = qmax - qmin;
        const double a = qmax - q(i), b = q(i) - qmin;
        const double denom = (a*a) * (b*b);
        // dU/dq_i = (1/lambda)*Delta^2*(2q - qmax - qmin)/[(qmax - q)^2 (q - qmin)^2]
        qdot0(i) = -(1.0/lambda) * (Delta*Delta) * (2.0*q(i) - qmax - qmin) / denom;
    }

    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nj, nj);
    const Eigen::MatrixXd N = I - (A_pinv * A);

    return Kv*qdot_vis + N * qdot0;
}


