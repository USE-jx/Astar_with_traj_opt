#ifndef SIMPLE_OPTIMAL_CONTROL_H
#define SIMPLE_OPTIMAL_CONTROL_H

#include <iostream>
#include <casadi/casadi.hpp>
#include <Eigen/Eigen>
#include <tf2/utils.h>
#include <vector>

using std::vector;

using casadi::MX;

class PointMotion
{
private:
    int N_ = 100;
    double dt_;

    MX X_, U_, T_;
    casadi::Function dynamic_equation_;
    std::unique_ptr<casadi::OptiSol> solution_;
    vector<double> x_init_, y_init_;

public:
    PointMotion();
    ~PointMotion();

    casadi::Function setDynamicEquation();
    MX RK4_discretisize(MX dt, MX x_v, MX u_v);
    void setInitial(vector<Eigen::Vector2d> pos);
    bool solve(Eigen::Vector3d start, Eigen::Vector3d goal);
    void getSolution(casadi::DM& state, casadi::DM& control, casadi::DM& tf);
    
};


#endif