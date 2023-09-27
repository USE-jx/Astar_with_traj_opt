#include "simple_optimal_control/point_motion.h"
#include <vector>


using namespace casadi;
using namespace std;

PointMotion::PointMotion()
{
    dynamic_equation_ = setDynamicEquation();
    
}

PointMotion::~PointMotion()
{
}

casadi::Function PointMotion::setDynamicEquation() {
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX psi = MX::sym("psi");
    MX v = MX::sym("v");
    MX state_vars = MX::vertcat({x, y, psi, v});
    MX a = MX::sym("a");
    MX r = MX::sym("r");
    MX control_vars = MX::vertcat({a, r});

    MX rhs = MX::vertcat({v * MX::cos(psi),
                          v * MX::sin(psi),
                          r,
                          a});
    return Function("kinematic_equation", {state_vars, control_vars},{rhs});
}

MX PointMotion::RK4_discretisize(MX dt, MX x_v, MX u_v) {
    MX result;
    vector<MX> input(2);
    input[0] = x_v;
    input[1] = u_v;

    MX k1 = dynamic_equation_(input)[0];
    input[0] += dt / 2 * k1;
    MX k2 = dynamic_equation_(input)[0];
    input[0] = x_v + dt / 2 * k2;
    MX k3 = dynamic_equation_(input)[0];
    input[0] = x_v + dt * k3;
    MX k4 = dynamic_equation_(input)[0];
    result = x_v + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    return result;
}
void PointMotion::setInitial(vector<Eigen::Vector2d> pos) {
    int init_size = pos.size();
    //cout << pos.size() << " " <<  N_ << endl;
    x_init_.clear();
    y_init_.clear();

    if (init_size > N_) {
        int delta = init_size / N_;
        for (int i = 0; i < init_size; i += delta) {
            x_init_.push_back(pos[i](0));
            y_init_.push_back(pos[i](1));
            if (x_init_.size() == N_ + 1) {
                break;
            }
        }
        //cout << "x_init size:" << x_init_.size() << endl;

        
    } else {
        int delta = N_ / init_size;
        for (int i = 0; i < init_size; i++) {
            for (int j = 0; j < delta; ++j) {
                x_init_.push_back(pos[i](0));
                y_init_.push_back(pos[i](1));
            
            }
        }
        //cout << "x_init size:" << x_init_.size() << endl;
        if (x_init_.size() < N_) {
            for (int i = x_init_.size() -  1; i < N_; ++i) {
                x_init_.push_back(x_init_[x_init_.size() -  1]);
                y_init_.push_back(y_init_[y_init_.size() -  1]);
            }
        }
    }

}

bool PointMotion::solve(Eigen::Vector3d start, Eigen::Vector3d goal) {
    Opti opti = Opti();

    Slice all;
    cout << "start solve" << endl;

    X_ = opti.variable(4, N_ + 1);
    U_ = opti.variable(2, N_);
    T_ = opti.variable();

    MX x = X_(0, all);
    MX y = X_(1, all);
    MX psi = X_(2, all);
    MX v = X_(3, all);

    MX a = U_(0, all);
    MX r = U_(1, all);

    opti.minimize(T_);

    MX dt = T_ / N_;

    cout << "set dynamic constraints" << endl;
    //-------dynamic constraints----------
     for (int i = 0; i < N_; ++i) {
        //欧拉法
        // vector<MX> input(2);
        // input[0] = X_(all, i);
        // input[1] = U_(all, i);
        // MX X_next = dynamic_equation_(input)[0] * dt_ + X_(all, i);

        //四阶龙哥库塔法
        MX X_next = RK4_discretisize(dt, X_(all, i), U_(all, i)); 
        opti.subject_to(X_next == X_(all, i + 1));
        
    }
    cout << "set dynamic constraints finish" << endl;

    //v and a constrains
    opti.subject_to(-10 <= a <= 10);
    opti.subject_to(-0.7 <= r <= 0.7);
    opti.subject_to(-5 <= v <= 5);

    
    //boundary conditions
    opti.subject_to(x(0) == start(0));   
    opti.subject_to(x(N_) == goal(0));   
    opti.subject_to(y(0) == start(1));     
    opti.subject_to(y(N_) == goal(1)); 
    opti.subject_to(psi(0) == start(2));
    opti.subject_to(psi(N_) == goal(2));
    opti.subject_to(v(0) == 0);     
    opti.subject_to(v(N_) == 0); 
    opti.subject_to(T_ >= 0);
  
    /*
    三角形面积法碰撞约束
    核心思想是如果点在多边形外部，那么这个点和多边形所有边组成的三角形面积之和大于原多边形面积
    */
    for (int i = 1; i < N_-1; ++i) {
        opti.subject_to(abs(X_(0, i) * (-10.5) + 4.1    * (-7.6)    + (-6.5) * X_(1, i)
                          - X_(0, i) * (-7.6)  - 4.1    * X_(1, i)  - (-6.5) * (-10.5))
                      + abs(X_(0, i) * (-7.6)  + (-6.5) * (-10.5)   + (-6.5) *  X_(1, i)
                          - X_(0, i) * (-10.5) - (-6.5) *  X_(1, i) - (-6.5) * (-7.6))
                      + abs(X_(0, i) * (-10.5) + (-6.5) * (-10.5)   + (4.1)  *  X_(1, i)
                          - X_(0, i) * (-10.5) - (-6.5) *  X_(1, i) - (4.1)  * (-10.5)) 
                          >
                           abs(4.1 * (-7.6) + (-6.5)    * (-10.5)    + (-6.5) * (-10.5)
                          - 4.1 * (-10.5) - (-6.5)    * (-10.5)  - (-6.5) * (-7.6))+3);
    }    


    //set initial value
    cout << "set init val" << endl;
    DM x_init(x_init_);
    DM y_init(y_init_);
    //cout << "x_init size:" << x_init_.size() << endl;
    
    // x_init = DM::zeros(N_+1);
    //y_init = DM::zeros(N_+1);

    opti.set_initial(x, x_init);
    opti.set_initial(y, y_init);
    opti.set_initial(psi, 0);
    opti.set_initial(v, 0);

    opti.set_initial(T_, 2);

    Dict solver_opts;
    solver_opts["expand"] = true;
    solver_opts["ipopt.max_iter"] = 50000;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.acceptable_tol"] = 1e-6;
    solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    opti.solver("ipopt", solver_opts);
    cout << "solve finish" << endl;

    solution_ = std::make_unique<OptiSol>(opti.solve());
    
    return true;

}


void PointMotion::getSolution(casadi::DM& state, casadi::DM& control, casadi::DM& tf) {
    state = solution_->value(X_);
    control = solution_->value(U_);
    tf = solution_->value(T_);
}