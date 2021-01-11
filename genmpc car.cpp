#include <iostream>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

/*ACADO::Expression taylorSin(const ACADO::DifferentialState& x)
{
    return x - 1.f/6.f*x.getPow(3);
    //return -(x-M_PI) + 1.f/6.f*(x-M_PI).getPow(3);
}*/

int main()
{
    ACADO::DifferentialState x;
    ACADO::DifferentialState y;
    ACADO::DifferentialState theta;
    ACADO::Control phi_L;

    ACADO::DifferentialEquation f;
    ACADO::Function h;
    ACADO::Function hN;

    // SYSTEM VARIABLES
    //
    float l_vehicle = 2.78;
    float v_vehicle = 0.1;
    float Ts = 0.05;
    float horizon_t = 10.0;

    float weight_x = 1.0;
    float weight_y = 1.0;
    float weight_theta = 1.0;
    float weight_phi_L = 0.1;

    // Model equations
    //
    f << dot(x) == v_vehicle * cos(theta);
    f << dot(y) == v_vehicle * sin(theta);
    f << dot(theta) == v_vehicle / l_vehicle * tan(phi_L);

    // Reference functions
    //
    h << x << y << theta << phi_L;
    hN << x << y<< theta;

    // Weighting matrices
    //
    ACADO::DMatrix W = ACADO::eye<double>(h.getDim());
    W(0, 0) = weight_x;
    W(1, 1) = weight_y;
    W(2, 2) = weight_theta;
    W(3, 3) = weight_phi_L;

    ACADO::DMatrix WN = ACADO::eye<double>(hN.getDim());
    WN(0, 0) = weight_x;
    WN(1, 1) = weight_y;
    WN(2, 2) = weight_theta;

    // MPC PROBLEM FORMULATION
    //
    int horizon_steps = horizon_t / Ts;
    
    ACADO::OCP ocp(0.0, 5.0, horizon_steps);

    ocp.subjectTo(f);
    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);
    ocp.subjectTo(-0.5 <= phi_L <= 1.5);

    // Export as  C-Code
    //
    ACADO::OCPexport mpc(ocp);
    int Ni = 1;

    mpc.set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);
    mpc.set(ACADO::DISCRETIZATION_TYPE, ACADO::MULTIPLE_SHOOTING);
    mpc.set(ACADO::INTEGRATOR_TYPE, ACADO::INT_RK4);
    mpc.set(ACADO::NUM_INTEGRATOR_STEPS, horizon_steps * Ni);
    mpc.set(ACADO::SPARSE_QP_SOLUTION, ACADO::CONDENSING);
    mpc.set(ACADO::QP_SOLVER, ACADO::QP_QPOASES);
    mpc.set(ACADO::HOTSTART_QP, YES);
    mpc.set(ACADO::GENERATE_TEST_FILE, YES);
    mpc.set(ACADO::GENERATE_MAKE_FILE, YES);
    mpc.set(ACADO::GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(ACADO::GENERATE_SIMULINK_INTERFACE, NO);

    if (mpc.exportCode("codegen2") != ACADO::SUCCESSFUL_RETURN)
    {
        exit( EXIT_FAILURE);
    }

    return 0;
};