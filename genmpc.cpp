#include <iostream>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

namespace std
{
    ACADO::Expression sin(const ACADO::DifferentialState& x)
    {
        return x.getSin();
    }
}

int main()
{
    /*const float r1 = 0.075;
    const float r2 = r1 + 0.02;
    const float thick = 0.009;
    const float m_disk = M_PI * (r2*r2-r1*r1) * thick * 2700.f;
    const float Ir = 0.5*m_disk*(r1*r1+r2*r2) * 1.5; // 0.008125, add 10% for motor intertia
    std::cout << Ir << std::endl; 0.0028519
    std::cout << 0.5*m_disk*(r1*r1+r2*r2) << std::endl;
    exit(0);*/

    const float l = 0.15f;
    const float g = 9.82f;
    const float Ir = 0.01209677419 * 0.5; // real -> 0.01209677419; // izz="0.008125"
    const float m = 0.43;
    const float I = Ir + m*l*l;

    ACADO::Control u;

    ACADO::DifferentialState x1; // theta
    ACADO::DifferentialState x2; // theta dot
    ACADO::DifferentialState x3; // omega

    ACADO::DifferentialEquation f;

    f << dot(x1) == x2;
    f << dot(x2) == (m*g*std::sin(x1)*l-u) / I;
    f << dot(x3) == u / Ir;

    ACADO::Function h;
    h << x1;
    h << x3;
    h << u;
    ACADO::Function hN;
    hN << x1;
    hN << x3;
    
    const int N = 40;
    const float Ts = 0.025;
    const float tStart = 0.0;
    const float tEnd = Ts * (float)N;

    ACADO::OCP ocp(tStart, tEnd, N);
    ocp.subjectTo(f);
    //ocp.subjectTo(-2.f*M_PI <= x1 <= 2.f*M_PI);
    ocp.subjectTo(-30 <= x3 <= 30);
    ocp.subjectTo(-0.5 <= u <= 0.5);

    ACADO::VariablesGrid u0(1, tStart, tEnd, N);
    u0.setZero();
    float torq = 0.41;
    u0(1,0) = -torq;
    u0(2,0) = -torq;
    u0(3,0) = torq;
    u0(4,0) = torq;
    u0(5,0) = torq;
    u0(6,0) = torq;
    u0(7,0) = torq;
    u0(8,0) = torq;
    u0(9,0) = -torq;
    u0(10,0) = -torq;
    u0(11,0) = -torq;
    u0(12,0) = -torq;
    u0(13,0) = -torq;
    u0(14,0) = -torq;
    u0(15,0) = -torq;
    u0(16,0) = -torq;
    u0(17,0) = -torq;
    u0(18,0) = -torq;
    u0(19,0) = -torq;
    u0(20,0) = torq;
    u0(21,0) = torq;
    u0(22,0) = torq;
    u0(23,0) = torq;
    u0(24,0) = torq;
    u0(25,0) = torq;
    u0(26,0) = torq;
    u0(27,0) = torq;
    u0(28,0) = torq;
    u0(29,0) = torq;
    u0(30,0) = torq;
    u0(31,0) = torq;
    u0(32,0) = torq;
    u0(33,0) = torq;
    u0(34,0) = torq;

    ACADO::DVector x0(3);
    x0.setZero();
    x0[0] = M_PI;

    ACADO::DynamicSystem sys(f);
    ACADO::Process pro;
    pro.setDynamicSystem(sys, ACADO::INT_RK45);
    pro.set(ACADO::ABSOLUTE_TOLERANCE, 1.0e-8);
    pro.set(ACADO::PLOT_RESOLUTION, ACADO::HIGH);

    pro.init(tStart, x0);
    pro.run(u0);

    ACADO::VariablesGrid xInit;

    pro.getLast(ACADO::LOG_SIMULATED_DIFFERENTIAL_STATES, xInit);

    #if FALSE
    ACADO::GnuplotWindow window;
    window.addSubplot(xInit(0), "angle");
    window.addSubplot(xInit(1), "vel");
    window.addSubplot(xInit(2), "rotvel");
    window.addSubplot(u0(0), "u");  
    window.plot();
    #endif

    #if FALSE
    ocp.subjectTo(ACADO::AT_START, x1 == x0(0));
    ocp.subjectTo(ACADO::AT_END, x1 == 0);
    ocp.subjectTo(ACADO::AT_START, x2 == x0(1));
    ocp.subjectTo(ACADO::AT_START, x3 == x0(2));

    ACADO::DMatrix Q(h.getDim(), h.getDim());
    Q << 1, 0, 0,
         0, 1, 0,
         0, 0, 0.1;
    ACADO::DVector r(h.getDim());
    r.setZero();
    ACADO::DMatrix QN(hN.getDim(), hN.getDim());
    QN.setIdentity();
    ACADO::DVector rN(hN.getDim());
    rN.setZero();

    ocp.minimizeLSQ(Q, h, r);
    ocp.minimizeLSQEndTerm(QN, hN);
    
    ACADO::OptimizationAlgorithm alg(ocp);
    alg.set(ACADO::INTEGRATOR_TOLERANCE, 1e-6);
    alg.set(ACADO::KKT_TOLERANCE, 1e-3);
    alg.set(ACADO::DISCRETIZATION_TYPE,   ACADO::MULTIPLE_SHOOTING);

    alg.initializeDifferentialStates(xInit);
    alg.initializeControls(u0);

    ACADO::GnuplotWindow window;
    window.addSubplot(u, "u");
    window.addSubplot(x1, "x1");
    window.addSubplot(x2, "x2");
    window.addSubplot(x3, "x3");
    alg << window;

    alg.solve();

    alg.getControls("u.txt");
    alg.getDifferentialStates("x.txt");
    #endif

    //#if FALSE
    ACADO::BMatrix Q(h.getDim(), h.getDim());
    Q.setIdentity();
    ACADO::BMatrix QN(hN.getDim(), hN.getDim());
    QN.setIdentity();

    ocp.minimizeLSQ(Q, h);
    ocp.minimizeLSQEndTerm(QN, hN);

    auto mpc = ACADO::OCPexport(ocp);
    mpc.set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);
    mpc.set(ACADO::DISCRETIZATION_TYPE, ACADO::MULTIPLE_SHOOTING);
    mpc.set(ACADO::SPARSE_QP_SOLUTION, ACADO::FULL_CONDENSING_N2);
    mpc.set(ACADO::LEVENBERG_MARQUARDT, 1e-5);
    mpc.set(ACADO::INTEGRATOR_TYPE, ACADO::INT_RK45);
    mpc.set(ACADO::NUM_INTEGRATOR_STEPS, 2*N);
    mpc.set(ACADO::QP_SOLVER, ACADO::QP_QPOASES);

    mpc.set(ACADO::HOTSTART_QP, NO);
    mpc.set(ACADO::CG_USE_OPENMP, NO); // Parrallell processing multiple shooting
    mpc.set(ACADO::CG_HARDCODE_CONSTRAINT_VALUES, NO);
    mpc.set(ACADO::CG_USE_VARIABLE_WEIGHTING_MATRIX, NO);

    mpc.set(ACADO::GENERATE_MAKE_FILE, NO);
    mpc.set(ACADO::GENERATE_TEST_FILE, NO);

    mpc.exportCode("../mpcexport");
    //#endif

    return 0;
};