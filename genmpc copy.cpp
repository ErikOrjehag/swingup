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
    const float m = 0.5f;
    const float l = 0.2f;
    const float g = 9.82f;
    const float Ir = 0.5*m*(0.1*0.1+0.15*0.15); // 0.008125
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
    //h << u;

    const float tStart = 0.0;
    const float tEnd = 4.0;
    const int horizon = 200;

    ACADO::OCP ocp(tStart, tEnd, horizon);
    ocp.subjectTo(f);
    ocp.subjectTo(-2.f*M_PI <= x1 <= 2.f*M_PI);
    ocp.subjectTo(-50 <= x3 <= 50);
    ocp.subjectTo(-1.0 <= u <= 1.0);

    #if FALSE
    ACADO::VariablesGrid u0(1, 0.0, 8.0, 40);
    u0.setZero();
    u0(0,0) = -0.1;

    ACADO::DVector x0(3);
    x0.setZero();
    x0[0] = M_PI;

    ACADO::DynamicSystem sys(f);
    ACADO::Process pro;
    pro.setDynamicSystem(sys, ACADO::INT_RK45);
    pro.set(ACADO::ABSOLUTE_TOLERANCE, 1.0e-8);
    pro.set(ACADO::PLOT_RESOLUTION, ACADO::HIGH);

    pro.init(0.0, x0);
    pro.run(u0);

    ACADO::VariablesGrid x;

    pro.getLast(ACADO::LOG_SIMULATED_DIFFERENTIAL_STATES, x);

    ACADO::GnuplotWindow window;
    window.addSubplot(x(0), "x1");
    window.addSubplot(x(1), "x2");
    window.addSubplot(x(2), "x3");
    
    window.plot();
    #endif



    int task = 0;

    if (task == 0)
    {
        const float x1Start = M_PI;
        const float x1End = 0.f;
        const float x2Start = 0.f;
        const float x3Start = 0.f;
        const float x3End = 25.f;

        ACADO::Grid timeGrid(tStart, tEnd, horizon);
        ACADO::VariablesGrid xInit(3, timeGrid);
        ACADO::VariablesGrid uInit(1, timeGrid);

        for (int i = 0; i < horizon; i++) {
            float alpha = (float)i/float(horizon-1);
            xInit(i, 0) = x1Start+(x1End-x1Start)*alpha;
            xInit(i, 1) = x2Start;
            xInit(i, 2) = x3Start+(x3End-x3Start)*alpha;

            if (i < horizon) {
                uInit(i, 0) = 0.f;
            }
        }

        ocp.subjectTo(ACADO::AT_START, x1 == M_PI);
        ocp.subjectTo(ACADO::AT_END, x1 == 0);
        ocp.subjectTo(ACADO::AT_START, x2 == 0);
        ocp.subjectTo(ACADO::AT_START, x3 == 0);
        //ocp.subjectTo(ACADO::AT_END, x3 == 25);

        //std::cout << h.getDim() << std::endl;

        /**/ACADO::DMatrix Q(2, 2);
        Q << 1, 0, 0, 0.001;
        ACADO::DVector r(2);
        r << 0, 25;
        ocp.minimizeLSQ(Q, h, r);
        //ocp.minimizeMayerTerm((x1-M_PI)*(x1-M_PI));
        //ocp.minimizeLagrangeTerm((x1-M_PI)*(x1-M_PI)+0.01*u*u);
        //ocp.minimizeLagrangeTerm(x1*x1);

        ACADO::OptimizationAlgorithm alg(ocp);
        alg.set(ACADO::INTEGRATOR_TOLERANCE, 1e-6);
        alg.set(ACADO::KKT_TOLERANCE, 1e-3);
        alg.set(ACADO::DISCRETIZATION_TYPE,   ACADO::MULTIPLE_SHOOTING);

        alg.initializeDifferentialStates(xInit);
        alg.initializeControls(uInit);

        //alg.set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON_WITH_BLOCK_BFGS);
        //alg.set(ACADO::KKT_TOLERANCE, 1e-5);
        //alg.set(ACADO::ABSOLUTE_TOLERANCE, 1.0e-7);
        //alg.set(ACADO::INTEGRATOR_TOLERANCE , 1.0e-7);
        //alg.set(ACADO::HESSIAN_APPROXIMATION, ACADO::EXACT_HESSIAN);

        ACADO::GnuplotWindow window;
        window.addSubplot(u, "u");
        window.addSubplot(x1, "x1");
        window.addSubplot(x2, "x2");
        window.addSubplot(x3, "x3");
        alg << window;

        alg.solve();

    }

    return 0;
};