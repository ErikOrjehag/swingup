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

/*ACADO::Expression taylorSin(const ACADO::DifferentialState& x)
{
    return x - 1.f/6.f*x.getPow(3);
    //return -(x-M_PI) + 1.f/6.f*(x-M_PI).getPow(3);
}*/

int main()
{
    const float tStart = 0.0;
    const float tEnd = 3.0;
    const int horizon = 150;
    const float dt = (tEnd-tStart)/horizon;
    const float hz = 1.f/dt;

    const float maxTorque = 0.4;
    const float maxAngVel = M_PI*2*5;

    const float m = 0.5f;
    const float l = 0.2f;
    const float g = 9.82f;
    const float Ir = 0.5*m*(0.1*0.1+0.15*0.15); // 0.008125
    const float I = Ir + m*l*l;

    std::cout 
    << "dt: " << dt 
    << "\nhz: " << hz
    << "\nmaxTorque: " << maxTorque
    << "\nmaxAngVel: " << maxAngVel
    << std::endl;

    ACADO::Control u;

    ACADO::DifferentialState x1; // theta
    ACADO::DifferentialState x2; // theta dot
    ACADO::DifferentialState x3; // omega
    ACADO::DifferentialState x4; // u dot

    ACADO::DiscretizedDifferentialEquation f(dt);

    f << next(x1) == x1 + dt * x2;
    f << next(x2) == x2 + dt * (m*g*std::sin(x1)*l-u) / I;
    //f << next(x2) == x2 + dt * (m*g*taylorSin(x1)*l-u) / I;
    f << next(x3) == x3 + dt * u / Ir;

    ACADO::Function h;
    h << x1;
    //h << x3;
    //h << u;

    ACADO::OCP ocp(tStart, tEnd, horizon);
    ocp.subjectTo(f);
    ocp.subjectTo(-2.f*M_PI <= x1 <= 2.f*M_PI);
    ocp.subjectTo(-maxAngVel <= x3 <= maxAngVel);
    ocp.subjectTo(-maxTorque <= u <= maxTorque);

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

    ocp.subjectTo(ACADO::AT_START, x1 == M_PI);
    ocp.subjectTo(ACADO::AT_END, x1 == 0);
    ocp.subjectTo(ACADO::AT_START, x2 == 0);
    ocp.subjectTo(ACADO::AT_START, x3 == 0);
    ocp.subjectTo(ACADO::AT_END, x3 == 0);

    bool genCode = true;

    if (!genCode)
    {
        const float x1Start = M_PI;
        const float x1End = 0.f;
        const float x2Start = 0.f;
        const float x3Start = 0.f;
        const float x3End = maxAngVel / 2;

        ACADO::Grid timeGrid(tStart, tEnd, horizon);
        ACADO::VariablesGrid xInit(4, timeGrid);
        ACADO::VariablesGrid uInit(1, timeGrid);

        for (int i = 0; i < horizon; i++) {
            float alpha = (float)i/float(horizon-1);
            xInit(i, 0) = x1Start+(x1End-x1Start)*alpha;
            xInit(i, 1) = x2Start;
            xInit(i, 2) = x3Start+(x3End-x3Start)*alpha;
            xInit(i, 3) = 0;

            if (i < horizon) {
                uInit(i, 0) = 0.0;
            }
        }

        

        std::cout << h.getDim() << std::endl;

        ACADO::DMatrix Q(h.getDim(), h.getDim());
        Q << 1;
        ACADO::DVector r(h.getDim());
        r << 0;
        ocp.minimizeLSQ(Q, h, r);
        ocp.minimizeLSQEndTerm(Q, h, r);
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
    else 
    {

        ACADO::BMatrix Q(h.getDim(), h.getDim());
        Q.setIdentity();
        
        //ocp.minimizeLSQ(Q, h);
        //ocp.minimizeLSQEndTerm(Q, h);
        ocp.minimizeLagrangeTerm(x1*x1);

        ACADO::OCPexport mpc(ocp);
        /*mpc.set(ACADO::HESSIAN_APPROXIMATION, ACADO::GAUSS_NEWTON);       // is robust/stable
        mpc.set(ACADO::DISCRETIZATION_TYPE,   ACADO::MULTIPLE_SHOOTING);  // good convergence
        mpc.set(ACADO::SPARSE_QP_SOLUTION,    ACADO::FULL_CONDENSING_N2); // due to qpOASES
        mpc.set(ACADO::INTEGRATOR_TYPE,       ACADO::INT_IRK_GL4);        // accurate
        mpc.set(ACADO::NUM_INTEGRATOR_STEPS,  horizon);
        mpc.set(ACADO::QP_SOLVER,             ACADO::QP_QPOASES);         // free, source code
        mpc.set(ACADO::HOTSTART_QP,                      YES);
        mpc.set(ACADO::CG_USE_OPENMP,                    YES);            // paralellization
        mpc.set(ACADO::CG_HARDCODE_CONSTRAINT_VALUES,    YES);             // set on runtime
        mpc.set(ACADO::CG_USE_VARIABLE_WEIGHTING_MATRIX, NO);            // time-varying costs
        mpc.set(ACADO::USE_SINGLE_PRECISION,             YES);*/

        mpc.set(ACADO::HESSIAN_APPROXIMATION,       ACADO::EXACT_HESSIAN  		);
        mpc.set(ACADO::DISCRETIZATION_TYPE,         ACADO::MULTIPLE_SHOOTING 	);
        mpc.set(ACADO::INTEGRATOR_TYPE,             ACADO::INT_RK4   			);
        mpc.set(ACADO::NUM_INTEGRATOR_STEPS,        horizon            		);
        mpc.set(ACADO::QP_SOLVER,                   ACADO::QP_QPOASES    		);
        mpc.set(ACADO::HOTSTART_QP,                 NO             		);
        mpc.set(ACADO::SPARSE_QP_SOLUTION, 		  ACADO::FULL_CONDENSING_N2	);
        mpc.set(ACADO::DYNAMIC_SENSITIVITY, 		  ACADO::SYMMETRIC				);
        mpc.set(ACADO::CG_HARDCODE_CONSTRAINT_VALUES, YES 					);
        mpc.set(ACADO::CG_USE_VARIABLE_WEIGHTING_MATRIX, YES 				);

        mpc.set(ACADO::GENERATE_TEST_FILE,          YES);
        mpc.set(ACADO::GENERATE_MAKE_FILE,          YES);
        mpc.set(ACADO::GENERATE_MATLAB_INTERFACE,   NO);
        mpc.set(ACADO::GENERATE_SIMULINK_INTERFACE, NO);

        std::cout << "Export code start" << std::endl;

        if (mpc.exportCode("codegen") != ACADO::SUCCESSFUL_RETURN) {
            exit(EXIT_FAILURE);
        }

        std::cout << "Export code end" << std::endl;

        mpc.printDimensionsQP();
    }

    return 0;
};