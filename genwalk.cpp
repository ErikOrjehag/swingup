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
    const float g = 9.82f;
    const float z0 = 1.0;

    const float Ts = 1.0;
    const float w = std::sqrt(g/z0);

    ACADO::Control px;

    ACADO::DifferentialState x;
    ACADO::DifferentialState xdot;

    ACADO::DiscretizedDifferentialEquation f(Ts);

    const float e1 = std::exp(w*Ts)  + std::exp(-w*Ts);
    const float e2 = std::exp(w*Ts)  - std::exp(-w*Ts);
    const float e3 = std::exp(-w*Ts) - std::exp(w*Ts);

    f << next(x)    == 0.5 * ( e1     * x + e2 / w * xdot) + (1.0-0.5*e1) * px;
    f << next(xdot) == 0.5 * ( e2 * w * x + e1     * xdot) + 0.5*w*e3     * px;


    ACADO::Function h;
    h << xdot;
    h << (next(px) - px);
    ACADO::Function hN;
    hN << xdot;
    
    const int N = 5;
    const float tStart = 0.0;
    const float tEnd = Ts * (float)N;

    ACADO::OCP ocp(tStart, tEnd, N);
    ocp.subjectTo(f);
    ocp.subjectTo(ACADO::AT_START, x == 0);
    ocp.subjectTo(ACADO::AT_START, xdot == 0.5);
    
    std::cout << h.getDim() << std::endl;

    ACADO::DMatrix Q(h.getDim(), h.getDim());
    Q << 1.0, 0.0,
         0.0, 0.5;
    ACADO::DVector r(h.getDim());
    r.setZero();
    r(0) = 0.1;
    ACADO::DMatrix QN(hN.getDim(), hN.getDim());
    QN << 1.0;
    ACADO::DVector rN(hN.getDim());
    rN.setZero();
    rN(0) = 0.1;

    ocp.minimizeLSQ(Q, h, r);
    ocp.minimizeLSQEndTerm(QN, hN);
    
    ACADO::OptimizationAlgorithm alg(ocp);
    alg.set(ACADO::INTEGRATOR_TOLERANCE, 1e-6);
    alg.set(ACADO::KKT_TOLERANCE, 1e-3);
    alg.set(ACADO::DISCRETIZATION_TYPE,   ACADO::MULTIPLE_SHOOTING);

    ACADO::GnuplotWindow window;
    window.addSubplot(px, "px");
    window.addSubplot(x, "x");
    window.addSubplot(xdot, "xdot");
    alg << window;

    alg.solve();

    return 0;
};