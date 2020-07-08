#include <acado_optimal_control.hpp>
#include "/home/mspahn/develop/acado/acado/bindings/acado_gnuplot/gnuplot_window.hpp"

int main( )
{
    USING_NAMESPACE_ACADO
    
    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState     x, y, theta, s1;
    Control               v1, v2, sv1;
    DifferentialEquation  f;
    
    const double x_d = 4.0;
    const double y_d = 4.0;
    const double theta_d = 0.7;
    const double r = 0.08;
    const double L = 0.544;
    const double w_x = 100;
    const double w_o = 5;
    const double w_slack = 100000000;
    const double w_u = 1;
    const double o1_x = 1.2;
    const double o1_y = 1.0;
    const double o1_r = 1.2;

    Expression dist_pos =  (x_d - x) * (x_d - x) + (y_d - y) * (y_d - y);
    Expression dist_theta = (theta_d - theta) * (theta_d - theta);
    Expression u_mag = v1 * v1 + v2 * v2;
    
    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(x) == r / 2 * (v1 + v2) * cos(theta);
    f << dot(y) == r / 2 * (v1 + v2) * sin(theta);
    f << dot(theta) == r / (2 * L) * (-v1 + v2);
    f << dot(s1) == sv1;
    //
    
    
    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    // ocp(0.0, time_horizon, steps)
    //OCP ocp( t_start, t_end, 10 );
    OCP ocp(0.0, 5.0, 10);

	  ocp.minimizeLagrangeTerm(
      w_x * dist_pos +
      w_o * dist_theta + 
      w_u * u_mag + 
      w_slack * s1 * s1
    );
    /*
    ocp.minimizeMayerTerm( x);
    */

	  ocp.subjectTo(f);
    
    ocp.subjectTo(AT_START, x == 0.0);
    ocp.subjectTo(AT_START, y == 0.0);
    ocp.subjectTo(AT_START, s1 == 0.0);
    ocp.subjectTo(AT_START, theta == 0.0);
    ocp.subjectTo( -4.0 <= v1 <= 4.0 );
    ocp.subjectTo( -4.0 <= v2 <= 4.0 );
    ocp.subjectTo( -0.1 <= sv1 <= 0.1 );
    ocp.subjectTo( 0 <= s1 <= 10000.0 );
    
    Expression dist_o1_base(1);
    dist_o1_base(0) = pow(pow(x - o1_x, 2) + pow(y - o1_y, 2) , 0.5);
    ocp.subjectTo(dist_o1_base - o1_r + s1 >= 0);

    GnuplotWindow window;
    window.addSubplot(x,"DIFFERENTIAL STATE  x");
    window.addSubplot(y,"DIFFERENTIAL STATE  y");
    window.addSubplot(s1,"DIFFERENTIAL STATE s1");
    window.addSubplot(v1,"CONTROL  v1"   );
    window.addSubplot(v2,"CONTROL  v2"   );
    window.addSubplot(sv1,"CONTROL  sv1"   );
    window.addSubplot(x, y);
    
    
    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ----------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);
    
    algorithm.set( ABSOLUTE_TOLERANCE    , 1e-7          );
    algorithm.set( INTEGRATOR_TOLERANCE  , 1e-7          );
    algorithm.set( HESSIAN_APPROXIMATION , EXACT_HESSIAN );
    
    algorithm << window;
    algorithm.solve();
    
    return 0;
}
