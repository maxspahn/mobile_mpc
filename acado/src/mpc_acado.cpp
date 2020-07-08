/*
s file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


 /**
  *    \file   examples/ocp/bioreactor.cpp
  *    \author Boris Houska, Filip Logist, Rien Quirynen
  *    \date   2014
  */
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>

/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState     x,y,theta,q1,q2,q3,q4,q5,q6,q7,s1;
    Control               v1,v2,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,sv1;
    DifferentialEquation  f;

    OnlineData x_d; //0
    OnlineData y_d; //1
    OnlineData theta_d; //2
    OnlineData q1_d; //3
    OnlineData q2_d; //4
    OnlineData q3_d; //5
    OnlineData q4_d; //6
    OnlineData q5_d; //7
    OnlineData q6_d; //8
    OnlineData q7_d; //9

    OnlineData r; //10
    OnlineData L; //11
    OnlineData w_x; //12
    OnlineData w_o; //13
    OnlineData w_q; //14
    OnlineData w_u; //15
    OnlineData w_qdot; //16
    OnlineData w_slack; //17

    OnlineData o1_x; // 18
    OnlineData o1_y; // 19
    OnlineData o1_z; // 20
    OnlineData o1_r; // 21

    /*
    OnlineData plane_p1;
    OnlineData plane_p2;
    OnlineData plane_p3;
    OnlineData plane_v1;
    OnlineData plane_v2;
    OnlineData plane_v3;
    OnlineData plane_w1;
    OnlineData plane_w2;
    OnlineData plane_w3;
    */
  
    // Compute distance to goal
    Expression dist_pos =  (x_d - x) * (x_d - x) + (y_d - y) * (y_d - y);
    Expression dist_theta = (theta_d - theta) * (theta_d - theta);
    Expression dist_q = (q1_d - q1) * (q1_d - q1) +
                        (q2_d - q2) * (q2_d - q2) +
                        (q3_d - q3) * (q3_d - q3) +
                        (q4_d - q4) * (q4_d - q4) +
                        (q5_d - q5) * (q5_d - q5) +
                        (q6_d - q6) * (q6_d - q6) +
                        (q7_d - q7) * (q7_d - q7);
    Expression u_mag = v1 * v1 + v2 * v2;
    Expression qdot_mag = q1dot * q1dot + 
                          q2dot * q2dot + 
                          q3dot * q3dot + 
                          q4dot * q4dot + 
                          q5dot * q5dot + 
                          q6dot * q6dot + 
                          q7dot * q7dot;

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    
    f << dot(x) == r / 2 * (v1 + v2) * cos(theta);
    f << dot(y) == r / 2 * (v1 + v2) * sin(theta);
    f << dot(theta) == r / (2 * L) * (-v1 + v2);
    f << dot(q1) == q1dot;
    f << dot(q2) == q2dot;
    f << dot(q3) == q3dot;
    f << dot(q4) == q4dot;
    f << dot(q5) == q5dot;
    f << dot(q6) == q6dot;
    f << dot(q7) == q7dot;
    f << dot(s1) == sv1;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    // ocp(0.0, time_horizon, steps)
    OCP ocp( 0.0, 4.0, 8.0 );

    // Need to set the number of online variables!
    // Count with :%s/pattern//gn
    ocp.setNOD(22);


	  ocp.minimizeLagrangeTerm(
      w_x * dist_pos +
      w_o * dist_theta + 
      w_q * dist_q +
      w_u * u_mag +
      w_qdot * qdot_mag +
      w_slack * s1 * s1
    );
    //ocp.subjectTo( f );
	  ocp.setModel(f);

    //ocp.subjectTo( AT_END, s ==  2.5 );
    //ocp.subjectTo( AT_START, y == 0.0 );
    //ocp.subjectTo( AT_START, theta == 0.0 );
    //ocp.subjectTo( AT_START, s == 0.0 );
	  // Franka Limits
    // q_lim_franka_up = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];
    // q_lim_franka_low = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973];
    // q_lim_franka_vel = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];

    ocp.subjectTo( -4.0 <= v1 <= 4.0 );
    ocp.subjectTo( -4.0 <= v2 <= 4.0 );
    ocp.subjectTo( -2.8973 <= q1 <= 2.8973);
    ocp.subjectTo( -1.7628 <= q2 <= 1.7628);
    ocp.subjectTo( -2.8973 <= q3 <= 2.8973);
    ocp.subjectTo( -3.0718 <= q4 <= -0.0698);
    ocp.subjectTo( -2.8973 <= q5 <= 2.8973);
    ocp.subjectTo( -0.0175 <= q6 <= 3.7525);
    ocp.subjectTo( -2.8973 <= q7 <= 2.8973);
    ocp.subjectTo( -2.1750 <= q1dot <= 2.1750 );
    ocp.subjectTo( -2.1750 <= q2dot <= 2.1750 );
    ocp.subjectTo( -2.1750 <= q3dot <= 2.1750 );
    ocp.subjectTo( -2.1750 <= q4dot <= 2.1750 );
    ocp.subjectTo( -2.6100 <= q5dot <= 2.6100 );
    ocp.subjectTo( -2.6100 <= q6dot <= 2.6100 );
    ocp.subjectTo( -2.6100 <= q7dot <= 2.6100 );
    ocp.subjectTo( 0.0 <= s1 <= 1000000.0 );
    ocp.subjectTo( -1.0 <= sv1 <= 1.0 );

    // DEFINE COLLISION AVOIDANCE CONSTRAINTS
    // -------------------------------------

    Expression fk(3);
    fk(0) = x + 0.25;
    fk(1) = y;
    fk(2) = 0.2;

    Expression dist_o1_base(1);
    dist_o1_base(0) = pow(pow(fk(0) - o1_x, 2) + pow(fk(1) - o1_y, 2) + pow(fk(2) - o1_z, 2) , 0.5);
    ocp.subjectTo(dist_o1_base - o1_r + s1 >= 0);


    // DEFINE A PLOT WINDOW:
    // ---------------------
   /* GnuplotWindow window;
        window.addSubplot( x ,"X"  );
        window.addSubplot( y ,"Y"  );
        window.addSubplot( theta ,"Theta"  );
        window.addSubplot( s,"V" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
	OptimizationAlgorithm algorithm(ocp);
	//RealTimeAlgorithm algorithm(ocp);
    algorithm.set( HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE );
	algorithm.set(PRINTLEVEL, NONE);                       // default MEDIUM (NONE, MEDIUM, HIGH)
	algorithm.set(PRINT_SCP_METHOD_PROFILE, false);        // default false
	algorithm.set(PRINT_COPYRIGHT, false);                 // default true
	algorithm.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
	Grid t(0,5.0,50);
	VariablesGrid s2(4,0,5.0,50),c2(2,0,5.0,50);

    algorithm.initializeDifferentialStates(s2);
    algorithm.initializeControls          (c2);
    
    algorithm.set( MAX_NUM_ITERATIONS, 100 );
    algorithm.set( KKT_TOLERANCE, 1e-8 );
    algorithm << window;
    //algorithm.solve(0.0,state_ini);
	algorithm.solve();
    VariablesGrid s3,c3;
    algorithm.getDifferentialStates(s3);
    algorithm.getControls          (c3);*/

	// DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       EXACT_HESSIAN  		);
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING 	);
	mpc.set( INTEGRATOR_TYPE,             INT_RK4			);
	mpc.set( NUM_INTEGRATOR_STEPS,        50            		);
	mpc.set( QP_SOLVER,                   QP_QPOASES    		);
	mpc.set( HOTSTART_QP,                 NO             		);
	mpc.set( GENERATE_TEST_FILE,          YES            		);
	mpc.set( GENERATE_MAKE_FILE,          YES            		);
	mpc.set( GENERATE_MATLAB_INTERFACE,   NO            		);
	mpc.set( SPARSE_QP_SOLUTION, 		  FULL_CONDENSING_N2	);
	mpc.set( DYNAMIC_SENSITIVITY, 		  SYMMETRIC				);
	mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO 					);
	mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES 				);

	mpc.exportCode( "generated_mpc" );
	mpc.printDimensionsQP( );
	// ----------------------------------------------------------
    return 0;
}
