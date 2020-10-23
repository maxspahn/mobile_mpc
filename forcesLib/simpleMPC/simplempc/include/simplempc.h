/*
simplempc : A fast customized optimization solver.

Copyright (C) 2013-2020 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

/* Generated by FORCES PRO v4.0.0 on Monday, October 19, 2020 at 8:39:18 AM */

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif

#ifndef simplempc_H
#define simplempc_H

#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif


/* DATA TYPE ------------------------------------------------------------*/
typedef double simplempc_float;

typedef double simplempcinterface_float;

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_simplempc
#define MISRA_C_simplempc (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_simplempc
#define RESTRICT_CODE_simplempc (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_simplempc
#define SET_PRINTLEVEL_simplempc    (0)
#endif

/* timing */
#ifndef SET_TIMING_simplempc
#define SET_TIMING_simplempc    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_simplempc			(200)	

/* scaling factor of line search (FTB rule) */
#define SET_FLS_SCALE_simplempc		(simplempc_float)(0.99)      

/* maximum number of supported elements in the filter */
#define MAX_FILTER_SIZE_simplempc	(200) 

/* maximum number of supported elements in the filter */
#define MAX_SOC_IT_simplempc			(4) 

/* desired relative duality gap */
#define SET_ACC_RDGAP_simplempc		(simplempc_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_simplempc		(simplempc_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_simplempc	(simplempc_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_simplempc	(simplempc_float)(1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_simplempc      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_simplempc (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_simplempc   (2)

/* wrong number of inequalities error */
#define INVALID_NUM_INEQ_ERROR_simplempc  (-4)

/* factorization error */
#define FACTORIZATION_ERROR_simplempc   (-5)

/* NaN encountered in function evaluations */
#define BADFUNCEVAL_simplempc  (-6)

/* no progress in method possible */
#define NOPROGRESS_simplempc   (-7)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_simplempc   (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_simplempc   (-12)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_simplempc  (-100)

/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
    /* vector of size 300 */
    simplempc_float x0[300];

    /* vector of size 19 */
    simplempc_float xinit[19];

    /* vector of size 5115 */
    simplempc_float all_parameters[5115];


} simplempc_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
    /* vector of size 20 */
    simplempc_float x01[20];

    /* vector of size 20 */
    simplempc_float x02[20];

    /* vector of size 20 */
    simplempc_float x03[20];

    /* vector of size 20 */
    simplempc_float x04[20];

    /* vector of size 20 */
    simplempc_float x05[20];

    /* vector of size 20 */
    simplempc_float x06[20];

    /* vector of size 20 */
    simplempc_float x07[20];

    /* vector of size 20 */
    simplempc_float x08[20];

    /* vector of size 20 */
    simplempc_float x09[20];

    /* vector of size 20 */
    simplempc_float x10[20];

    /* vector of size 20 */
    simplempc_float x11[20];

    /* vector of size 20 */
    simplempc_float x12[20];

    /* vector of size 20 */
    simplempc_float x13[20];

    /* vector of size 20 */
    simplempc_float x14[20];

    /* vector of size 20 */
    simplempc_float x15[20];


} simplempc_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    simplempc_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    simplempc_float res_ineq;

	/* norm of stationarity condition */
    simplempc_float rsnorm;

	/* max of all complementarity violations */
    simplempc_float rcompnorm;

    /* primal objective */
    simplempc_float pobj;	
	
    /* dual objective */
    simplempc_float dobj;	

    /* duality gap := pobj - dobj */
    simplempc_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    simplempc_float rdgap;		

    /* duality measure */
    simplempc_float mu;

	/* duality measure (after affine step) */
    simplempc_float mu_aff;
	
    /* centering parameter */
    simplempc_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    simplempc_float step_aff;
    
    /* step size (combined direction) */
    simplempc_float step_cc;    

	/* solvertime */
	simplempc_float solvetime;   

	/* time spent in function evaluations */
	simplempc_float fevalstime;  


} simplempc_info;







/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Monday, October 19, 2020 8:39:19 AM */
/* User License expires on: (UTC) Tuesday, March 23, 2021 10:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Tuesday, March 23, 2021 10:00:00 PM (approx.) */
/* Solver Generation Request Id: 5bb2ee5d-4770-40a0-b49a-5837e717f4aa */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef void (*simplempc_extfunc)(simplempc_float* x, simplempc_float* y, simplempc_float* lambda, simplempc_float* params, simplempc_float* pobj, simplempc_float* g, simplempc_float* c, simplempc_float* Jeq, simplempc_float* h, simplempc_float* Jineq, simplempc_float* H, solver_int32_default stage, solver_int32_default iterations);

extern solver_int32_default simplempc_solve(simplempc_params *params, simplempc_output *output, simplempc_info *info, FILE *fs, simplempc_extfunc evalextfunctions_simplempc);	





#ifdef __cplusplus
}
#endif

#endif
