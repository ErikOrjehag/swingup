/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */



/*

IMPORTANT: This file should serve as a starting point to develop the user
code for the OCP solver. The code below is for illustration purposes. Most
likely you will not get good results if you execute this code without any
modification(s).

Please read the examples in order to understand how to write user code how
to run the OCP solver. You can find more info on the website:
www.acadotoolkit.org

*/

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <stdio.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   1        /* Number of real-time iterations. */
#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
extern ACADOvariables acadoVariables;
extern ACADOworkspace acadoWorkspace;
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

int initMPC()
{
	/* Initialize the solver. */
	acado_initializeSolver();

	int i;

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize lower and upper bounds on controls */
	for (i = 0; i < N; ++i)  acadoVariables.lbValues[ i ] = -0.4;
	for (i = 0; i < N; ++i)  acadoVariables.ubValues[ i ] = 0.4;

	/* Initialize lower and upper bounds on states */
	for (i = 0; i < N; ++i)  acadoVariables.lbAValues[ i ] = -30.0;
	for (i = 0; i < N; ++i)  acadoVariables.ubAValues[ i ] = 30.0;

	//for (i = 0; i < (N + 1); ++i)  acadoVariables.lbAValues[ i ] = -15.0;
	//for (i = 0; i < (N + 1); ++i)  acadoVariables.ubAValues[ i ] = 15.0;

	/* Initialize reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

	/* Initialize weights. */
	acadoVariables.W[0] = 1.0;
	acadoVariables.W[1] = 0.0;
	acadoVariables.W[2] = 0.0;//

	acadoVariables.W[3] = 0.0;
	acadoVariables.W[4] = 0.01;
	acadoVariables.W[5] = 0.0;//

	acadoVariables.W[6] = 0.0;
	acadoVariables.W[7] = 0.0;
	acadoVariables.W[8] = 20.0;//


	acadoVariables.WN[0] = 10.0;
	acadoVariables.WN[1] = 0.0;//

	acadoVariables.WN[2] = 0.0;
	acadoVariables.WN[3] = 1.0;


	printf("Init done, acadoVariables.lbValues[0] = %.3e\n", acadoVariables.lbValues[0]);
}

int runMPC(int numSteps)
{
	int    iter;
	
	/* The "real-time iterations" loop. */
	for(iter = 0; iter < numSteps; ++iter)
	{
		/* Prepare first step */
		acado_preparationStep();

        /* Perform the feedback step. */
		acado_feedbackStep( );

		/* Apply the new control immediately to the process, first NU components. */

		if( VERBOSE ) printf("Real-Time Iteration %d:  KKT Tolerance = %.3e  Objective = %.3e\n", iter, acado_getKKT(), acado_getObjective() );

		/* Optional: shift the initialization (look at acado_common.h). */
        /* acado_shiftStates(2, 0, 0); */
		/* acado_shiftControls( 0 ); */
	}
}

/* A template for testing of the solver. */
int main( )
{
	/* Some temporary variables. */
	acado_timer t;

	/* Initialize problem variables */
	initMPC();

	/* MPC: initialize the current state feedback. */
	acadoVariables.x0[ 0 ] = 3.1415;
	acadoVariables.x0[ 1 ] = 0.0;
	acadoVariables.x0[ 2 ] = 0.0;

	/* Get the time before start of the loop. */
	acado_tic( &t );

	/* Do some RTI MPC */
	runMPC(NUM_STEPS);

	/* Read the elapsed time. */
	real_t te = acado_toc( &t );

	if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

	/* Eye-candy. */

	if( VERBOSE ) printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

	acado_printDifferentialVariables();
	acado_printControlVariables();

    return 0;
}



/*acadoVariables.x[0] = 3.1415926535897931e+00;
	acadoVariables.x[1] = 0.0000000000000000e+00;
	acadoVariables.x[2] = 0.0000000000000000e+00;
	acadoVariables.x[3] = 3.1062949699776583e+00;
	acadoVariables.x[4] = -1.4016234266031440e+00;
	acadoVariables.x[5] = 4.9230771036519760e+00;
	acadoVariables.x[6] = 3.0115687274862006e+00;
	acadoVariables.x[7] = -2.3599193807520114e+00;
	acadoVariables.x[8] = 8.7149447556791344e+00;
	acadoVariables.x[9] = 2.8877872978405095e+00;
	acadoVariables.x[10] = -2.5559423455436030e+00;
	acadoVariables.x[11] = 1.0540548185064168e+01;
	acadoVariables.x[12] = 2.7661710890680342e+00;
	acadoVariables.x[13] = -2.2750290815567542e+00;
	acadoVariables.x[14] = 1.1443758953248825e+01;
	acadoVariables.x[15] = 2.6700903617019947e+00;
	acadoVariables.x[16] = -1.5427063043801035e+00;
	acadoVariables.x[17] = 1.1407871146141842e+01;
	acadoVariables.x[18] = 2.6201597163985926e+00;
	acadoVariables.x[19] = -4.4174906979322004e-01;
	acadoVariables.x[20] = 1.0499388070740615e+01;
	acadoVariables.x[21] = 2.6321433356652384e+00;
	acadoVariables.x[22] = 9.1806213000868475e-01;
	acadoVariables.x[23] = 8.8008474911540695e+00;
	acadoVariables.x[24] = 2.7160191833989971e+00;
	acadoVariables.x[25] = 2.4152028638606748e+00;
	acadoVariables.x[26] = 6.3746744847266346e+00;
	acadoVariables.x[27] = 2.8750598049892968e+00;
	acadoVariables.x[28] = 3.9029171622555978e+00;
	acadoVariables.x[29] = 3.3076885208970905e+00;
	acadoVariables.x[30] = 3.1038747516333909e+00;
	acadoVariables.x[31] = 5.1838969498606033e+00;
	acadoVariables.x[32] = -1.8083509510380671e-01;
	acadoVariables.x[33] = 3.3859594216073372e+00;
	acadoVariables.x[34] = 6.0178774105002137e+00;
	acadoVariables.x[35] = -3.6680327457921420e+00;
	acadoVariables.x[36] = 3.6934485314930976e+00;
	acadoVariables.x[37] = 6.1992837186963738e+00;
	acadoVariables.x[38] = -6.6253201867853724e+00;
	acadoVariables.x[39] = 3.9917629672956592e+00;
	acadoVariables.x[40] = 5.6671113817979526e+00;
	acadoVariables.x[41] = -8.6767027505202901e+00;
	acadoVariables.x[42] = 4.2481159245453703e+00;
	acadoVariables.x[43] = 4.5455652328989329e+00;
	acadoVariables.x[44] = -9.8095890816468572e+00;
	acadoVariables.x[45] = 4.4384811989538875e+00;
	acadoVariables.x[46] = 3.0492657699587702e+00;
	acadoVariables.x[47] = -1.0271150796968099e+01;
	acadoVariables.x[48] = 4.5492809566432646e+00;
	acadoVariables.x[49] = 1.3758687915439403e+00;
	acadoVariables.x[50] = -1.0383951558407789e+01;
	acadoVariables.x[51] = 4.5745569309968959e+00;
	acadoVariables.x[52] = -3.6590073875998919e-01;
	acadoVariables.x[53] = -1.0335771121451030e+01;
	acadoVariables.x[54] = 4.5114217918029365e+00;
	acadoVariables.x[55] = -2.1564886638876795e+00;
	acadoVariables.x[56] = -1.0100572878162581e+01;
	acadoVariables.x[57] = 4.3564040752914384e+00;
	acadoVariables.x[58] = -4.0320280315558179e+00;
	acadoVariables.x[59] = -9.4255410590448854e+00;
	acadoVariables.x[60] = 4.1040866709128707e+00;
	acadoVariables.x[61] = -6.0269850237988516e+00;
	acadoVariables.x[62] = -7.8829606103820096e+00;
	acadoVariables.x[63] = 3.7496563691141658e+00;
	acadoVariables.x[64] = -8.0778270652539810e+00;
	acadoVariables.x[65] = -5.0704071288961794e+00;
	acadoVariables.x[66] = 3.2971710192302650e+00;
	acadoVariables.x[67] = -9.9000954891294981e+00;
	acadoVariables.x[68] = -1.0374517406103454e+00;
	acadoVariables.x[69] = 2.7763924263242625e+00;
	acadoVariables.x[70] = -1.0781120562343032e+01;
	acadoVariables.x[71] = 2.6154020436746723e+00;
	acadoVariables.x[72] = 2.2301391810241071e+00;
	acadoVariables.x[73] = -1.0942130954374919e+01;
	acadoVariables.x[74] = 6.7256898287669751e+00;
	acadoVariables.x[75] = 1.7005787675419961e+00;
	acadoVariables.x[76] = -1.0181708913301389e+01;
	acadoVariables.x[77] = 9.6145355915825554e+00;
	acadoVariables.x[78] = 1.2495551872601551e+00;
	acadoVariables.x[79] = -7.8724654320872753e+00;
	acadoVariables.x[80] = 7.5796897754985464e+00;
	acadoVariables.x[81] = 9.0327078941142258e-01;
	acadoVariables.x[82] = -6.0269494533138275e+00;
	acadoVariables.x[83] = 6.4621591118716966e+00;
	acadoVariables.x[84] = 6.4236341097925509e-01;
	acadoVariables.x[85] = -4.4637530352161514e+00;
	acadoVariables.x[86] = 5.2299324258850000e+00;
	acadoVariables.x[87] = 4.5167741499056563e-01;
	acadoVariables.x[88] = -3.2110608851107876e+00;
	acadoVariables.x[89] = 4.0052723994401100e+00;
	acadoVariables.x[90] = 3.1520514081011863e-01;
	acadoVariables.x[91] = -2.2846221151523136e+00;
	acadoVariables.x[92] = 3.0358007970671408e+00;
	acadoVariables.x[93] = 2.1848100270659571e-01;
	acadoVariables.x[94] = -1.6114583393094688e+00;
	acadoVariables.x[95] = 2.2821508060273454e+00;
	acadoVariables.x[96] = 1.5043743766966669e-01;
	acadoVariables.x[97] = -1.1297207185259583e+00;
	acadoVariables.x[98] = 1.7108659675335192e+00;
	acadoVariables.x[99] = 1.0282878124624259e-01;
	acadoVariables.x[100] = -7.8834778331660527e-01;
	acadoVariables.x[101] = 1.2838134105559471e+00;
	acadoVariables.x[102] = 6.9649925859819800e-02;
	acadoVariables.x[103] = -5.4841061962838766e-01;
	acadoVariables.x[104] = 9.6773262732401943e-01;
	acadoVariables.x[105] = 4.6572807669138148e-02;
	acadoVariables.x[106] = -3.8136766618429085e-01;
	acadoVariables.x[107] = 7.3628015532730462e-01;
	acadoVariables.x[108] = 3.0485682890622177e-02;
	acadoVariables.x[109] = -2.6678780755258041e-01;
	acadoVariables.x[110] = 5.6955389213255359e-01;
	acadoVariables.x[111] = 1.9139343006698329e-02;
	acadoVariables.x[112] = -1.9036132487995941e-01;
	acadoVariables.x[113] = 4.5300741741244788e-01;
	acadoVariables.x[114] = 1.0880889181484114e-02;
	acadoVariables.x[115] = -1.4237595502472428e-01;
	acadoVariables.x[116] = 3.7640147785077038e-01;
	acadoVariables.x[117] = 4.4585229664800526e-03;
	acadoVariables.x[118] = -1.1638458188299720e-01;
	acadoVariables.x[119] = 3.3212594915558052e-01;
	acadoVariables.x[120] = 2.1061387280252186e-15;
	acadoVariables.x[121] = -6.3251707439210575e-02;
	acadoVariables.x[122] = 1.6034011735814926e-01;
	
	acadoVariables.u[0] = 7.9999999999961968e-01;
	acadoVariables.u[1] = 6.1617847085312039e-01;
	acadoVariables.u[2] = 2.9666054639362360e-01;
	acadoVariables.u[3] = 1.4677174444645097e-01;
	acadoVariables.u[4] = -5.8317684409770606e-03;
	acadoVariables.u[5] = -1.4762849433771930e-01;
	acadoVariables.u[6] = -2.7601283405872290e-01;
	acadoVariables.u[7] = -3.9425309908334005e-01;
	acadoVariables.u[8] = -4.9838520084163956e-01;
	acadoVariables.u[9] = -5.6688506680692452e-01;
	acadoVariables.u[10] = -5.6666959745153622e-01;
	acadoVariables.u[11] = -4.8055919153459475e-01;
	acadoVariables.u[12] = -3.3334965437973119e-01;
	acadoVariables.u[13] = -1.8409402205553971e-01;
	acadoVariables.u[14] = -7.5003775988580185e-02;
	acadoVariables.u[15] = -1.8330123061604326e-02;
	acadoVariables.u[16] = 7.8293207182958697e-03;
	acadoVariables.u[17] = 3.8219713132481785e-02;
	acadoVariables.u[18] = 1.0969266658312245e-01;
	acadoVariables.u[19] = 2.5066931371322038e-01;
	acadoVariables.u[20] = 4.5703992397732102e-01;
	acadoVariables.u[21] = 6.5535522655815992e-01;
	acadoVariables.u[22] = 5.9358871817361103e-01;
	acadoVariables.u[23] = 6.6792174057827491e-01;
	acadoVariables.u[24] = 4.6943741923866772e-01;
	acadoVariables.u[25] = -3.3066243298502429e-01;
	acadoVariables.u[26] = -1.8159872617836106e-01;
	acadoVariables.u[27] = -2.0023682912819599e-01;
	acadoVariables.u[28] = -1.9900724699775194e-01;
	acadoVariables.u[29] = -1.5753912960710614e-01;
	acadoVariables.u[30] = -1.2246811905186281e-01;
	acadoVariables.u[31] = -9.2833782850123642e-02;
	acadoVariables.u[32] = -6.9396037963423932e-02;
	acadoVariables.u[33] = -5.1363125391199860e-02;
	acadoVariables.u[34] = -3.7611025319901827e-02;
	acadoVariables.u[35] = -2.7093016775381024e-02;
	acadoVariables.u[36] = -1.8938801447346023e-02;
	acadoVariables.u[37] = -1.2448464722165641e-02;
	acadoVariables.u[38] = -7.1947731490656580e-03;
	acadoVariables.u[39] = -2.7915196643159248e-02;
	acadoVariables.u[40] = -2.7915196643159248e-02;*/
