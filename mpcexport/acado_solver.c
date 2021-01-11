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


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 3 + 2];

acadoWorkspace.state[15] = acadoVariables.u[lRun1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 3] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 3 + 3];
acadoWorkspace.d[lRun1 * 3 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 3 + 4];
acadoWorkspace.d[lRun1 * 3 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 3 + 5];

acadoWorkspace.evGx[lRun1 * 9] = acadoWorkspace.state[3];
acadoWorkspace.evGx[lRun1 * 9 + 1] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 9 + 2] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 9 + 3] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 9 + 4] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 9 + 5] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 9 + 6] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 9 + 7] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 9 + 8] = acadoWorkspace.state[11];

acadoWorkspace.evGu[lRun1 * 3] = acadoWorkspace.state[12];
acadoWorkspace.evGu[lRun1 * 3 + 1] = acadoWorkspace.state[13];
acadoWorkspace.evGu[lRun1 * 3 + 2] = acadoWorkspace.state[14];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[2];
out[2] = u[0];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[2];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = 0.0;
;
tmpQ2[4] = 0.0;
;
tmpQ2[5] = 0.0;
;
tmpQ2[6] = +tmpObjS[3];
tmpQ2[7] = +tmpObjS[4];
tmpQ2[8] = +tmpObjS[5];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = 0.0;
;
tmpQ1[2] = + tmpQ2[1];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = 0.0;
;
tmpQ1[5] = + tmpQ2[4];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = 0.0;
;
tmpQ1[8] = + tmpQ2[7];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[6];
tmpR2[1] = +tmpObjS[7];
tmpR2[2] = +tmpObjS[8];
tmpR1[0] = + tmpR2[2];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = 0.0;
;
tmpQN2[3] = 0.0;
;
tmpQN2[4] = +tmpObjSEndTerm[2];
tmpQN2[5] = +tmpObjSEndTerm[3];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = 0.0;
;
tmpQN1[2] = + tmpQN2[1];
tmpQN1[3] = + tmpQN2[2];
tmpQN1[4] = 0.0;
;
tmpQN1[5] = + tmpQN2[3];
tmpQN1[6] = + tmpQN2[4];
tmpQN1[7] = 0.0;
;
tmpQN1[8] = + tmpQN2[5];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 100; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 3] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 3 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 3 + 2] = acadoWorkspace.objValueOut[2];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 9 ]), &(acadoWorkspace.Q2[ runObj * 9 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj ]), &(acadoWorkspace.R2[ runObj * 3 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[300];
acadoWorkspace.objValueIn[1] = acadoVariables.x[301];
acadoWorkspace.objValueIn[2] = acadoVariables.x[302];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1] + Gx1[2]*Gu1[2];
Gu2[1] = + Gx1[3]*Gu1[0] + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[2];
Gu2[2] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[1] + Gx1[8]*Gu1[2];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 100) + (iCol)] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 101] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + R11[0];
acadoWorkspace.H[iRow * 101] += 1.0000000000000001e-05;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[3]*Gu1[1] + Gx1[6]*Gu1[2];
Gu2[1] = + Gx1[1]*Gu1[0] + Gx1[4]*Gu1[1] + Gx1[7]*Gu1[2];
Gu2[2] = + Gx1[2]*Gu1[0] + Gx1[5]*Gu1[1] + Gx1[8]*Gu1[2];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[1] + Q11[2]*Gu1[2] + Gu2[0];
Gu3[1] = + Q11[3]*Gu1[0] + Q11[4]*Gu1[1] + Q11[5]*Gu1[2] + Gu2[1];
Gu3[2] = + Q11[6]*Gu1[0] + Q11[7]*Gu1[1] + Q11[8]*Gu1[2] + Gu2[2];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[3]*w11[1] + Gx1[6]*w11[2] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[4]*w11[1] + Gx1[7]*w11[2] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[5]*w11[1] + Gx1[8]*w11[2] + w12[2];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[1]*w11[1] + Gu1[2]*w11[2];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + w12[0];
w13[1] = + Q11[3]*w11[0] + Q11[4]*w11[1] + Q11[5]*w11[2] + w12[1];
w13[2] = + Q11[6]*w11[0] + Q11[7]*w11[1] + Q11[8]*w11[2] + w12[2];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2];
w12[1] += + Gx1[3]*w11[0] + Gx1[4]*w11[1] + Gx1[5]*w11[2];
w12[2] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2];
w12[1] += + Gx1[3]*w11[0] + Gx1[4]*w11[1] + Gx1[5]*w11[2];
w12[2] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2];
w12[0] += + Gu1[0]*U1[0];
w12[1] += + Gu1[1]*U1[0];
w12[2] += + Gu1[2]*U1[0];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 100) + (iCol)] = acadoWorkspace.H[(iCol * 100) + (iRow)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2];
QDy1[1] = + Q2[3]*Dy1[0] + Q2[4]*Dy1[1] + Q2[5]*Dy1[2];
QDy1[2] = + Q2[6]*Dy1[0] + Q2[7]*Dy1[1] + Q2[8]*Dy1[2];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 100 */
static const int xBoundIndices[ 100 ] = 
{ 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47, 50, 53, 56, 59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89, 92, 95, 98, 101, 104, 107, 110, 113, 116, 119, 122, 125, 128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 179, 182, 185, 188, 191, 194, 197, 200, 203, 206, 209, 212, 215, 218, 221, 224, 227, 230, 233, 236, 239, 242, 245, 248, 251, 254, 257, 260, 263, 266, 269, 272, 275, 278, 281, 284, 287, 290, 293, 296, 299, 302 };
for (lRun2 = 0; lRun2 < 100; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 201)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 3 ]), &(acadoWorkspace.E[ lRun3 * 3 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 100; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (3)) * (3)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (3)) * (1)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (3)) * (1)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (100)) - (1)) * (3)) * (1)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 99; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 3 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 9 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 9 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (3)) * (1)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 ]), &(acadoWorkspace.evGu[ lRun2 * 3 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun1 = 0; lRun1 < 300; ++lRun1)
acadoWorkspace.sbar[lRun1 + 3] = acadoWorkspace.d[lRun1];


for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 3;
lRun4 = ((lRun3) / (3)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 199)) / (2)) + (lRun4)) - (1)) * (3)) + ((lRun3) % (3));
acadoWorkspace.A[(lRun1 * 100) + (lRun2)] = acadoWorkspace.E[lRun5];
}
}

}

void acado_condenseFdb(  )
{
int lRun1;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
for (lRun1 = 0; lRun1 < 300; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 3 ]), &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 6 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 9 ]), &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 12 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 15 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 18 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 21 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 24 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 27 ]), &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 30 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 33 ]), &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 36 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 39 ]), &(acadoWorkspace.Dy[ 39 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 42 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 45 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 51 ]), &(acadoWorkspace.Dy[ 51 ]), &(acadoWorkspace.g[ 17 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 54 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 57 ]), &(acadoWorkspace.Dy[ 57 ]), &(acadoWorkspace.g[ 19 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 63 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 66 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 69 ]), &(acadoWorkspace.Dy[ 69 ]), &(acadoWorkspace.g[ 23 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 75 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.g[ 25 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 78 ]), &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 81 ]), &(acadoWorkspace.Dy[ 81 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 84 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 87 ]), &(acadoWorkspace.Dy[ 87 ]), &(acadoWorkspace.g[ 29 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 90 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 93 ]), &(acadoWorkspace.Dy[ 93 ]), &(acadoWorkspace.g[ 31 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 96 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 99 ]), &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 102 ]), &(acadoWorkspace.Dy[ 102 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 105 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 35 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 108 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 111 ]), &(acadoWorkspace.Dy[ 111 ]), &(acadoWorkspace.g[ 37 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 114 ]), &(acadoWorkspace.Dy[ 114 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 117 ]), &(acadoWorkspace.Dy[ 117 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 123 ]), &(acadoWorkspace.Dy[ 123 ]), &(acadoWorkspace.g[ 41 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 126 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 129 ]), &(acadoWorkspace.Dy[ 129 ]), &(acadoWorkspace.g[ 43 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 132 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 135 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 138 ]), &(acadoWorkspace.Dy[ 138 ]), &(acadoWorkspace.g[ 46 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 141 ]), &(acadoWorkspace.Dy[ 141 ]), &(acadoWorkspace.g[ 47 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 147 ]), &(acadoWorkspace.Dy[ 147 ]), &(acadoWorkspace.g[ 49 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 150 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.g[ 50 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 153 ]), &(acadoWorkspace.Dy[ 153 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 156 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 159 ]), &(acadoWorkspace.Dy[ 159 ]), &(acadoWorkspace.g[ 53 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 162 ]), &(acadoWorkspace.Dy[ 162 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 165 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.g[ 55 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 168 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 171 ]), &(acadoWorkspace.Dy[ 171 ]), &(acadoWorkspace.g[ 57 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 174 ]), &(acadoWorkspace.Dy[ 174 ]), &(acadoWorkspace.g[ 58 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 177 ]), &(acadoWorkspace.Dy[ 177 ]), &(acadoWorkspace.g[ 59 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 183 ]), &(acadoWorkspace.Dy[ 183 ]), &(acadoWorkspace.g[ 61 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 186 ]), &(acadoWorkspace.Dy[ 186 ]), &(acadoWorkspace.g[ 62 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 189 ]), &(acadoWorkspace.Dy[ 189 ]), &(acadoWorkspace.g[ 63 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 192 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 195 ]), &(acadoWorkspace.Dy[ 195 ]), &(acadoWorkspace.g[ 65 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 198 ]), &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.g[ 66 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 201 ]), &(acadoWorkspace.Dy[ 201 ]), &(acadoWorkspace.g[ 67 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 204 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 207 ]), &(acadoWorkspace.Dy[ 207 ]), &(acadoWorkspace.g[ 69 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 210 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.g[ 70 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 213 ]), &(acadoWorkspace.Dy[ 213 ]), &(acadoWorkspace.g[ 71 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 216 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 219 ]), &(acadoWorkspace.Dy[ 219 ]), &(acadoWorkspace.g[ 73 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 222 ]), &(acadoWorkspace.Dy[ 222 ]), &(acadoWorkspace.g[ 74 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 225 ]), &(acadoWorkspace.Dy[ 225 ]), &(acadoWorkspace.g[ 75 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 228 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.g[ 76 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 231 ]), &(acadoWorkspace.Dy[ 231 ]), &(acadoWorkspace.g[ 77 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 234 ]), &(acadoWorkspace.Dy[ 234 ]), &(acadoWorkspace.g[ 78 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 237 ]), &(acadoWorkspace.Dy[ 237 ]), &(acadoWorkspace.g[ 79 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 80 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 243 ]), &(acadoWorkspace.Dy[ 243 ]), &(acadoWorkspace.g[ 81 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 246 ]), &(acadoWorkspace.Dy[ 246 ]), &(acadoWorkspace.g[ 82 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 249 ]), &(acadoWorkspace.Dy[ 249 ]), &(acadoWorkspace.g[ 83 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 252 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.g[ 84 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 255 ]), &(acadoWorkspace.Dy[ 255 ]), &(acadoWorkspace.g[ 85 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 258 ]), &(acadoWorkspace.Dy[ 258 ]), &(acadoWorkspace.g[ 86 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 261 ]), &(acadoWorkspace.Dy[ 261 ]), &(acadoWorkspace.g[ 87 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 264 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.g[ 88 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 267 ]), &(acadoWorkspace.Dy[ 267 ]), &(acadoWorkspace.g[ 89 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 270 ]), &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.g[ 90 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 273 ]), &(acadoWorkspace.Dy[ 273 ]), &(acadoWorkspace.g[ 91 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 276 ]), &(acadoWorkspace.Dy[ 276 ]), &(acadoWorkspace.g[ 92 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 279 ]), &(acadoWorkspace.Dy[ 279 ]), &(acadoWorkspace.g[ 93 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 282 ]), &(acadoWorkspace.Dy[ 282 ]), &(acadoWorkspace.g[ 94 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 285 ]), &(acadoWorkspace.Dy[ 285 ]), &(acadoWorkspace.g[ 95 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.g[ 96 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 291 ]), &(acadoWorkspace.Dy[ 291 ]), &(acadoWorkspace.g[ 97 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 294 ]), &(acadoWorkspace.Dy[ 294 ]), &(acadoWorkspace.g[ 98 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 297 ]), &(acadoWorkspace.Dy[ 297 ]), &(acadoWorkspace.g[ 99 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 9 ]), &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.QDy[ 3 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 18 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 27 ]), &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 36 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 45 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 54 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 63 ]), &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 72 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 81 ]), &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 90 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 99 ]), &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.QDy[ 33 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 108 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 117 ]), &(acadoWorkspace.Dy[ 39 ]), &(acadoWorkspace.QDy[ 39 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 126 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 135 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 144 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 153 ]), &(acadoWorkspace.Dy[ 51 ]), &(acadoWorkspace.QDy[ 51 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 162 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 171 ]), &(acadoWorkspace.Dy[ 57 ]), &(acadoWorkspace.QDy[ 57 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 180 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 189 ]), &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 198 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.QDy[ 66 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 207 ]), &(acadoWorkspace.Dy[ 69 ]), &(acadoWorkspace.QDy[ 69 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 216 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 225 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.QDy[ 75 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 234 ]), &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.QDy[ 78 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 243 ]), &(acadoWorkspace.Dy[ 81 ]), &(acadoWorkspace.QDy[ 81 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 252 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 261 ]), &(acadoWorkspace.Dy[ 87 ]), &(acadoWorkspace.QDy[ 87 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 270 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 279 ]), &(acadoWorkspace.Dy[ 93 ]), &(acadoWorkspace.QDy[ 93 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 288 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 297 ]), &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.QDy[ 99 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 306 ]), &(acadoWorkspace.Dy[ 102 ]), &(acadoWorkspace.QDy[ 102 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 315 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 105 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 324 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 333 ]), &(acadoWorkspace.Dy[ 111 ]), &(acadoWorkspace.QDy[ 111 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 342 ]), &(acadoWorkspace.Dy[ 114 ]), &(acadoWorkspace.QDy[ 114 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 351 ]), &(acadoWorkspace.Dy[ 117 ]), &(acadoWorkspace.QDy[ 117 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 360 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 369 ]), &(acadoWorkspace.Dy[ 123 ]), &(acadoWorkspace.QDy[ 123 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 378 ]), &(acadoWorkspace.Dy[ 126 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 387 ]), &(acadoWorkspace.Dy[ 129 ]), &(acadoWorkspace.QDy[ 129 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 396 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 132 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 405 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 414 ]), &(acadoWorkspace.Dy[ 138 ]), &(acadoWorkspace.QDy[ 138 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 423 ]), &(acadoWorkspace.Dy[ 141 ]), &(acadoWorkspace.QDy[ 141 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 432 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 441 ]), &(acadoWorkspace.Dy[ 147 ]), &(acadoWorkspace.QDy[ 147 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 450 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.QDy[ 150 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 459 ]), &(acadoWorkspace.Dy[ 153 ]), &(acadoWorkspace.QDy[ 153 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 468 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.QDy[ 156 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 477 ]), &(acadoWorkspace.Dy[ 159 ]), &(acadoWorkspace.QDy[ 159 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 486 ]), &(acadoWorkspace.Dy[ 162 ]), &(acadoWorkspace.QDy[ 162 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 495 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.QDy[ 165 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 504 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 168 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 513 ]), &(acadoWorkspace.Dy[ 171 ]), &(acadoWorkspace.QDy[ 171 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 522 ]), &(acadoWorkspace.Dy[ 174 ]), &(acadoWorkspace.QDy[ 174 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 531 ]), &(acadoWorkspace.Dy[ 177 ]), &(acadoWorkspace.QDy[ 177 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 180 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 549 ]), &(acadoWorkspace.Dy[ 183 ]), &(acadoWorkspace.QDy[ 183 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 558 ]), &(acadoWorkspace.Dy[ 186 ]), &(acadoWorkspace.QDy[ 186 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 567 ]), &(acadoWorkspace.Dy[ 189 ]), &(acadoWorkspace.QDy[ 189 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 576 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.QDy[ 192 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 585 ]), &(acadoWorkspace.Dy[ 195 ]), &(acadoWorkspace.QDy[ 195 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 594 ]), &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.QDy[ 198 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 603 ]), &(acadoWorkspace.Dy[ 201 ]), &(acadoWorkspace.QDy[ 201 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 612 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.QDy[ 204 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 621 ]), &(acadoWorkspace.Dy[ 207 ]), &(acadoWorkspace.QDy[ 207 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 630 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.QDy[ 210 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 639 ]), &(acadoWorkspace.Dy[ 213 ]), &(acadoWorkspace.QDy[ 213 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 648 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.QDy[ 216 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 657 ]), &(acadoWorkspace.Dy[ 219 ]), &(acadoWorkspace.QDy[ 219 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 666 ]), &(acadoWorkspace.Dy[ 222 ]), &(acadoWorkspace.QDy[ 222 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 675 ]), &(acadoWorkspace.Dy[ 225 ]), &(acadoWorkspace.QDy[ 225 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 684 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.QDy[ 228 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 693 ]), &(acadoWorkspace.Dy[ 231 ]), &(acadoWorkspace.QDy[ 231 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 702 ]), &(acadoWorkspace.Dy[ 234 ]), &(acadoWorkspace.QDy[ 234 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 711 ]), &(acadoWorkspace.Dy[ 237 ]), &(acadoWorkspace.QDy[ 237 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 240 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 729 ]), &(acadoWorkspace.Dy[ 243 ]), &(acadoWorkspace.QDy[ 243 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 738 ]), &(acadoWorkspace.Dy[ 246 ]), &(acadoWorkspace.QDy[ 246 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 747 ]), &(acadoWorkspace.Dy[ 249 ]), &(acadoWorkspace.QDy[ 249 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 756 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.QDy[ 252 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 765 ]), &(acadoWorkspace.Dy[ 255 ]), &(acadoWorkspace.QDy[ 255 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 774 ]), &(acadoWorkspace.Dy[ 258 ]), &(acadoWorkspace.QDy[ 258 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 783 ]), &(acadoWorkspace.Dy[ 261 ]), &(acadoWorkspace.QDy[ 261 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 792 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.QDy[ 264 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 801 ]), &(acadoWorkspace.Dy[ 267 ]), &(acadoWorkspace.QDy[ 267 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 810 ]), &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.QDy[ 270 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 819 ]), &(acadoWorkspace.Dy[ 273 ]), &(acadoWorkspace.QDy[ 273 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 828 ]), &(acadoWorkspace.Dy[ 276 ]), &(acadoWorkspace.QDy[ 276 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 837 ]), &(acadoWorkspace.Dy[ 279 ]), &(acadoWorkspace.QDy[ 279 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 846 ]), &(acadoWorkspace.Dy[ 282 ]), &(acadoWorkspace.QDy[ 282 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 855 ]), &(acadoWorkspace.Dy[ 285 ]), &(acadoWorkspace.QDy[ 285 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 864 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.QDy[ 288 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 873 ]), &(acadoWorkspace.Dy[ 291 ]), &(acadoWorkspace.QDy[ 291 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 882 ]), &(acadoWorkspace.Dy[ 294 ]), &(acadoWorkspace.QDy[ 294 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 891 ]), &(acadoWorkspace.Dy[ 297 ]), &(acadoWorkspace.QDy[ 297 ]) );

acadoWorkspace.QDy[300] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[301] = + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[302] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 3 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 9 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 33 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 51 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 57 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 198 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 69 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 207 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 75 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 234 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 87 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 261 ]), &(acadoWorkspace.sbar[ 87 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 93 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 279 ]), &(acadoWorkspace.sbar[ 93 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 306 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 315 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 111 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 333 ]), &(acadoWorkspace.sbar[ 111 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 342 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 123 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 369 ]), &(acadoWorkspace.sbar[ 123 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 129 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 387 ]), &(acadoWorkspace.sbar[ 129 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 138 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 414 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.sbar[ 141 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 423 ]), &(acadoWorkspace.sbar[ 141 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 147 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.sbar[ 150 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 450 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 459 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 159 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 477 ]), &(acadoWorkspace.sbar[ 159 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 165 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 495 ]), &(acadoWorkspace.sbar[ 165 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 513 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 174 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 522 ]), &(acadoWorkspace.sbar[ 174 ]), &(acadoWorkspace.sbar[ 177 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 531 ]), &(acadoWorkspace.sbar[ 177 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 183 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 549 ]), &(acadoWorkspace.sbar[ 183 ]), &(acadoWorkspace.sbar[ 186 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 558 ]), &(acadoWorkspace.sbar[ 186 ]), &(acadoWorkspace.sbar[ 189 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.sbar[ 192 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.sbar[ 195 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 585 ]), &(acadoWorkspace.sbar[ 195 ]), &(acadoWorkspace.sbar[ 198 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 594 ]), &(acadoWorkspace.sbar[ 198 ]), &(acadoWorkspace.sbar[ 201 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 603 ]), &(acadoWorkspace.sbar[ 201 ]), &(acadoWorkspace.sbar[ 204 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.sbar[ 204 ]), &(acadoWorkspace.sbar[ 207 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 621 ]), &(acadoWorkspace.sbar[ 207 ]), &(acadoWorkspace.sbar[ 210 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 630 ]), &(acadoWorkspace.sbar[ 210 ]), &(acadoWorkspace.sbar[ 213 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 639 ]), &(acadoWorkspace.sbar[ 213 ]), &(acadoWorkspace.sbar[ 216 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.sbar[ 219 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 657 ]), &(acadoWorkspace.sbar[ 219 ]), &(acadoWorkspace.sbar[ 222 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 666 ]), &(acadoWorkspace.sbar[ 222 ]), &(acadoWorkspace.sbar[ 225 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 675 ]), &(acadoWorkspace.sbar[ 225 ]), &(acadoWorkspace.sbar[ 228 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.sbar[ 228 ]), &(acadoWorkspace.sbar[ 231 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 693 ]), &(acadoWorkspace.sbar[ 231 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 702 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 237 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 711 ]), &(acadoWorkspace.sbar[ 237 ]), &(acadoWorkspace.sbar[ 240 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.sbar[ 240 ]), &(acadoWorkspace.sbar[ 243 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.sbar[ 243 ]), &(acadoWorkspace.sbar[ 246 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 738 ]), &(acadoWorkspace.sbar[ 246 ]), &(acadoWorkspace.sbar[ 249 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 747 ]), &(acadoWorkspace.sbar[ 249 ]), &(acadoWorkspace.sbar[ 252 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 756 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.sbar[ 255 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 765 ]), &(acadoWorkspace.sbar[ 255 ]), &(acadoWorkspace.sbar[ 258 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 774 ]), &(acadoWorkspace.sbar[ 258 ]), &(acadoWorkspace.sbar[ 261 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 783 ]), &(acadoWorkspace.sbar[ 261 ]), &(acadoWorkspace.sbar[ 264 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 792 ]), &(acadoWorkspace.sbar[ 264 ]), &(acadoWorkspace.sbar[ 267 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 801 ]), &(acadoWorkspace.sbar[ 267 ]), &(acadoWorkspace.sbar[ 270 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.sbar[ 270 ]), &(acadoWorkspace.sbar[ 273 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 819 ]), &(acadoWorkspace.sbar[ 273 ]), &(acadoWorkspace.sbar[ 276 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 828 ]), &(acadoWorkspace.sbar[ 276 ]), &(acadoWorkspace.sbar[ 279 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 837 ]), &(acadoWorkspace.sbar[ 279 ]), &(acadoWorkspace.sbar[ 282 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 846 ]), &(acadoWorkspace.sbar[ 282 ]), &(acadoWorkspace.sbar[ 285 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 855 ]), &(acadoWorkspace.sbar[ 285 ]), &(acadoWorkspace.sbar[ 288 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.sbar[ 288 ]), &(acadoWorkspace.sbar[ 291 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 873 ]), &(acadoWorkspace.sbar[ 291 ]), &(acadoWorkspace.sbar[ 294 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 882 ]), &(acadoWorkspace.sbar[ 294 ]), &(acadoWorkspace.sbar[ 297 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.sbar[ 297 ]), &(acadoWorkspace.sbar[ 300 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[300] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[301] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[302] + acadoWorkspace.QDy[300];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[300] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[301] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[302] + acadoWorkspace.QDy[301];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[300] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[301] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[302] + acadoWorkspace.QDy[302];
acado_macBTw1( &(acadoWorkspace.evGu[ 297 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 99 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 891 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 297 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 891 ]), &(acadoWorkspace.sbar[ 297 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 294 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 98 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 882 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 294 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 882 ]), &(acadoWorkspace.sbar[ 294 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 291 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 97 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 873 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 291 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 873 ]), &(acadoWorkspace.sbar[ 291 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 96 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 288 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 864 ]), &(acadoWorkspace.sbar[ 288 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 285 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 95 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 855 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 285 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 855 ]), &(acadoWorkspace.sbar[ 285 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 282 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 94 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 846 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 282 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 846 ]), &(acadoWorkspace.sbar[ 282 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 279 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 93 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 837 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 279 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 837 ]), &(acadoWorkspace.sbar[ 279 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 276 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 92 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 828 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 276 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 828 ]), &(acadoWorkspace.sbar[ 276 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 273 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 91 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 819 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 273 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 819 ]), &(acadoWorkspace.sbar[ 273 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 90 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 810 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 270 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 810 ]), &(acadoWorkspace.sbar[ 270 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 267 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 89 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 801 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 267 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 801 ]), &(acadoWorkspace.sbar[ 267 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 264 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 88 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 792 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 264 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 792 ]), &(acadoWorkspace.sbar[ 264 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 261 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 87 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 783 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 261 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 783 ]), &(acadoWorkspace.sbar[ 261 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 258 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 86 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 774 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 258 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 774 ]), &(acadoWorkspace.sbar[ 258 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 255 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 85 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 765 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 255 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 765 ]), &(acadoWorkspace.sbar[ 255 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 84 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 756 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 252 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 756 ]), &(acadoWorkspace.sbar[ 252 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 249 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 83 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 747 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 249 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 747 ]), &(acadoWorkspace.sbar[ 249 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 246 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 82 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 738 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 246 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 738 ]), &(acadoWorkspace.sbar[ 246 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 81 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 729 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 243 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.sbar[ 243 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 80 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 240 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.sbar[ 240 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 237 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 79 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 711 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 237 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 711 ]), &(acadoWorkspace.sbar[ 237 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 234 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 78 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 702 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 234 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 702 ]), &(acadoWorkspace.sbar[ 234 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 231 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 77 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 693 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 231 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 693 ]), &(acadoWorkspace.sbar[ 231 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 228 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 76 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 684 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 228 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 684 ]), &(acadoWorkspace.sbar[ 228 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 225 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 75 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 675 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 225 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 675 ]), &(acadoWorkspace.sbar[ 225 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 222 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 74 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 666 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 222 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 666 ]), &(acadoWorkspace.sbar[ 222 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 219 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 73 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 657 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 219 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 657 ]), &(acadoWorkspace.sbar[ 219 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 216 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 216 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 213 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 71 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 639 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 213 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 639 ]), &(acadoWorkspace.sbar[ 213 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 210 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 70 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 630 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 210 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 630 ]), &(acadoWorkspace.sbar[ 210 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 207 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 69 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 621 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 207 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 621 ]), &(acadoWorkspace.sbar[ 207 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 204 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 68 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 612 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 204 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 612 ]), &(acadoWorkspace.sbar[ 204 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 201 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 67 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 603 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 201 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 603 ]), &(acadoWorkspace.sbar[ 201 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 198 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 66 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 594 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 198 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 594 ]), &(acadoWorkspace.sbar[ 198 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 195 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 65 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 585 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 195 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 585 ]), &(acadoWorkspace.sbar[ 195 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 64 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 192 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 192 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 63 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 567 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 189 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.sbar[ 189 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 186 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 62 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 558 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 186 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 558 ]), &(acadoWorkspace.sbar[ 186 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 183 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 61 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 549 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 183 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 549 ]), &(acadoWorkspace.sbar[ 183 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 540 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 180 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 540 ]), &(acadoWorkspace.sbar[ 180 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 177 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 59 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 531 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 177 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 531 ]), &(acadoWorkspace.sbar[ 177 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 174 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 58 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 522 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 174 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 522 ]), &(acadoWorkspace.sbar[ 174 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 171 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 57 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 513 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 171 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 513 ]), &(acadoWorkspace.sbar[ 171 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 168 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 504 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 168 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 504 ]), &(acadoWorkspace.sbar[ 168 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 165 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 55 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 495 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 165 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 495 ]), &(acadoWorkspace.sbar[ 165 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 162 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.sbar[ 162 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 159 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 53 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 477 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 159 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 477 ]), &(acadoWorkspace.sbar[ 159 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 156 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 468 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 156 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 468 ]), &(acadoWorkspace.sbar[ 156 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 153 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 51 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 459 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 153 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 459 ]), &(acadoWorkspace.sbar[ 153 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 150 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 50 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 450 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 150 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 450 ]), &(acadoWorkspace.sbar[ 150 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 147 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 49 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 441 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 147 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 441 ]), &(acadoWorkspace.sbar[ 147 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 141 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 47 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 423 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 141 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 423 ]), &(acadoWorkspace.sbar[ 141 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 138 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 46 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 414 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 138 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 414 ]), &(acadoWorkspace.sbar[ 138 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 45 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 135 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 132 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 396 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 132 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 396 ]), &(acadoWorkspace.sbar[ 132 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 129 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 43 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 387 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 129 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 387 ]), &(acadoWorkspace.sbar[ 129 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 126 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 378 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 123 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 41 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 369 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 123 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 369 ]), &(acadoWorkspace.sbar[ 123 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 360 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 117 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 351 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 114 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 38 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 342 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 114 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 342 ]), &(acadoWorkspace.sbar[ 114 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 111 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 37 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 333 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 111 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 333 ]), &(acadoWorkspace.sbar[ 111 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 105 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 35 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 315 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 105 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 315 ]), &(acadoWorkspace.sbar[ 105 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 102 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 34 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 306 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 102 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 306 ]), &(acadoWorkspace.sbar[ 102 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 99 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 297 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 99 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 93 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 31 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 279 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 93 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 279 ]), &(acadoWorkspace.sbar[ 93 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 87 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 29 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 261 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 87 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 261 ]), &(acadoWorkspace.sbar[ 87 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 84 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 252 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 81 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 78 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 26 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 234 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 78 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 234 ]), &(acadoWorkspace.sbar[ 78 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 75 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 25 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 75 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.sbar[ 75 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 69 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 23 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 207 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 69 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 207 ]), &(acadoWorkspace.sbar[ 69 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 66 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 22 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 198 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 66 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 198 ]), &(acadoWorkspace.sbar[ 66 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 63 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 57 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 19 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 171 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 57 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.sbar[ 57 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 51 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 17 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 153 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 51 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.sbar[ 51 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 45 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 42 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 126 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.sbar[ 42 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 39 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 13 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 117 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 39 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.sbar[ 39 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 33 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 11 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 99 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 33 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.sbar[ 33 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 21 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 7 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 63 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 21 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.sbar[ 21 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 15 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 5 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 45 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.sbar[ 15 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 9 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 3 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 1 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 3 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 9 ]), &(acadoWorkspace.sbar[ 3 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.lb[60] = acadoVariables.lbValues[60] - acadoVariables.u[60];
acadoWorkspace.lb[61] = acadoVariables.lbValues[61] - acadoVariables.u[61];
acadoWorkspace.lb[62] = acadoVariables.lbValues[62] - acadoVariables.u[62];
acadoWorkspace.lb[63] = acadoVariables.lbValues[63] - acadoVariables.u[63];
acadoWorkspace.lb[64] = acadoVariables.lbValues[64] - acadoVariables.u[64];
acadoWorkspace.lb[65] = acadoVariables.lbValues[65] - acadoVariables.u[65];
acadoWorkspace.lb[66] = acadoVariables.lbValues[66] - acadoVariables.u[66];
acadoWorkspace.lb[67] = acadoVariables.lbValues[67] - acadoVariables.u[67];
acadoWorkspace.lb[68] = acadoVariables.lbValues[68] - acadoVariables.u[68];
acadoWorkspace.lb[69] = acadoVariables.lbValues[69] - acadoVariables.u[69];
acadoWorkspace.lb[70] = acadoVariables.lbValues[70] - acadoVariables.u[70];
acadoWorkspace.lb[71] = acadoVariables.lbValues[71] - acadoVariables.u[71];
acadoWorkspace.lb[72] = acadoVariables.lbValues[72] - acadoVariables.u[72];
acadoWorkspace.lb[73] = acadoVariables.lbValues[73] - acadoVariables.u[73];
acadoWorkspace.lb[74] = acadoVariables.lbValues[74] - acadoVariables.u[74];
acadoWorkspace.lb[75] = acadoVariables.lbValues[75] - acadoVariables.u[75];
acadoWorkspace.lb[76] = acadoVariables.lbValues[76] - acadoVariables.u[76];
acadoWorkspace.lb[77] = acadoVariables.lbValues[77] - acadoVariables.u[77];
acadoWorkspace.lb[78] = acadoVariables.lbValues[78] - acadoVariables.u[78];
acadoWorkspace.lb[79] = acadoVariables.lbValues[79] - acadoVariables.u[79];
acadoWorkspace.lb[80] = acadoVariables.lbValues[80] - acadoVariables.u[80];
acadoWorkspace.lb[81] = acadoVariables.lbValues[81] - acadoVariables.u[81];
acadoWorkspace.lb[82] = acadoVariables.lbValues[82] - acadoVariables.u[82];
acadoWorkspace.lb[83] = acadoVariables.lbValues[83] - acadoVariables.u[83];
acadoWorkspace.lb[84] = acadoVariables.lbValues[84] - acadoVariables.u[84];
acadoWorkspace.lb[85] = acadoVariables.lbValues[85] - acadoVariables.u[85];
acadoWorkspace.lb[86] = acadoVariables.lbValues[86] - acadoVariables.u[86];
acadoWorkspace.lb[87] = acadoVariables.lbValues[87] - acadoVariables.u[87];
acadoWorkspace.lb[88] = acadoVariables.lbValues[88] - acadoVariables.u[88];
acadoWorkspace.lb[89] = acadoVariables.lbValues[89] - acadoVariables.u[89];
acadoWorkspace.lb[90] = acadoVariables.lbValues[90] - acadoVariables.u[90];
acadoWorkspace.lb[91] = acadoVariables.lbValues[91] - acadoVariables.u[91];
acadoWorkspace.lb[92] = acadoVariables.lbValues[92] - acadoVariables.u[92];
acadoWorkspace.lb[93] = acadoVariables.lbValues[93] - acadoVariables.u[93];
acadoWorkspace.lb[94] = acadoVariables.lbValues[94] - acadoVariables.u[94];
acadoWorkspace.lb[95] = acadoVariables.lbValues[95] - acadoVariables.u[95];
acadoWorkspace.lb[96] = acadoVariables.lbValues[96] - acadoVariables.u[96];
acadoWorkspace.lb[97] = acadoVariables.lbValues[97] - acadoVariables.u[97];
acadoWorkspace.lb[98] = acadoVariables.lbValues[98] - acadoVariables.u[98];
acadoWorkspace.lb[99] = acadoVariables.lbValues[99] - acadoVariables.u[99];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[60] = acadoVariables.ubValues[60] - acadoVariables.u[60];
acadoWorkspace.ub[61] = acadoVariables.ubValues[61] - acadoVariables.u[61];
acadoWorkspace.ub[62] = acadoVariables.ubValues[62] - acadoVariables.u[62];
acadoWorkspace.ub[63] = acadoVariables.ubValues[63] - acadoVariables.u[63];
acadoWorkspace.ub[64] = acadoVariables.ubValues[64] - acadoVariables.u[64];
acadoWorkspace.ub[65] = acadoVariables.ubValues[65] - acadoVariables.u[65];
acadoWorkspace.ub[66] = acadoVariables.ubValues[66] - acadoVariables.u[66];
acadoWorkspace.ub[67] = acadoVariables.ubValues[67] - acadoVariables.u[67];
acadoWorkspace.ub[68] = acadoVariables.ubValues[68] - acadoVariables.u[68];
acadoWorkspace.ub[69] = acadoVariables.ubValues[69] - acadoVariables.u[69];
acadoWorkspace.ub[70] = acadoVariables.ubValues[70] - acadoVariables.u[70];
acadoWorkspace.ub[71] = acadoVariables.ubValues[71] - acadoVariables.u[71];
acadoWorkspace.ub[72] = acadoVariables.ubValues[72] - acadoVariables.u[72];
acadoWorkspace.ub[73] = acadoVariables.ubValues[73] - acadoVariables.u[73];
acadoWorkspace.ub[74] = acadoVariables.ubValues[74] - acadoVariables.u[74];
acadoWorkspace.ub[75] = acadoVariables.ubValues[75] - acadoVariables.u[75];
acadoWorkspace.ub[76] = acadoVariables.ubValues[76] - acadoVariables.u[76];
acadoWorkspace.ub[77] = acadoVariables.ubValues[77] - acadoVariables.u[77];
acadoWorkspace.ub[78] = acadoVariables.ubValues[78] - acadoVariables.u[78];
acadoWorkspace.ub[79] = acadoVariables.ubValues[79] - acadoVariables.u[79];
acadoWorkspace.ub[80] = acadoVariables.ubValues[80] - acadoVariables.u[80];
acadoWorkspace.ub[81] = acadoVariables.ubValues[81] - acadoVariables.u[81];
acadoWorkspace.ub[82] = acadoVariables.ubValues[82] - acadoVariables.u[82];
acadoWorkspace.ub[83] = acadoVariables.ubValues[83] - acadoVariables.u[83];
acadoWorkspace.ub[84] = acadoVariables.ubValues[84] - acadoVariables.u[84];
acadoWorkspace.ub[85] = acadoVariables.ubValues[85] - acadoVariables.u[85];
acadoWorkspace.ub[86] = acadoVariables.ubValues[86] - acadoVariables.u[86];
acadoWorkspace.ub[87] = acadoVariables.ubValues[87] - acadoVariables.u[87];
acadoWorkspace.ub[88] = acadoVariables.ubValues[88] - acadoVariables.u[88];
acadoWorkspace.ub[89] = acadoVariables.ubValues[89] - acadoVariables.u[89];
acadoWorkspace.ub[90] = acadoVariables.ubValues[90] - acadoVariables.u[90];
acadoWorkspace.ub[91] = acadoVariables.ubValues[91] - acadoVariables.u[91];
acadoWorkspace.ub[92] = acadoVariables.ubValues[92] - acadoVariables.u[92];
acadoWorkspace.ub[93] = acadoVariables.ubValues[93] - acadoVariables.u[93];
acadoWorkspace.ub[94] = acadoVariables.ubValues[94] - acadoVariables.u[94];
acadoWorkspace.ub[95] = acadoVariables.ubValues[95] - acadoVariables.u[95];
acadoWorkspace.ub[96] = acadoVariables.ubValues[96] - acadoVariables.u[96];
acadoWorkspace.ub[97] = acadoVariables.ubValues[97] - acadoVariables.u[97];
acadoWorkspace.ub[98] = acadoVariables.ubValues[98] - acadoVariables.u[98];
acadoWorkspace.ub[99] = acadoVariables.ubValues[99] - acadoVariables.u[99];

tmp = acadoWorkspace.sbar[5] + acadoVariables.x[5];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - tmp;
acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - tmp;
tmp = acadoWorkspace.sbar[8] + acadoVariables.x[8];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - tmp;
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - tmp;
tmp = acadoWorkspace.sbar[11] + acadoVariables.x[11];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - tmp;
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - tmp;
tmp = acadoWorkspace.sbar[14] + acadoVariables.x[14];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - tmp;
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - tmp;
tmp = acadoWorkspace.sbar[17] + acadoVariables.x[17];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - tmp;
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - tmp;
tmp = acadoWorkspace.sbar[20] + acadoVariables.x[20];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - tmp;
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - tmp;
tmp = acadoWorkspace.sbar[23] + acadoVariables.x[23];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - tmp;
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - tmp;
tmp = acadoWorkspace.sbar[26] + acadoVariables.x[26];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - tmp;
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - tmp;
tmp = acadoWorkspace.sbar[29] + acadoVariables.x[29];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - tmp;
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - tmp;
tmp = acadoWorkspace.sbar[32] + acadoVariables.x[32];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - tmp;
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - tmp;
tmp = acadoWorkspace.sbar[35] + acadoVariables.x[35];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - tmp;
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - tmp;
tmp = acadoWorkspace.sbar[38] + acadoVariables.x[38];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - tmp;
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - tmp;
tmp = acadoWorkspace.sbar[41] + acadoVariables.x[41];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - tmp;
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - tmp;
tmp = acadoWorkspace.sbar[44] + acadoVariables.x[44];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - tmp;
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - tmp;
tmp = acadoWorkspace.sbar[47] + acadoVariables.x[47];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - tmp;
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - tmp;
tmp = acadoWorkspace.sbar[50] + acadoVariables.x[50];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - tmp;
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - tmp;
tmp = acadoWorkspace.sbar[53] + acadoVariables.x[53];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - tmp;
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - tmp;
tmp = acadoWorkspace.sbar[56] + acadoVariables.x[56];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - tmp;
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - tmp;
tmp = acadoWorkspace.sbar[59] + acadoVariables.x[59];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - tmp;
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - tmp;
tmp = acadoWorkspace.sbar[62] + acadoVariables.x[62];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - tmp;
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - tmp;
tmp = acadoWorkspace.sbar[65] + acadoVariables.x[65];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - tmp;
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - tmp;
tmp = acadoWorkspace.sbar[68] + acadoVariables.x[68];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - tmp;
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - tmp;
tmp = acadoWorkspace.sbar[71] + acadoVariables.x[71];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - tmp;
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - tmp;
tmp = acadoWorkspace.sbar[74] + acadoVariables.x[74];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - tmp;
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - tmp;
tmp = acadoWorkspace.sbar[77] + acadoVariables.x[77];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - tmp;
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - tmp;
tmp = acadoWorkspace.sbar[80] + acadoVariables.x[80];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - tmp;
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - tmp;
tmp = acadoWorkspace.sbar[83] + acadoVariables.x[83];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - tmp;
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - tmp;
tmp = acadoWorkspace.sbar[86] + acadoVariables.x[86];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - tmp;
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - tmp;
tmp = acadoWorkspace.sbar[89] + acadoVariables.x[89];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - tmp;
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - tmp;
tmp = acadoWorkspace.sbar[92] + acadoVariables.x[92];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - tmp;
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - tmp;
tmp = acadoWorkspace.sbar[95] + acadoVariables.x[95];
acadoWorkspace.lbA[30] = acadoVariables.lbAValues[30] - tmp;
acadoWorkspace.ubA[30] = acadoVariables.ubAValues[30] - tmp;
tmp = acadoWorkspace.sbar[98] + acadoVariables.x[98];
acadoWorkspace.lbA[31] = acadoVariables.lbAValues[31] - tmp;
acadoWorkspace.ubA[31] = acadoVariables.ubAValues[31] - tmp;
tmp = acadoWorkspace.sbar[101] + acadoVariables.x[101];
acadoWorkspace.lbA[32] = acadoVariables.lbAValues[32] - tmp;
acadoWorkspace.ubA[32] = acadoVariables.ubAValues[32] - tmp;
tmp = acadoWorkspace.sbar[104] + acadoVariables.x[104];
acadoWorkspace.lbA[33] = acadoVariables.lbAValues[33] - tmp;
acadoWorkspace.ubA[33] = acadoVariables.ubAValues[33] - tmp;
tmp = acadoWorkspace.sbar[107] + acadoVariables.x[107];
acadoWorkspace.lbA[34] = acadoVariables.lbAValues[34] - tmp;
acadoWorkspace.ubA[34] = acadoVariables.ubAValues[34] - tmp;
tmp = acadoWorkspace.sbar[110] + acadoVariables.x[110];
acadoWorkspace.lbA[35] = acadoVariables.lbAValues[35] - tmp;
acadoWorkspace.ubA[35] = acadoVariables.ubAValues[35] - tmp;
tmp = acadoWorkspace.sbar[113] + acadoVariables.x[113];
acadoWorkspace.lbA[36] = acadoVariables.lbAValues[36] - tmp;
acadoWorkspace.ubA[36] = acadoVariables.ubAValues[36] - tmp;
tmp = acadoWorkspace.sbar[116] + acadoVariables.x[116];
acadoWorkspace.lbA[37] = acadoVariables.lbAValues[37] - tmp;
acadoWorkspace.ubA[37] = acadoVariables.ubAValues[37] - tmp;
tmp = acadoWorkspace.sbar[119] + acadoVariables.x[119];
acadoWorkspace.lbA[38] = acadoVariables.lbAValues[38] - tmp;
acadoWorkspace.ubA[38] = acadoVariables.ubAValues[38] - tmp;
tmp = acadoWorkspace.sbar[122] + acadoVariables.x[122];
acadoWorkspace.lbA[39] = acadoVariables.lbAValues[39] - tmp;
acadoWorkspace.ubA[39] = acadoVariables.ubAValues[39] - tmp;
tmp = acadoWorkspace.sbar[125] + acadoVariables.x[125];
acadoWorkspace.lbA[40] = acadoVariables.lbAValues[40] - tmp;
acadoWorkspace.ubA[40] = acadoVariables.ubAValues[40] - tmp;
tmp = acadoWorkspace.sbar[128] + acadoVariables.x[128];
acadoWorkspace.lbA[41] = acadoVariables.lbAValues[41] - tmp;
acadoWorkspace.ubA[41] = acadoVariables.ubAValues[41] - tmp;
tmp = acadoWorkspace.sbar[131] + acadoVariables.x[131];
acadoWorkspace.lbA[42] = acadoVariables.lbAValues[42] - tmp;
acadoWorkspace.ubA[42] = acadoVariables.ubAValues[42] - tmp;
tmp = acadoWorkspace.sbar[134] + acadoVariables.x[134];
acadoWorkspace.lbA[43] = acadoVariables.lbAValues[43] - tmp;
acadoWorkspace.ubA[43] = acadoVariables.ubAValues[43] - tmp;
tmp = acadoWorkspace.sbar[137] + acadoVariables.x[137];
acadoWorkspace.lbA[44] = acadoVariables.lbAValues[44] - tmp;
acadoWorkspace.ubA[44] = acadoVariables.ubAValues[44] - tmp;
tmp = acadoWorkspace.sbar[140] + acadoVariables.x[140];
acadoWorkspace.lbA[45] = acadoVariables.lbAValues[45] - tmp;
acadoWorkspace.ubA[45] = acadoVariables.ubAValues[45] - tmp;
tmp = acadoWorkspace.sbar[143] + acadoVariables.x[143];
acadoWorkspace.lbA[46] = acadoVariables.lbAValues[46] - tmp;
acadoWorkspace.ubA[46] = acadoVariables.ubAValues[46] - tmp;
tmp = acadoWorkspace.sbar[146] + acadoVariables.x[146];
acadoWorkspace.lbA[47] = acadoVariables.lbAValues[47] - tmp;
acadoWorkspace.ubA[47] = acadoVariables.ubAValues[47] - tmp;
tmp = acadoWorkspace.sbar[149] + acadoVariables.x[149];
acadoWorkspace.lbA[48] = acadoVariables.lbAValues[48] - tmp;
acadoWorkspace.ubA[48] = acadoVariables.ubAValues[48] - tmp;
tmp = acadoWorkspace.sbar[152] + acadoVariables.x[152];
acadoWorkspace.lbA[49] = acadoVariables.lbAValues[49] - tmp;
acadoWorkspace.ubA[49] = acadoVariables.ubAValues[49] - tmp;
tmp = acadoWorkspace.sbar[155] + acadoVariables.x[155];
acadoWorkspace.lbA[50] = acadoVariables.lbAValues[50] - tmp;
acadoWorkspace.ubA[50] = acadoVariables.ubAValues[50] - tmp;
tmp = acadoWorkspace.sbar[158] + acadoVariables.x[158];
acadoWorkspace.lbA[51] = acadoVariables.lbAValues[51] - tmp;
acadoWorkspace.ubA[51] = acadoVariables.ubAValues[51] - tmp;
tmp = acadoWorkspace.sbar[161] + acadoVariables.x[161];
acadoWorkspace.lbA[52] = acadoVariables.lbAValues[52] - tmp;
acadoWorkspace.ubA[52] = acadoVariables.ubAValues[52] - tmp;
tmp = acadoWorkspace.sbar[164] + acadoVariables.x[164];
acadoWorkspace.lbA[53] = acadoVariables.lbAValues[53] - tmp;
acadoWorkspace.ubA[53] = acadoVariables.ubAValues[53] - tmp;
tmp = acadoWorkspace.sbar[167] + acadoVariables.x[167];
acadoWorkspace.lbA[54] = acadoVariables.lbAValues[54] - tmp;
acadoWorkspace.ubA[54] = acadoVariables.ubAValues[54] - tmp;
tmp = acadoWorkspace.sbar[170] + acadoVariables.x[170];
acadoWorkspace.lbA[55] = acadoVariables.lbAValues[55] - tmp;
acadoWorkspace.ubA[55] = acadoVariables.ubAValues[55] - tmp;
tmp = acadoWorkspace.sbar[173] + acadoVariables.x[173];
acadoWorkspace.lbA[56] = acadoVariables.lbAValues[56] - tmp;
acadoWorkspace.ubA[56] = acadoVariables.ubAValues[56] - tmp;
tmp = acadoWorkspace.sbar[176] + acadoVariables.x[176];
acadoWorkspace.lbA[57] = acadoVariables.lbAValues[57] - tmp;
acadoWorkspace.ubA[57] = acadoVariables.ubAValues[57] - tmp;
tmp = acadoWorkspace.sbar[179] + acadoVariables.x[179];
acadoWorkspace.lbA[58] = acadoVariables.lbAValues[58] - tmp;
acadoWorkspace.ubA[58] = acadoVariables.ubAValues[58] - tmp;
tmp = acadoWorkspace.sbar[182] + acadoVariables.x[182];
acadoWorkspace.lbA[59] = acadoVariables.lbAValues[59] - tmp;
acadoWorkspace.ubA[59] = acadoVariables.ubAValues[59] - tmp;
tmp = acadoWorkspace.sbar[185] + acadoVariables.x[185];
acadoWorkspace.lbA[60] = acadoVariables.lbAValues[60] - tmp;
acadoWorkspace.ubA[60] = acadoVariables.ubAValues[60] - tmp;
tmp = acadoWorkspace.sbar[188] + acadoVariables.x[188];
acadoWorkspace.lbA[61] = acadoVariables.lbAValues[61] - tmp;
acadoWorkspace.ubA[61] = acadoVariables.ubAValues[61] - tmp;
tmp = acadoWorkspace.sbar[191] + acadoVariables.x[191];
acadoWorkspace.lbA[62] = acadoVariables.lbAValues[62] - tmp;
acadoWorkspace.ubA[62] = acadoVariables.ubAValues[62] - tmp;
tmp = acadoWorkspace.sbar[194] + acadoVariables.x[194];
acadoWorkspace.lbA[63] = acadoVariables.lbAValues[63] - tmp;
acadoWorkspace.ubA[63] = acadoVariables.ubAValues[63] - tmp;
tmp = acadoWorkspace.sbar[197] + acadoVariables.x[197];
acadoWorkspace.lbA[64] = acadoVariables.lbAValues[64] - tmp;
acadoWorkspace.ubA[64] = acadoVariables.ubAValues[64] - tmp;
tmp = acadoWorkspace.sbar[200] + acadoVariables.x[200];
acadoWorkspace.lbA[65] = acadoVariables.lbAValues[65] - tmp;
acadoWorkspace.ubA[65] = acadoVariables.ubAValues[65] - tmp;
tmp = acadoWorkspace.sbar[203] + acadoVariables.x[203];
acadoWorkspace.lbA[66] = acadoVariables.lbAValues[66] - tmp;
acadoWorkspace.ubA[66] = acadoVariables.ubAValues[66] - tmp;
tmp = acadoWorkspace.sbar[206] + acadoVariables.x[206];
acadoWorkspace.lbA[67] = acadoVariables.lbAValues[67] - tmp;
acadoWorkspace.ubA[67] = acadoVariables.ubAValues[67] - tmp;
tmp = acadoWorkspace.sbar[209] + acadoVariables.x[209];
acadoWorkspace.lbA[68] = acadoVariables.lbAValues[68] - tmp;
acadoWorkspace.ubA[68] = acadoVariables.ubAValues[68] - tmp;
tmp = acadoWorkspace.sbar[212] + acadoVariables.x[212];
acadoWorkspace.lbA[69] = acadoVariables.lbAValues[69] - tmp;
acadoWorkspace.ubA[69] = acadoVariables.ubAValues[69] - tmp;
tmp = acadoWorkspace.sbar[215] + acadoVariables.x[215];
acadoWorkspace.lbA[70] = acadoVariables.lbAValues[70] - tmp;
acadoWorkspace.ubA[70] = acadoVariables.ubAValues[70] - tmp;
tmp = acadoWorkspace.sbar[218] + acadoVariables.x[218];
acadoWorkspace.lbA[71] = acadoVariables.lbAValues[71] - tmp;
acadoWorkspace.ubA[71] = acadoVariables.ubAValues[71] - tmp;
tmp = acadoWorkspace.sbar[221] + acadoVariables.x[221];
acadoWorkspace.lbA[72] = acadoVariables.lbAValues[72] - tmp;
acadoWorkspace.ubA[72] = acadoVariables.ubAValues[72] - tmp;
tmp = acadoWorkspace.sbar[224] + acadoVariables.x[224];
acadoWorkspace.lbA[73] = acadoVariables.lbAValues[73] - tmp;
acadoWorkspace.ubA[73] = acadoVariables.ubAValues[73] - tmp;
tmp = acadoWorkspace.sbar[227] + acadoVariables.x[227];
acadoWorkspace.lbA[74] = acadoVariables.lbAValues[74] - tmp;
acadoWorkspace.ubA[74] = acadoVariables.ubAValues[74] - tmp;
tmp = acadoWorkspace.sbar[230] + acadoVariables.x[230];
acadoWorkspace.lbA[75] = acadoVariables.lbAValues[75] - tmp;
acadoWorkspace.ubA[75] = acadoVariables.ubAValues[75] - tmp;
tmp = acadoWorkspace.sbar[233] + acadoVariables.x[233];
acadoWorkspace.lbA[76] = acadoVariables.lbAValues[76] - tmp;
acadoWorkspace.ubA[76] = acadoVariables.ubAValues[76] - tmp;
tmp = acadoWorkspace.sbar[236] + acadoVariables.x[236];
acadoWorkspace.lbA[77] = acadoVariables.lbAValues[77] - tmp;
acadoWorkspace.ubA[77] = acadoVariables.ubAValues[77] - tmp;
tmp = acadoWorkspace.sbar[239] + acadoVariables.x[239];
acadoWorkspace.lbA[78] = acadoVariables.lbAValues[78] - tmp;
acadoWorkspace.ubA[78] = acadoVariables.ubAValues[78] - tmp;
tmp = acadoWorkspace.sbar[242] + acadoVariables.x[242];
acadoWorkspace.lbA[79] = acadoVariables.lbAValues[79] - tmp;
acadoWorkspace.ubA[79] = acadoVariables.ubAValues[79] - tmp;
tmp = acadoWorkspace.sbar[245] + acadoVariables.x[245];
acadoWorkspace.lbA[80] = acadoVariables.lbAValues[80] - tmp;
acadoWorkspace.ubA[80] = acadoVariables.ubAValues[80] - tmp;
tmp = acadoWorkspace.sbar[248] + acadoVariables.x[248];
acadoWorkspace.lbA[81] = acadoVariables.lbAValues[81] - tmp;
acadoWorkspace.ubA[81] = acadoVariables.ubAValues[81] - tmp;
tmp = acadoWorkspace.sbar[251] + acadoVariables.x[251];
acadoWorkspace.lbA[82] = acadoVariables.lbAValues[82] - tmp;
acadoWorkspace.ubA[82] = acadoVariables.ubAValues[82] - tmp;
tmp = acadoWorkspace.sbar[254] + acadoVariables.x[254];
acadoWorkspace.lbA[83] = acadoVariables.lbAValues[83] - tmp;
acadoWorkspace.ubA[83] = acadoVariables.ubAValues[83] - tmp;
tmp = acadoWorkspace.sbar[257] + acadoVariables.x[257];
acadoWorkspace.lbA[84] = acadoVariables.lbAValues[84] - tmp;
acadoWorkspace.ubA[84] = acadoVariables.ubAValues[84] - tmp;
tmp = acadoWorkspace.sbar[260] + acadoVariables.x[260];
acadoWorkspace.lbA[85] = acadoVariables.lbAValues[85] - tmp;
acadoWorkspace.ubA[85] = acadoVariables.ubAValues[85] - tmp;
tmp = acadoWorkspace.sbar[263] + acadoVariables.x[263];
acadoWorkspace.lbA[86] = acadoVariables.lbAValues[86] - tmp;
acadoWorkspace.ubA[86] = acadoVariables.ubAValues[86] - tmp;
tmp = acadoWorkspace.sbar[266] + acadoVariables.x[266];
acadoWorkspace.lbA[87] = acadoVariables.lbAValues[87] - tmp;
acadoWorkspace.ubA[87] = acadoVariables.ubAValues[87] - tmp;
tmp = acadoWorkspace.sbar[269] + acadoVariables.x[269];
acadoWorkspace.lbA[88] = acadoVariables.lbAValues[88] - tmp;
acadoWorkspace.ubA[88] = acadoVariables.ubAValues[88] - tmp;
tmp = acadoWorkspace.sbar[272] + acadoVariables.x[272];
acadoWorkspace.lbA[89] = acadoVariables.lbAValues[89] - tmp;
acadoWorkspace.ubA[89] = acadoVariables.ubAValues[89] - tmp;
tmp = acadoWorkspace.sbar[275] + acadoVariables.x[275];
acadoWorkspace.lbA[90] = acadoVariables.lbAValues[90] - tmp;
acadoWorkspace.ubA[90] = acadoVariables.ubAValues[90] - tmp;
tmp = acadoWorkspace.sbar[278] + acadoVariables.x[278];
acadoWorkspace.lbA[91] = acadoVariables.lbAValues[91] - tmp;
acadoWorkspace.ubA[91] = acadoVariables.ubAValues[91] - tmp;
tmp = acadoWorkspace.sbar[281] + acadoVariables.x[281];
acadoWorkspace.lbA[92] = acadoVariables.lbAValues[92] - tmp;
acadoWorkspace.ubA[92] = acadoVariables.ubAValues[92] - tmp;
tmp = acadoWorkspace.sbar[284] + acadoVariables.x[284];
acadoWorkspace.lbA[93] = acadoVariables.lbAValues[93] - tmp;
acadoWorkspace.ubA[93] = acadoVariables.ubAValues[93] - tmp;
tmp = acadoWorkspace.sbar[287] + acadoVariables.x[287];
acadoWorkspace.lbA[94] = acadoVariables.lbAValues[94] - tmp;
acadoWorkspace.ubA[94] = acadoVariables.ubAValues[94] - tmp;
tmp = acadoWorkspace.sbar[290] + acadoVariables.x[290];
acadoWorkspace.lbA[95] = acadoVariables.lbAValues[95] - tmp;
acadoWorkspace.ubA[95] = acadoVariables.ubAValues[95] - tmp;
tmp = acadoWorkspace.sbar[293] + acadoVariables.x[293];
acadoWorkspace.lbA[96] = acadoVariables.lbAValues[96] - tmp;
acadoWorkspace.ubA[96] = acadoVariables.ubAValues[96] - tmp;
tmp = acadoWorkspace.sbar[296] + acadoVariables.x[296];
acadoWorkspace.lbA[97] = acadoVariables.lbAValues[97] - tmp;
acadoWorkspace.ubA[97] = acadoVariables.ubAValues[97] - tmp;
tmp = acadoWorkspace.sbar[299] + acadoVariables.x[299];
acadoWorkspace.lbA[98] = acadoVariables.lbAValues[98] - tmp;
acadoWorkspace.ubA[98] = acadoVariables.ubAValues[98] - tmp;
tmp = acadoWorkspace.sbar[302] + acadoVariables.x[302];
acadoWorkspace.lbA[99] = acadoVariables.lbAValues[99] - tmp;
acadoWorkspace.ubA[99] = acadoVariables.ubAValues[99] - tmp;

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoVariables.u[80] += acadoWorkspace.x[80];
acadoVariables.u[81] += acadoWorkspace.x[81];
acadoVariables.u[82] += acadoWorkspace.x[82];
acadoVariables.u[83] += acadoWorkspace.x[83];
acadoVariables.u[84] += acadoWorkspace.x[84];
acadoVariables.u[85] += acadoWorkspace.x[85];
acadoVariables.u[86] += acadoWorkspace.x[86];
acadoVariables.u[87] += acadoWorkspace.x[87];
acadoVariables.u[88] += acadoWorkspace.x[88];
acadoVariables.u[89] += acadoWorkspace.x[89];
acadoVariables.u[90] += acadoWorkspace.x[90];
acadoVariables.u[91] += acadoWorkspace.x[91];
acadoVariables.u[92] += acadoWorkspace.x[92];
acadoVariables.u[93] += acadoWorkspace.x[93];
acadoVariables.u[94] += acadoWorkspace.x[94];
acadoVariables.u[95] += acadoWorkspace.x[95];
acadoVariables.u[96] += acadoWorkspace.x[96];
acadoVariables.u[97] += acadoWorkspace.x[97];
acadoVariables.u[98] += acadoWorkspace.x[98];
acadoVariables.u[99] += acadoWorkspace.x[99];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
for (lRun1 = 0; lRun1 < 300; ++lRun1)
acadoWorkspace.sbar[lRun1 + 3] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 3 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.evGu[ 3 ]), &(acadoWorkspace.x[ 1 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 9 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.evGu[ 9 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.evGu[ 15 ]), &(acadoWorkspace.x[ 5 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.evGu[ 21 ]), &(acadoWorkspace.x[ 7 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.evGu[ 27 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 33 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.evGu[ 33 ]), &(acadoWorkspace.x[ 11 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.evGu[ 39 ]), &(acadoWorkspace.x[ 13 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.evGu[ 42 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.evGu[ 45 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 51 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.evGu[ 51 ]), &(acadoWorkspace.x[ 17 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 57 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.evGu[ 57 ]), &(acadoWorkspace.x[ 19 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 189 ]), &(acadoWorkspace.evGu[ 63 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 198 ]), &(acadoWorkspace.evGu[ 66 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 69 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 207 ]), &(acadoWorkspace.evGu[ 69 ]), &(acadoWorkspace.x[ 23 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 75 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.evGu[ 75 ]), &(acadoWorkspace.x[ 25 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 234 ]), &(acadoWorkspace.evGu[ 78 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.evGu[ 81 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 87 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 261 ]), &(acadoWorkspace.evGu[ 87 ]), &(acadoWorkspace.x[ 29 ]), &(acadoWorkspace.sbar[ 87 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 270 ]), &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 93 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 279 ]), &(acadoWorkspace.evGu[ 93 ]), &(acadoWorkspace.x[ 31 ]), &(acadoWorkspace.sbar[ 93 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 297 ]), &(acadoWorkspace.evGu[ 99 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 306 ]), &(acadoWorkspace.evGu[ 102 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 315 ]), &(acadoWorkspace.evGu[ 105 ]), &(acadoWorkspace.x[ 35 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 111 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 333 ]), &(acadoWorkspace.evGu[ 111 ]), &(acadoWorkspace.x[ 37 ]), &(acadoWorkspace.sbar[ 111 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 342 ]), &(acadoWorkspace.evGu[ 114 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 351 ]), &(acadoWorkspace.evGu[ 117 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 123 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 369 ]), &(acadoWorkspace.evGu[ 123 ]), &(acadoWorkspace.x[ 41 ]), &(acadoWorkspace.sbar[ 123 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 378 ]), &(acadoWorkspace.evGu[ 126 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 129 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 387 ]), &(acadoWorkspace.evGu[ 129 ]), &(acadoWorkspace.x[ 43 ]), &(acadoWorkspace.sbar[ 129 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.evGu[ 132 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.evGu[ 135 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 138 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 414 ]), &(acadoWorkspace.evGu[ 138 ]), &(acadoWorkspace.x[ 46 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.sbar[ 141 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 423 ]), &(acadoWorkspace.evGu[ 141 ]), &(acadoWorkspace.x[ 47 ]), &(acadoWorkspace.sbar[ 141 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 147 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.evGu[ 147 ]), &(acadoWorkspace.x[ 49 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.sbar[ 150 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 450 ]), &(acadoWorkspace.evGu[ 150 ]), &(acadoWorkspace.x[ 50 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 459 ]), &(acadoWorkspace.evGu[ 153 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 159 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 477 ]), &(acadoWorkspace.evGu[ 159 ]), &(acadoWorkspace.x[ 53 ]), &(acadoWorkspace.sbar[ 159 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.evGu[ 162 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 165 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 495 ]), &(acadoWorkspace.evGu[ 165 ]), &(acadoWorkspace.x[ 55 ]), &(acadoWorkspace.sbar[ 165 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.evGu[ 168 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 513 ]), &(acadoWorkspace.evGu[ 171 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 174 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 522 ]), &(acadoWorkspace.evGu[ 174 ]), &(acadoWorkspace.x[ 58 ]), &(acadoWorkspace.sbar[ 174 ]), &(acadoWorkspace.sbar[ 177 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 531 ]), &(acadoWorkspace.evGu[ 177 ]), &(acadoWorkspace.x[ 59 ]), &(acadoWorkspace.sbar[ 177 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.evGu[ 180 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 183 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 549 ]), &(acadoWorkspace.evGu[ 183 ]), &(acadoWorkspace.x[ 61 ]), &(acadoWorkspace.sbar[ 183 ]), &(acadoWorkspace.sbar[ 186 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 558 ]), &(acadoWorkspace.evGu[ 186 ]), &(acadoWorkspace.x[ 62 ]), &(acadoWorkspace.sbar[ 186 ]), &(acadoWorkspace.sbar[ 189 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.evGu[ 189 ]), &(acadoWorkspace.x[ 63 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.sbar[ 192 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.sbar[ 195 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 585 ]), &(acadoWorkspace.evGu[ 195 ]), &(acadoWorkspace.x[ 65 ]), &(acadoWorkspace.sbar[ 195 ]), &(acadoWorkspace.sbar[ 198 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 594 ]), &(acadoWorkspace.evGu[ 198 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.sbar[ 198 ]), &(acadoWorkspace.sbar[ 201 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 603 ]), &(acadoWorkspace.evGu[ 201 ]), &(acadoWorkspace.x[ 67 ]), &(acadoWorkspace.sbar[ 201 ]), &(acadoWorkspace.sbar[ 204 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.evGu[ 204 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.sbar[ 204 ]), &(acadoWorkspace.sbar[ 207 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 621 ]), &(acadoWorkspace.evGu[ 207 ]), &(acadoWorkspace.x[ 69 ]), &(acadoWorkspace.sbar[ 207 ]), &(acadoWorkspace.sbar[ 210 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 630 ]), &(acadoWorkspace.evGu[ 210 ]), &(acadoWorkspace.x[ 70 ]), &(acadoWorkspace.sbar[ 210 ]), &(acadoWorkspace.sbar[ 213 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 639 ]), &(acadoWorkspace.evGu[ 213 ]), &(acadoWorkspace.x[ 71 ]), &(acadoWorkspace.sbar[ 213 ]), &(acadoWorkspace.sbar[ 216 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.sbar[ 219 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 657 ]), &(acadoWorkspace.evGu[ 219 ]), &(acadoWorkspace.x[ 73 ]), &(acadoWorkspace.sbar[ 219 ]), &(acadoWorkspace.sbar[ 222 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 666 ]), &(acadoWorkspace.evGu[ 222 ]), &(acadoWorkspace.x[ 74 ]), &(acadoWorkspace.sbar[ 222 ]), &(acadoWorkspace.sbar[ 225 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 675 ]), &(acadoWorkspace.evGu[ 225 ]), &(acadoWorkspace.x[ 75 ]), &(acadoWorkspace.sbar[ 225 ]), &(acadoWorkspace.sbar[ 228 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.evGu[ 228 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.sbar[ 228 ]), &(acadoWorkspace.sbar[ 231 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 693 ]), &(acadoWorkspace.evGu[ 231 ]), &(acadoWorkspace.x[ 77 ]), &(acadoWorkspace.sbar[ 231 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 702 ]), &(acadoWorkspace.evGu[ 234 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 237 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 711 ]), &(acadoWorkspace.evGu[ 237 ]), &(acadoWorkspace.x[ 79 ]), &(acadoWorkspace.sbar[ 237 ]), &(acadoWorkspace.sbar[ 240 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.x[ 80 ]), &(acadoWorkspace.sbar[ 240 ]), &(acadoWorkspace.sbar[ 243 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.evGu[ 243 ]), &(acadoWorkspace.x[ 81 ]), &(acadoWorkspace.sbar[ 243 ]), &(acadoWorkspace.sbar[ 246 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 738 ]), &(acadoWorkspace.evGu[ 246 ]), &(acadoWorkspace.x[ 82 ]), &(acadoWorkspace.sbar[ 246 ]), &(acadoWorkspace.sbar[ 249 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 747 ]), &(acadoWorkspace.evGu[ 249 ]), &(acadoWorkspace.x[ 83 ]), &(acadoWorkspace.sbar[ 249 ]), &(acadoWorkspace.sbar[ 252 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 756 ]), &(acadoWorkspace.evGu[ 252 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.sbar[ 255 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 765 ]), &(acadoWorkspace.evGu[ 255 ]), &(acadoWorkspace.x[ 85 ]), &(acadoWorkspace.sbar[ 255 ]), &(acadoWorkspace.sbar[ 258 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 774 ]), &(acadoWorkspace.evGu[ 258 ]), &(acadoWorkspace.x[ 86 ]), &(acadoWorkspace.sbar[ 258 ]), &(acadoWorkspace.sbar[ 261 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 783 ]), &(acadoWorkspace.evGu[ 261 ]), &(acadoWorkspace.x[ 87 ]), &(acadoWorkspace.sbar[ 261 ]), &(acadoWorkspace.sbar[ 264 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 792 ]), &(acadoWorkspace.evGu[ 264 ]), &(acadoWorkspace.x[ 88 ]), &(acadoWorkspace.sbar[ 264 ]), &(acadoWorkspace.sbar[ 267 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 801 ]), &(acadoWorkspace.evGu[ 267 ]), &(acadoWorkspace.x[ 89 ]), &(acadoWorkspace.sbar[ 267 ]), &(acadoWorkspace.sbar[ 270 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.evGu[ 270 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.sbar[ 270 ]), &(acadoWorkspace.sbar[ 273 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 819 ]), &(acadoWorkspace.evGu[ 273 ]), &(acadoWorkspace.x[ 91 ]), &(acadoWorkspace.sbar[ 273 ]), &(acadoWorkspace.sbar[ 276 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 828 ]), &(acadoWorkspace.evGu[ 276 ]), &(acadoWorkspace.x[ 92 ]), &(acadoWorkspace.sbar[ 276 ]), &(acadoWorkspace.sbar[ 279 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 837 ]), &(acadoWorkspace.evGu[ 279 ]), &(acadoWorkspace.x[ 93 ]), &(acadoWorkspace.sbar[ 279 ]), &(acadoWorkspace.sbar[ 282 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 846 ]), &(acadoWorkspace.evGu[ 282 ]), &(acadoWorkspace.x[ 94 ]), &(acadoWorkspace.sbar[ 282 ]), &(acadoWorkspace.sbar[ 285 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 855 ]), &(acadoWorkspace.evGu[ 285 ]), &(acadoWorkspace.x[ 95 ]), &(acadoWorkspace.sbar[ 285 ]), &(acadoWorkspace.sbar[ 288 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.sbar[ 288 ]), &(acadoWorkspace.sbar[ 291 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 873 ]), &(acadoWorkspace.evGu[ 291 ]), &(acadoWorkspace.x[ 97 ]), &(acadoWorkspace.sbar[ 291 ]), &(acadoWorkspace.sbar[ 294 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 882 ]), &(acadoWorkspace.evGu[ 294 ]), &(acadoWorkspace.x[ 98 ]), &(acadoWorkspace.sbar[ 294 ]), &(acadoWorkspace.sbar[ 297 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.evGu[ 297 ]), &(acadoWorkspace.x[ 99 ]), &(acadoWorkspace.sbar[ 297 ]), &(acadoWorkspace.sbar[ 300 ]) );
for (lRun1 = 0; lRun1 < 303; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -5.0000000000000000e-01;
acadoVariables.lbValues[1] = -5.0000000000000000e-01;
acadoVariables.lbValues[2] = -5.0000000000000000e-01;
acadoVariables.lbValues[3] = -5.0000000000000000e-01;
acadoVariables.lbValues[4] = -5.0000000000000000e-01;
acadoVariables.lbValues[5] = -5.0000000000000000e-01;
acadoVariables.lbValues[6] = -5.0000000000000000e-01;
acadoVariables.lbValues[7] = -5.0000000000000000e-01;
acadoVariables.lbValues[8] = -5.0000000000000000e-01;
acadoVariables.lbValues[9] = -5.0000000000000000e-01;
acadoVariables.lbValues[10] = -5.0000000000000000e-01;
acadoVariables.lbValues[11] = -5.0000000000000000e-01;
acadoVariables.lbValues[12] = -5.0000000000000000e-01;
acadoVariables.lbValues[13] = -5.0000000000000000e-01;
acadoVariables.lbValues[14] = -5.0000000000000000e-01;
acadoVariables.lbValues[15] = -5.0000000000000000e-01;
acadoVariables.lbValues[16] = -5.0000000000000000e-01;
acadoVariables.lbValues[17] = -5.0000000000000000e-01;
acadoVariables.lbValues[18] = -5.0000000000000000e-01;
acadoVariables.lbValues[19] = -5.0000000000000000e-01;
acadoVariables.lbValues[20] = -5.0000000000000000e-01;
acadoVariables.lbValues[21] = -5.0000000000000000e-01;
acadoVariables.lbValues[22] = -5.0000000000000000e-01;
acadoVariables.lbValues[23] = -5.0000000000000000e-01;
acadoVariables.lbValues[24] = -5.0000000000000000e-01;
acadoVariables.lbValues[25] = -5.0000000000000000e-01;
acadoVariables.lbValues[26] = -5.0000000000000000e-01;
acadoVariables.lbValues[27] = -5.0000000000000000e-01;
acadoVariables.lbValues[28] = -5.0000000000000000e-01;
acadoVariables.lbValues[29] = -5.0000000000000000e-01;
acadoVariables.lbValues[30] = -5.0000000000000000e-01;
acadoVariables.lbValues[31] = -5.0000000000000000e-01;
acadoVariables.lbValues[32] = -5.0000000000000000e-01;
acadoVariables.lbValues[33] = -5.0000000000000000e-01;
acadoVariables.lbValues[34] = -5.0000000000000000e-01;
acadoVariables.lbValues[35] = -5.0000000000000000e-01;
acadoVariables.lbValues[36] = -5.0000000000000000e-01;
acadoVariables.lbValues[37] = -5.0000000000000000e-01;
acadoVariables.lbValues[38] = -5.0000000000000000e-01;
acadoVariables.lbValues[39] = -5.0000000000000000e-01;
acadoVariables.lbValues[40] = -5.0000000000000000e-01;
acadoVariables.lbValues[41] = -5.0000000000000000e-01;
acadoVariables.lbValues[42] = -5.0000000000000000e-01;
acadoVariables.lbValues[43] = -5.0000000000000000e-01;
acadoVariables.lbValues[44] = -5.0000000000000000e-01;
acadoVariables.lbValues[45] = -5.0000000000000000e-01;
acadoVariables.lbValues[46] = -5.0000000000000000e-01;
acadoVariables.lbValues[47] = -5.0000000000000000e-01;
acadoVariables.lbValues[48] = -5.0000000000000000e-01;
acadoVariables.lbValues[49] = -5.0000000000000000e-01;
acadoVariables.lbValues[50] = -5.0000000000000000e-01;
acadoVariables.lbValues[51] = -5.0000000000000000e-01;
acadoVariables.lbValues[52] = -5.0000000000000000e-01;
acadoVariables.lbValues[53] = -5.0000000000000000e-01;
acadoVariables.lbValues[54] = -5.0000000000000000e-01;
acadoVariables.lbValues[55] = -5.0000000000000000e-01;
acadoVariables.lbValues[56] = -5.0000000000000000e-01;
acadoVariables.lbValues[57] = -5.0000000000000000e-01;
acadoVariables.lbValues[58] = -5.0000000000000000e-01;
acadoVariables.lbValues[59] = -5.0000000000000000e-01;
acadoVariables.lbValues[60] = -5.0000000000000000e-01;
acadoVariables.lbValues[61] = -5.0000000000000000e-01;
acadoVariables.lbValues[62] = -5.0000000000000000e-01;
acadoVariables.lbValues[63] = -5.0000000000000000e-01;
acadoVariables.lbValues[64] = -5.0000000000000000e-01;
acadoVariables.lbValues[65] = -5.0000000000000000e-01;
acadoVariables.lbValues[66] = -5.0000000000000000e-01;
acadoVariables.lbValues[67] = -5.0000000000000000e-01;
acadoVariables.lbValues[68] = -5.0000000000000000e-01;
acadoVariables.lbValues[69] = -5.0000000000000000e-01;
acadoVariables.lbValues[70] = -5.0000000000000000e-01;
acadoVariables.lbValues[71] = -5.0000000000000000e-01;
acadoVariables.lbValues[72] = -5.0000000000000000e-01;
acadoVariables.lbValues[73] = -5.0000000000000000e-01;
acadoVariables.lbValues[74] = -5.0000000000000000e-01;
acadoVariables.lbValues[75] = -5.0000000000000000e-01;
acadoVariables.lbValues[76] = -5.0000000000000000e-01;
acadoVariables.lbValues[77] = -5.0000000000000000e-01;
acadoVariables.lbValues[78] = -5.0000000000000000e-01;
acadoVariables.lbValues[79] = -5.0000000000000000e-01;
acadoVariables.lbValues[80] = -5.0000000000000000e-01;
acadoVariables.lbValues[81] = -5.0000000000000000e-01;
acadoVariables.lbValues[82] = -5.0000000000000000e-01;
acadoVariables.lbValues[83] = -5.0000000000000000e-01;
acadoVariables.lbValues[84] = -5.0000000000000000e-01;
acadoVariables.lbValues[85] = -5.0000000000000000e-01;
acadoVariables.lbValues[86] = -5.0000000000000000e-01;
acadoVariables.lbValues[87] = -5.0000000000000000e-01;
acadoVariables.lbValues[88] = -5.0000000000000000e-01;
acadoVariables.lbValues[89] = -5.0000000000000000e-01;
acadoVariables.lbValues[90] = -5.0000000000000000e-01;
acadoVariables.lbValues[91] = -5.0000000000000000e-01;
acadoVariables.lbValues[92] = -5.0000000000000000e-01;
acadoVariables.lbValues[93] = -5.0000000000000000e-01;
acadoVariables.lbValues[94] = -5.0000000000000000e-01;
acadoVariables.lbValues[95] = -5.0000000000000000e-01;
acadoVariables.lbValues[96] = -5.0000000000000000e-01;
acadoVariables.lbValues[97] = -5.0000000000000000e-01;
acadoVariables.lbValues[98] = -5.0000000000000000e-01;
acadoVariables.lbValues[99] = -5.0000000000000000e-01;
acadoVariables.ubValues[0] = 5.0000000000000000e-01;
acadoVariables.ubValues[1] = 5.0000000000000000e-01;
acadoVariables.ubValues[2] = 5.0000000000000000e-01;
acadoVariables.ubValues[3] = 5.0000000000000000e-01;
acadoVariables.ubValues[4] = 5.0000000000000000e-01;
acadoVariables.ubValues[5] = 5.0000000000000000e-01;
acadoVariables.ubValues[6] = 5.0000000000000000e-01;
acadoVariables.ubValues[7] = 5.0000000000000000e-01;
acadoVariables.ubValues[8] = 5.0000000000000000e-01;
acadoVariables.ubValues[9] = 5.0000000000000000e-01;
acadoVariables.ubValues[10] = 5.0000000000000000e-01;
acadoVariables.ubValues[11] = 5.0000000000000000e-01;
acadoVariables.ubValues[12] = 5.0000000000000000e-01;
acadoVariables.ubValues[13] = 5.0000000000000000e-01;
acadoVariables.ubValues[14] = 5.0000000000000000e-01;
acadoVariables.ubValues[15] = 5.0000000000000000e-01;
acadoVariables.ubValues[16] = 5.0000000000000000e-01;
acadoVariables.ubValues[17] = 5.0000000000000000e-01;
acadoVariables.ubValues[18] = 5.0000000000000000e-01;
acadoVariables.ubValues[19] = 5.0000000000000000e-01;
acadoVariables.ubValues[20] = 5.0000000000000000e-01;
acadoVariables.ubValues[21] = 5.0000000000000000e-01;
acadoVariables.ubValues[22] = 5.0000000000000000e-01;
acadoVariables.ubValues[23] = 5.0000000000000000e-01;
acadoVariables.ubValues[24] = 5.0000000000000000e-01;
acadoVariables.ubValues[25] = 5.0000000000000000e-01;
acadoVariables.ubValues[26] = 5.0000000000000000e-01;
acadoVariables.ubValues[27] = 5.0000000000000000e-01;
acadoVariables.ubValues[28] = 5.0000000000000000e-01;
acadoVariables.ubValues[29] = 5.0000000000000000e-01;
acadoVariables.ubValues[30] = 5.0000000000000000e-01;
acadoVariables.ubValues[31] = 5.0000000000000000e-01;
acadoVariables.ubValues[32] = 5.0000000000000000e-01;
acadoVariables.ubValues[33] = 5.0000000000000000e-01;
acadoVariables.ubValues[34] = 5.0000000000000000e-01;
acadoVariables.ubValues[35] = 5.0000000000000000e-01;
acadoVariables.ubValues[36] = 5.0000000000000000e-01;
acadoVariables.ubValues[37] = 5.0000000000000000e-01;
acadoVariables.ubValues[38] = 5.0000000000000000e-01;
acadoVariables.ubValues[39] = 5.0000000000000000e-01;
acadoVariables.ubValues[40] = 5.0000000000000000e-01;
acadoVariables.ubValues[41] = 5.0000000000000000e-01;
acadoVariables.ubValues[42] = 5.0000000000000000e-01;
acadoVariables.ubValues[43] = 5.0000000000000000e-01;
acadoVariables.ubValues[44] = 5.0000000000000000e-01;
acadoVariables.ubValues[45] = 5.0000000000000000e-01;
acadoVariables.ubValues[46] = 5.0000000000000000e-01;
acadoVariables.ubValues[47] = 5.0000000000000000e-01;
acadoVariables.ubValues[48] = 5.0000000000000000e-01;
acadoVariables.ubValues[49] = 5.0000000000000000e-01;
acadoVariables.ubValues[50] = 5.0000000000000000e-01;
acadoVariables.ubValues[51] = 5.0000000000000000e-01;
acadoVariables.ubValues[52] = 5.0000000000000000e-01;
acadoVariables.ubValues[53] = 5.0000000000000000e-01;
acadoVariables.ubValues[54] = 5.0000000000000000e-01;
acadoVariables.ubValues[55] = 5.0000000000000000e-01;
acadoVariables.ubValues[56] = 5.0000000000000000e-01;
acadoVariables.ubValues[57] = 5.0000000000000000e-01;
acadoVariables.ubValues[58] = 5.0000000000000000e-01;
acadoVariables.ubValues[59] = 5.0000000000000000e-01;
acadoVariables.ubValues[60] = 5.0000000000000000e-01;
acadoVariables.ubValues[61] = 5.0000000000000000e-01;
acadoVariables.ubValues[62] = 5.0000000000000000e-01;
acadoVariables.ubValues[63] = 5.0000000000000000e-01;
acadoVariables.ubValues[64] = 5.0000000000000000e-01;
acadoVariables.ubValues[65] = 5.0000000000000000e-01;
acadoVariables.ubValues[66] = 5.0000000000000000e-01;
acadoVariables.ubValues[67] = 5.0000000000000000e-01;
acadoVariables.ubValues[68] = 5.0000000000000000e-01;
acadoVariables.ubValues[69] = 5.0000000000000000e-01;
acadoVariables.ubValues[70] = 5.0000000000000000e-01;
acadoVariables.ubValues[71] = 5.0000000000000000e-01;
acadoVariables.ubValues[72] = 5.0000000000000000e-01;
acadoVariables.ubValues[73] = 5.0000000000000000e-01;
acadoVariables.ubValues[74] = 5.0000000000000000e-01;
acadoVariables.ubValues[75] = 5.0000000000000000e-01;
acadoVariables.ubValues[76] = 5.0000000000000000e-01;
acadoVariables.ubValues[77] = 5.0000000000000000e-01;
acadoVariables.ubValues[78] = 5.0000000000000000e-01;
acadoVariables.ubValues[79] = 5.0000000000000000e-01;
acadoVariables.ubValues[80] = 5.0000000000000000e-01;
acadoVariables.ubValues[81] = 5.0000000000000000e-01;
acadoVariables.ubValues[82] = 5.0000000000000000e-01;
acadoVariables.ubValues[83] = 5.0000000000000000e-01;
acadoVariables.ubValues[84] = 5.0000000000000000e-01;
acadoVariables.ubValues[85] = 5.0000000000000000e-01;
acadoVariables.ubValues[86] = 5.0000000000000000e-01;
acadoVariables.ubValues[87] = 5.0000000000000000e-01;
acadoVariables.ubValues[88] = 5.0000000000000000e-01;
acadoVariables.ubValues[89] = 5.0000000000000000e-01;
acadoVariables.ubValues[90] = 5.0000000000000000e-01;
acadoVariables.ubValues[91] = 5.0000000000000000e-01;
acadoVariables.ubValues[92] = 5.0000000000000000e-01;
acadoVariables.ubValues[93] = 5.0000000000000000e-01;
acadoVariables.ubValues[94] = 5.0000000000000000e-01;
acadoVariables.ubValues[95] = 5.0000000000000000e-01;
acadoVariables.ubValues[96] = 5.0000000000000000e-01;
acadoVariables.ubValues[97] = 5.0000000000000000e-01;
acadoVariables.ubValues[98] = 5.0000000000000000e-01;
acadoVariables.ubValues[99] = 5.0000000000000000e-01;
acadoVariables.lbAValues[0] = -3.0000000000000000e+01;
acadoVariables.lbAValues[1] = -3.0000000000000000e+01;
acadoVariables.lbAValues[2] = -3.0000000000000000e+01;
acadoVariables.lbAValues[3] = -3.0000000000000000e+01;
acadoVariables.lbAValues[4] = -3.0000000000000000e+01;
acadoVariables.lbAValues[5] = -3.0000000000000000e+01;
acadoVariables.lbAValues[6] = -3.0000000000000000e+01;
acadoVariables.lbAValues[7] = -3.0000000000000000e+01;
acadoVariables.lbAValues[8] = -3.0000000000000000e+01;
acadoVariables.lbAValues[9] = -3.0000000000000000e+01;
acadoVariables.lbAValues[10] = -3.0000000000000000e+01;
acadoVariables.lbAValues[11] = -3.0000000000000000e+01;
acadoVariables.lbAValues[12] = -3.0000000000000000e+01;
acadoVariables.lbAValues[13] = -3.0000000000000000e+01;
acadoVariables.lbAValues[14] = -3.0000000000000000e+01;
acadoVariables.lbAValues[15] = -3.0000000000000000e+01;
acadoVariables.lbAValues[16] = -3.0000000000000000e+01;
acadoVariables.lbAValues[17] = -3.0000000000000000e+01;
acadoVariables.lbAValues[18] = -3.0000000000000000e+01;
acadoVariables.lbAValues[19] = -3.0000000000000000e+01;
acadoVariables.lbAValues[20] = -3.0000000000000000e+01;
acadoVariables.lbAValues[21] = -3.0000000000000000e+01;
acadoVariables.lbAValues[22] = -3.0000000000000000e+01;
acadoVariables.lbAValues[23] = -3.0000000000000000e+01;
acadoVariables.lbAValues[24] = -3.0000000000000000e+01;
acadoVariables.lbAValues[25] = -3.0000000000000000e+01;
acadoVariables.lbAValues[26] = -3.0000000000000000e+01;
acadoVariables.lbAValues[27] = -3.0000000000000000e+01;
acadoVariables.lbAValues[28] = -3.0000000000000000e+01;
acadoVariables.lbAValues[29] = -3.0000000000000000e+01;
acadoVariables.lbAValues[30] = -3.0000000000000000e+01;
acadoVariables.lbAValues[31] = -3.0000000000000000e+01;
acadoVariables.lbAValues[32] = -3.0000000000000000e+01;
acadoVariables.lbAValues[33] = -3.0000000000000000e+01;
acadoVariables.lbAValues[34] = -3.0000000000000000e+01;
acadoVariables.lbAValues[35] = -3.0000000000000000e+01;
acadoVariables.lbAValues[36] = -3.0000000000000000e+01;
acadoVariables.lbAValues[37] = -3.0000000000000000e+01;
acadoVariables.lbAValues[38] = -3.0000000000000000e+01;
acadoVariables.lbAValues[39] = -3.0000000000000000e+01;
acadoVariables.lbAValues[40] = -3.0000000000000000e+01;
acadoVariables.lbAValues[41] = -3.0000000000000000e+01;
acadoVariables.lbAValues[42] = -3.0000000000000000e+01;
acadoVariables.lbAValues[43] = -3.0000000000000000e+01;
acadoVariables.lbAValues[44] = -3.0000000000000000e+01;
acadoVariables.lbAValues[45] = -3.0000000000000000e+01;
acadoVariables.lbAValues[46] = -3.0000000000000000e+01;
acadoVariables.lbAValues[47] = -3.0000000000000000e+01;
acadoVariables.lbAValues[48] = -3.0000000000000000e+01;
acadoVariables.lbAValues[49] = -3.0000000000000000e+01;
acadoVariables.lbAValues[50] = -3.0000000000000000e+01;
acadoVariables.lbAValues[51] = -3.0000000000000000e+01;
acadoVariables.lbAValues[52] = -3.0000000000000000e+01;
acadoVariables.lbAValues[53] = -3.0000000000000000e+01;
acadoVariables.lbAValues[54] = -3.0000000000000000e+01;
acadoVariables.lbAValues[55] = -3.0000000000000000e+01;
acadoVariables.lbAValues[56] = -3.0000000000000000e+01;
acadoVariables.lbAValues[57] = -3.0000000000000000e+01;
acadoVariables.lbAValues[58] = -3.0000000000000000e+01;
acadoVariables.lbAValues[59] = -3.0000000000000000e+01;
acadoVariables.lbAValues[60] = -3.0000000000000000e+01;
acadoVariables.lbAValues[61] = -3.0000000000000000e+01;
acadoVariables.lbAValues[62] = -3.0000000000000000e+01;
acadoVariables.lbAValues[63] = -3.0000000000000000e+01;
acadoVariables.lbAValues[64] = -3.0000000000000000e+01;
acadoVariables.lbAValues[65] = -3.0000000000000000e+01;
acadoVariables.lbAValues[66] = -3.0000000000000000e+01;
acadoVariables.lbAValues[67] = -3.0000000000000000e+01;
acadoVariables.lbAValues[68] = -3.0000000000000000e+01;
acadoVariables.lbAValues[69] = -3.0000000000000000e+01;
acadoVariables.lbAValues[70] = -3.0000000000000000e+01;
acadoVariables.lbAValues[71] = -3.0000000000000000e+01;
acadoVariables.lbAValues[72] = -3.0000000000000000e+01;
acadoVariables.lbAValues[73] = -3.0000000000000000e+01;
acadoVariables.lbAValues[74] = -3.0000000000000000e+01;
acadoVariables.lbAValues[75] = -3.0000000000000000e+01;
acadoVariables.lbAValues[76] = -3.0000000000000000e+01;
acadoVariables.lbAValues[77] = -3.0000000000000000e+01;
acadoVariables.lbAValues[78] = -3.0000000000000000e+01;
acadoVariables.lbAValues[79] = -3.0000000000000000e+01;
acadoVariables.lbAValues[80] = -3.0000000000000000e+01;
acadoVariables.lbAValues[81] = -3.0000000000000000e+01;
acadoVariables.lbAValues[82] = -3.0000000000000000e+01;
acadoVariables.lbAValues[83] = -3.0000000000000000e+01;
acadoVariables.lbAValues[84] = -3.0000000000000000e+01;
acadoVariables.lbAValues[85] = -3.0000000000000000e+01;
acadoVariables.lbAValues[86] = -3.0000000000000000e+01;
acadoVariables.lbAValues[87] = -3.0000000000000000e+01;
acadoVariables.lbAValues[88] = -3.0000000000000000e+01;
acadoVariables.lbAValues[89] = -3.0000000000000000e+01;
acadoVariables.lbAValues[90] = -3.0000000000000000e+01;
acadoVariables.lbAValues[91] = -3.0000000000000000e+01;
acadoVariables.lbAValues[92] = -3.0000000000000000e+01;
acadoVariables.lbAValues[93] = -3.0000000000000000e+01;
acadoVariables.lbAValues[94] = -3.0000000000000000e+01;
acadoVariables.lbAValues[95] = -3.0000000000000000e+01;
acadoVariables.lbAValues[96] = -3.0000000000000000e+01;
acadoVariables.lbAValues[97] = -3.0000000000000000e+01;
acadoVariables.lbAValues[98] = -3.0000000000000000e+01;
acadoVariables.lbAValues[99] = -3.0000000000000000e+01;
acadoVariables.ubAValues[0] = 3.0000000000000000e+01;
acadoVariables.ubAValues[1] = 3.0000000000000000e+01;
acadoVariables.ubAValues[2] = 3.0000000000000000e+01;
acadoVariables.ubAValues[3] = 3.0000000000000000e+01;
acadoVariables.ubAValues[4] = 3.0000000000000000e+01;
acadoVariables.ubAValues[5] = 3.0000000000000000e+01;
acadoVariables.ubAValues[6] = 3.0000000000000000e+01;
acadoVariables.ubAValues[7] = 3.0000000000000000e+01;
acadoVariables.ubAValues[8] = 3.0000000000000000e+01;
acadoVariables.ubAValues[9] = 3.0000000000000000e+01;
acadoVariables.ubAValues[10] = 3.0000000000000000e+01;
acadoVariables.ubAValues[11] = 3.0000000000000000e+01;
acadoVariables.ubAValues[12] = 3.0000000000000000e+01;
acadoVariables.ubAValues[13] = 3.0000000000000000e+01;
acadoVariables.ubAValues[14] = 3.0000000000000000e+01;
acadoVariables.ubAValues[15] = 3.0000000000000000e+01;
acadoVariables.ubAValues[16] = 3.0000000000000000e+01;
acadoVariables.ubAValues[17] = 3.0000000000000000e+01;
acadoVariables.ubAValues[18] = 3.0000000000000000e+01;
acadoVariables.ubAValues[19] = 3.0000000000000000e+01;
acadoVariables.ubAValues[20] = 3.0000000000000000e+01;
acadoVariables.ubAValues[21] = 3.0000000000000000e+01;
acadoVariables.ubAValues[22] = 3.0000000000000000e+01;
acadoVariables.ubAValues[23] = 3.0000000000000000e+01;
acadoVariables.ubAValues[24] = 3.0000000000000000e+01;
acadoVariables.ubAValues[25] = 3.0000000000000000e+01;
acadoVariables.ubAValues[26] = 3.0000000000000000e+01;
acadoVariables.ubAValues[27] = 3.0000000000000000e+01;
acadoVariables.ubAValues[28] = 3.0000000000000000e+01;
acadoVariables.ubAValues[29] = 3.0000000000000000e+01;
acadoVariables.ubAValues[30] = 3.0000000000000000e+01;
acadoVariables.ubAValues[31] = 3.0000000000000000e+01;
acadoVariables.ubAValues[32] = 3.0000000000000000e+01;
acadoVariables.ubAValues[33] = 3.0000000000000000e+01;
acadoVariables.ubAValues[34] = 3.0000000000000000e+01;
acadoVariables.ubAValues[35] = 3.0000000000000000e+01;
acadoVariables.ubAValues[36] = 3.0000000000000000e+01;
acadoVariables.ubAValues[37] = 3.0000000000000000e+01;
acadoVariables.ubAValues[38] = 3.0000000000000000e+01;
acadoVariables.ubAValues[39] = 3.0000000000000000e+01;
acadoVariables.ubAValues[40] = 3.0000000000000000e+01;
acadoVariables.ubAValues[41] = 3.0000000000000000e+01;
acadoVariables.ubAValues[42] = 3.0000000000000000e+01;
acadoVariables.ubAValues[43] = 3.0000000000000000e+01;
acadoVariables.ubAValues[44] = 3.0000000000000000e+01;
acadoVariables.ubAValues[45] = 3.0000000000000000e+01;
acadoVariables.ubAValues[46] = 3.0000000000000000e+01;
acadoVariables.ubAValues[47] = 3.0000000000000000e+01;
acadoVariables.ubAValues[48] = 3.0000000000000000e+01;
acadoVariables.ubAValues[49] = 3.0000000000000000e+01;
acadoVariables.ubAValues[50] = 3.0000000000000000e+01;
acadoVariables.ubAValues[51] = 3.0000000000000000e+01;
acadoVariables.ubAValues[52] = 3.0000000000000000e+01;
acadoVariables.ubAValues[53] = 3.0000000000000000e+01;
acadoVariables.ubAValues[54] = 3.0000000000000000e+01;
acadoVariables.ubAValues[55] = 3.0000000000000000e+01;
acadoVariables.ubAValues[56] = 3.0000000000000000e+01;
acadoVariables.ubAValues[57] = 3.0000000000000000e+01;
acadoVariables.ubAValues[58] = 3.0000000000000000e+01;
acadoVariables.ubAValues[59] = 3.0000000000000000e+01;
acadoVariables.ubAValues[60] = 3.0000000000000000e+01;
acadoVariables.ubAValues[61] = 3.0000000000000000e+01;
acadoVariables.ubAValues[62] = 3.0000000000000000e+01;
acadoVariables.ubAValues[63] = 3.0000000000000000e+01;
acadoVariables.ubAValues[64] = 3.0000000000000000e+01;
acadoVariables.ubAValues[65] = 3.0000000000000000e+01;
acadoVariables.ubAValues[66] = 3.0000000000000000e+01;
acadoVariables.ubAValues[67] = 3.0000000000000000e+01;
acadoVariables.ubAValues[68] = 3.0000000000000000e+01;
acadoVariables.ubAValues[69] = 3.0000000000000000e+01;
acadoVariables.ubAValues[70] = 3.0000000000000000e+01;
acadoVariables.ubAValues[71] = 3.0000000000000000e+01;
acadoVariables.ubAValues[72] = 3.0000000000000000e+01;
acadoVariables.ubAValues[73] = 3.0000000000000000e+01;
acadoVariables.ubAValues[74] = 3.0000000000000000e+01;
acadoVariables.ubAValues[75] = 3.0000000000000000e+01;
acadoVariables.ubAValues[76] = 3.0000000000000000e+01;
acadoVariables.ubAValues[77] = 3.0000000000000000e+01;
acadoVariables.ubAValues[78] = 3.0000000000000000e+01;
acadoVariables.ubAValues[79] = 3.0000000000000000e+01;
acadoVariables.ubAValues[80] = 3.0000000000000000e+01;
acadoVariables.ubAValues[81] = 3.0000000000000000e+01;
acadoVariables.ubAValues[82] = 3.0000000000000000e+01;
acadoVariables.ubAValues[83] = 3.0000000000000000e+01;
acadoVariables.ubAValues[84] = 3.0000000000000000e+01;
acadoVariables.ubAValues[85] = 3.0000000000000000e+01;
acadoVariables.ubAValues[86] = 3.0000000000000000e+01;
acadoVariables.ubAValues[87] = 3.0000000000000000e+01;
acadoVariables.ubAValues[88] = 3.0000000000000000e+01;
acadoVariables.ubAValues[89] = 3.0000000000000000e+01;
acadoVariables.ubAValues[90] = 3.0000000000000000e+01;
acadoVariables.ubAValues[91] = 3.0000000000000000e+01;
acadoVariables.ubAValues[92] = 3.0000000000000000e+01;
acadoVariables.ubAValues[93] = 3.0000000000000000e+01;
acadoVariables.ubAValues[94] = 3.0000000000000000e+01;
acadoVariables.ubAValues[95] = 3.0000000000000000e+01;
acadoVariables.ubAValues[96] = 3.0000000000000000e+01;
acadoVariables.ubAValues[97] = 3.0000000000000000e+01;
acadoVariables.ubAValues[98] = 3.0000000000000000e+01;
acadoVariables.ubAValues[99] = 3.0000000000000000e+01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 100; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 3];
acadoWorkspace.state[1] = acadoVariables.x[index * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 3 + 2];
acadoWorkspace.state[15] = acadoVariables.u[index];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[index * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[index * 3 + 5] = acadoWorkspace.state[2];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 100; ++index)
{
acadoVariables.x[index * 3] = acadoVariables.x[index * 3 + 3];
acadoVariables.x[index * 3 + 1] = acadoVariables.x[index * 3 + 4];
acadoVariables.x[index * 3 + 2] = acadoVariables.x[index * 3 + 5];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[300] = xEnd[0];
acadoVariables.x[301] = xEnd[1];
acadoVariables.x[302] = xEnd[2];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[300];
acadoWorkspace.state[1] = acadoVariables.x[301];
acadoWorkspace.state[2] = acadoVariables.x[302];
if (uEnd != 0)
{
acadoWorkspace.state[15] = uEnd[0];
}
else
{
acadoWorkspace.state[15] = acadoVariables.u[99];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[300] = acadoWorkspace.state[0];
acadoVariables.x[301] = acadoWorkspace.state[1];
acadoVariables.x[302] = acadoWorkspace.state[2];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 99; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[99] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99];
kkt = fabs( kkt );
for (index = 0; index < 100; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 100; ++index)
{
prd = acadoWorkspace.y[index + 100];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 3 */
real_t tmpDy[ 3 ];

/** Row vector of size: 2 */
real_t tmpDyN[ 2 ];

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 3] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 3];
acadoWorkspace.Dy[lRun1 * 3 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 3 + 1];
acadoWorkspace.Dy[lRun1 * 3 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 3 + 2];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[300];
acadoWorkspace.objValueIn[1] = acadoVariables.x[301];
acadoWorkspace.objValueIn[2] = acadoVariables.x[302];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 3]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 3 + 1]*acadoVariables.W[4];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 3 + 2]*acadoVariables.W[8];
objVal += + acadoWorkspace.Dy[lRun1 * 3]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 3 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 3 + 2]*tmpDy[2];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[3];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1];

objVal *= 0.5;
return objVal;
}

