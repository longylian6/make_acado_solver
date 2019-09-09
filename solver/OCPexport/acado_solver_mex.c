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
input
{
	control: optional
		0: init once, and run preparation and feedback; default behaviour
		1: initialize
		2: preparation
		3: feedback
		4: shift
	x
	u
	od
	y
	yN
	W
	WN
	x0: depends on the type of an OCP
	xAC: optional
	SAC: optional
	shifting: optional
	{
		strategy:
			1: use xEnd
			2: integrate
		xEnd
		uEnd
	}
	initialization: optional
		1: initialize by a forward simulation
		else: do nothing
}

output
{
	x
	u
	xAC: optional
	SAC: optional
	info
	{
		status
		cpuTime
		kktValue
		objValue
		nIterations: works only for qpOASES
	}	
}
*/

/** MEX interface for the ACADO OCP solver
 *
 *  \author Milan Vukov, milan.vukov@esat.kuleuven.be
 *
 *  Credits: Alexander Domahidi (ETHZ), Janick Frasch (KUL, OVGU)
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "mex.h"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define FREE( mem ) { if( mem ) { mxFree( mem ); mem = NULL; } }

/* Define number of outputs */
#define NOO 3

#if ACADO_NXA > 0
#define NOO_2 NOO + 1
#else
#define NOO_2 NOO
#endif

#if ACADO_USE_ARRIVAL_COST == 1
#define NOO_3 NOO_2 + 2
#else
#define NOO_3 NOO_2
#endif

#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
#define NOO_4 NOO_3 + 1
#else
#define NOO_4 NOO_3
#endif

/** Instance of the user data structure. */
ACADOvariables acadoVariables;
/** Instance of the private workspace structure. */
ACADOworkspace acadoWorkspace;

#if ACADO_QP_SOLVER == ACADO_FORCES
#include "forces.h"
extern forces_info acadoForces_info;
#elif ACADO_QP_SOLVER == ACADO_QPDUNES
#include <qpDUNES.h>
extern qpData_t    qpData;
#endif

/** A bit more advanced printing function. */
void mexErrMsgTxtAdv(	char* string,
						...
						)
{
	static char buffer[ 128 ];
	
	va_list printArgs;
	va_start(printArgs, string);
	
	vsprintf(buffer, string, printArgs);
	va_end( printArgs );

	mexErrMsgTxt( buffer );
}

/** A simple helper function. */
void printMatrix(	const char* name,
					real_t* mat,
					unsigned nRows,
					unsigned nCols
					)
{
    unsigned r, c;
    mexPrintf("%s: \n", name);
    for (r = 0; r < nRows; ++r)
    {
        for(c = 0; c < nCols; ++c)
            mexPrintf("\t%f", mat[r * nCols + c]);
        mexPrintf("\n");
    }
}

/** A function for copying data from MATLAB to C array. */
int getArray(	const unsigned mandatory,
				const mxArray* source,
				const int index,
				const char* name,
				real_t* destination,
				const unsigned nRows,
				const unsigned nCols
				)
{
	mxArray* mxPtr = mxGetField(source, index, name);
	unsigned i, j;
	double* dPtr;
	
	if (mxPtr == NULL)
	{
		if ( !mandatory )
			return -1;
		else
			mexErrMsgTxtAdv("Field %s not found.", name);
	}

    if ( !mxIsDouble( mxPtr ) )
		mexErrMsgTxtAdv("Field %s must be an array of doubles.", name);

    if (mxGetM( mxPtr ) != nRows || mxGetN( mxPtr ) != nCols )
		mexErrMsgTxtAdv("Field %s must be of size: %d x %d.", name, nRows, nCols);

	dPtr = mxGetPr( mxPtr );
	
	if (destination == NULL)
		destination = (real_t*)mxCalloc(nRows * nCols, sizeof( real_t ));

	if (nRows == 1 && nCols == 1)
		*destination = *dPtr;
	else
		for (i = 0; i < nRows; ++i)
			for (j = 0; j < nCols; ++j)
				destination[i * nCols + j] = (real_t)dPtr[j * nRows + i];
			
	return 0;
}

void setArray( 	mxArray* destination,
				const int index,
				const char* name,
				real_t* source,
				const unsigned nRows,
				const unsigned nCols
				)
{
	mxArray* mxPtr = mxCreateDoubleMatrix(nRows, nCols, mxREAL);
	double* dPtr = mxGetPr( mxPtr );
	unsigned i, j;
	
	if (nRows == 1 && nCols == 1)
		*dPtr = *source;
	else
		for (i = 0; i < nRows; ++i)
			for(j = 0; j < nCols; ++j)
				dPtr[j * nRows + i] = (double)source[i * nCols + j];

	mxSetField(destination, index, name, mxPtr);
}

/** The MEX interface function. */
void mexFunction(	int nlhs,
					mxArray *plhs[],
					int nrhs,
					const mxArray *prhs[]
					)
{
	static unsigned initialized = 0;
	unsigned ctrl;
	int ctrlIndex;
	unsigned strategy;
	unsigned initType;
	real_t* xEnd = NULL;
	real_t* uEnd = NULL;
	const mxArray* src = prhs[ 0 ];
	
	const char *infoNames[ 5 ] = {"status", "cpuTime", "kktValue", "objValue", "nIterations"};
	mxArray* info;
	real_t status, cpuTime, kktValue, objValue;
	double tmp[ 1 ];
	mxArray* shPtr;
#ifndef _DSPACE
	acado_timer tmr;
#endif
	double nIterations = 0;
	
	const char *outNames[ NOO_4 ];
	outNames[ 0 ] = "info";
	outNames[ 1 ] = "x";
	outNames[ 2 ] = "u";
#if ACADO_NXA
	outNames[ NOO ] = "z";
#endif	
		
#if ACADO_USE_ARRIVAL_COST == 1
	outNames[ NOO_2 ] = "xAC";
	outNames[NOO_2  + 1] = "SAC";
#endif
#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
	outNames[ NOO_3 ] = "sigmaN";
#endif
	
	if (nrhs != 1)
		mexErrMsgTxt(
			"This function requires exactly one input: a structure with parameters.");
			
	if (nlhs != 1)
		mexErrMsgTxt(
			"This function returns one output.");
			
	if( !mxIsStruct( src ) )
		mexErrMsgTxt("The function argument must be a structure.");
	
	/* Get the control flag. */
	if (getArray(0, src, 0, "control", tmp, 1, 1) == 0)
		ctrl = (unsigned)tmp[ 0 ];
	else
		ctrl = 0;
		
	/* Get the initialization flag. */
	if (getArray(0, src, 0, "initialization", tmp, 1, 1) == 0)
		initType = (unsigned)tmp[ 0 ];
	else
		initType = 0;
		
	/* Copy MATLAB arrays to C arrays. */
	getArray(1, src, 0, "x", acadoVariables.x, ACADO_N + 1, ACADO_NX);
	getArray(1, src, 0, "u", acadoVariables.u, ACADO_N, ACADO_NU);
	
#if ACADO_NXA	
	getArray(1, src, 0, "z", acadoVariables.z, ACADO_N, ACADO_NXA);
#endif	
	
#if ACADO_NOD
	getArray(1, src, 0, "od", acadoVariables.od, ACADO_N + 1, ACADO_NOD);
#endif
	
	getArray(1, src, 0, "y", acadoVariables.y, ACADO_N, ACADO_NY);
	
#if ACADO_NYN
	getArray(1, src, 0, "yN", acadoVariables.yN, 1, ACADO_NYN);
#endif /* ACADO_NYN */

#if ACADO_INITIAL_STATE_FIXED
	getArray(1, src, 0, "x0", acadoVariables.x0, 1, ACADO_NX);
#endif /* ACADO_INITIAL_STATE_FIXED */

#if ACADO_WEIGHTING_MATRICES_TYPE == 1
	getArray(1, src, 0, "W", acadoVariables.W, ACADO_NY, ACADO_NY);
	getArray(1, src, 0, "WN", acadoVariables.WN, ACADO_NYN, ACADO_NYN);
#elif ACADO_WEIGHTING_MATRICES_TYPE == 2
	getArray(1, src, 0, "W", acadoVariables.W, ACADO_N * ACADO_NY, ACADO_NY);
	getArray(1, src, 0, "WN", acadoVariables.WN, ACADO_NYN, ACADO_NYN);
#endif /* ACADO_WEIGHTING_MATRICES_TYPE */


#if (ACADO_HARDCODED_CONSTRAINT_VALUES == 0)
	if (!initialized)
	{
		acado_initializeSolver();
	}
#endif

#if (ACADO_HARDCODED_CONSTRAINT_VALUES == 0) && ( (ACADO_QP_SOLVER == ACADO_QPOASES) || (ACADO_QP_SOLVER == ACADO_QPOASES3) )
	/* Bounds */
#if ACADO_INITIAL_STATE_FIXED == 1
	getArray(1, src, 0, "lbValues", acadoVariables.lbValues, ACADO_N * ACADO_NU, 1);
	getArray(1, src, 0, "ubValues", acadoVariables.ubValues, ACADO_N * ACADO_NU, 1);
#else
	getArray(1, src, 0, "lbValues", acadoVariables.lbValues, ACADO_NX + ACADO_N * ACADO_NU, 1);
	getArray(1, src, 0, "ubValues", acadoVariables.ubValues, ACADO_NX + ACADO_N * ACADO_NU, 1);
#endif /* ACADO_INITIAL_STATE_FIXED == 0 */

#if QPOASES_NCMAX > 0
	/* Affine constraints */
	getArray(1, src, 0, "lbAValues", acadoVariables.lbAValues, QPOASES_NCMAX, 1);
	getArray(1, src, 0, "ubAValues", acadoVariables.ubAValues, QPOASES_NCMAX, 1);
#endif /* QPOASES_NCMAX > 0 */

#elif (ACADO_HARDCODED_CONSTRAINT_VALUES == 0) && (ACADO_QP_SOLVER == ACADO_FORCES) && !ACADO_BLOCK_CONDENSING
	/* Bounds */
	#if (ACADO_QP_NLB > 0)
	getArray(1, src, 0, "lbValues", acadoVariables.lbValues, ACADO_QP_NLB, 1);
	#endif
	#if (ACADO_QP_NUB > 0)
	getArray(1, src, 0, "ubValues", acadoVariables.ubValues, ACADO_QP_NUB, 1);
	#endif

#elif (ACADO_HARDCODED_CONSTRAINT_VALUES == 0) && (ACADO_QP_SOLVER == ACADO_QPDUNES) && !ACADO_BLOCK_CONDENSING
	/* Bounds */
	getArray(1, src, 0, "lbValues", acadoVariables.lbValues, ACADO_QP_NV, 1);
	getArray(1, src, 0, "ubValues", acadoVariables.ubValues, ACADO_QP_NV, 1);
	
#elif (ACADO_HARDCODED_CONSTRAINT_VALUES == 0) && ACADO_BLOCK_CONDENSING
	/* Bounds */
	getArray(1, src, 0, "lbValues", acadoVariables.lbValues, ACADO_QP_NV, 1);
	getArray(1, src, 0, "ubValues", acadoVariables.ubValues, ACADO_QP_NV, 1);
	#if (ACADO_QP_NCA > 0)
	/* Affine bounds */
	getArray(1, src, 0, "lbAValues", acadoVariables.lbAValues, ACADO_QP_NCA, 1);
	getArray(1, src, 0, "ubAValues", acadoVariables.ubAValues, ACADO_QP_NCA, 1);
	#endif
#elif (ACADO_HARDCODED_CONSTRAINT_VALUES == 0) && (ACADO_QP_SOLVER == ACADO_GENERIC)
	/* Bounds */
	getArray(1, src, 0, "lbValues", acadoVariables.lbValues, ACADO_N*(ACADO_NX+ACADO_NU), 1);
	getArray(1, src, 0, "ubValues", acadoVariables.ubValues, ACADO_N*(ACADO_NX+ACADO_NU), 1);
	#if (ACADO_NPAC > 0)
	/* Affine bounds */
	getArray(1, src, 0, "lbAValues", acadoVariables.lbAValues, ACADO_N*ACADO_NPAC, 1);
	getArray(1, src, 0, "ubAValues", acadoVariables.ubAValues, ACADO_N*ACADO_NPAC, 1);
	#endif
#endif

#if ACADO_USE_ARRIVAL_COST == 1
	getArray(1, src, 0, "xAC", acadoVariables.xAC, ACADO_NX, 1);
	getArray(1, src, 0, "SAC", acadoVariables.SAC, ACADO_NX, ACADO_NX);
    getArray(1, src, 0, "WL",  acadoVariables.WL,  ACADO_NX, ACADO_NX);
#endif

	/* Shifting strategy */
	shPtr = mxGetField(src, 0, "shifting");
	if (shPtr != NULL)
	{
		if( !mxIsStruct( shPtr ) )
			mexErrMsgTxt("Field \"shifting\" must be defined with a structure.");
		
		/* Get the shifting strategy flag */
		getArray(1, shPtr, 0, "strategy", tmp, 1, 1);
		strategy = (unsigned)tmp[ 0 ];
		
		if (strategy > 2)
			mexErrMsgTxt("Valid options for the shifting strategy are 1 or 2.");
	
		getArray(0, shPtr, 0, "xEnd", xEnd, ACADO_NX, 1);
		getArray(0, shPtr, 0, "uEnd", uEnd, ACADO_NU, 1);
	}
	else
		strategy = 0;
		
#ifndef _DSPACE
	acado_tic( &tmr );
#endif
	
	/* Call solver */
	switch ( ctrl )
	{
		case 0:
			/* Simple operational mode. Run one RTI with optional shifting. */
			
			if ( !initialized )
			{
				memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
			
#if ACADO_HARDCODED_CONSTRAINT_VALUES == 1
				acado_initializeSolver();
#endif /* ACADO_HARDCODED_CONSTRAINT_VALUES == 1 */
				
				if (initType == 1)
				{
					acado_initializeNodesByForwardSimulation();
				}
				
#if ACADO_USE_ARRIVAL_COST == 1 
                acado_updateArrivalCost( 1 );
#endif /* ACADO_USE_ARRIVAL_COST == 1 */
				
				initialized = 1;
			}
			else if (strategy == 1 || strategy == 2)
			{
#if ACADO_USE_ARRIVAL_COST == 1 
                acado_updateArrivalCost( 0 );
#endif /* ACADO_USE_ARRIVAL_COST == 1 */
				
				acado_shiftStates(strategy, xEnd, uEnd);
				acado_shiftControls(uEnd);
			}
			
			acado_preparationStep();
			
			status = (real_t)acado_feedbackStep();
			
			kktValue = acado_getKKT();
			objValue = acado_getObjective();

#if ( (ACADO_QP_SOLVER == ACADO_QPOASES) || (ACADO_QP_SOLVER == ACADO_QPOASES3) )
			nIterations = (double)acado_getNWSR();
#elif ACADO_QP_SOLVER == ACADO_FORCES
			nIterations = acadoForces_info.it;
#elif ACADO_QP_SOLVER == ACADO_QPDUNES
			nIterations = qpData.log.numIter;
#endif /* ACADO_QP_SOLVER */
			
			break;
		
		case 1:
			/* Initialize */
			
			memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
			
			acado_initializeSolver();
				
			if (initType == 1)
			{
				acado_initializeNodesByForwardSimulation();
			}
			
#if ACADO_USE_ARRIVAL_COST == 1 
			acado_updateArrivalCost( 1 );
#endif /* ACADO_USE_ARRIVAL_COST == 1 */

			initialized = 1;
			
			break;
		
		case 2:
			/* Preparation step */
			
			acado_preparationStep();
			
			break;
		
		case 3:
			/* Feedback step */
			
			status = (real_t)acado_feedbackStep();
			
			kktValue = acado_getKKT();
			objValue = acado_getObjective();
			
#if ( (ACADO_QP_SOLVER == ACADO_QPOASES) || (ACADO_QP_SOLVER == ACADO_QPOASES3) )
			nIterations = (double)acado_getNWSR();
#elif ACADO_QP_SOLVER == ACADO_FORCES
			nIterations = acadoForces_info.it;
#endif /* ACADO_QP_SOLVER */
			
			break;
		
		case 4:
			/* Shifting */
			
#if ACADO_USE_ARRIVAL_COST == 1 
			acado_updateArrivalCost( 0 );
#endif /* ACADO_USE_ARRIVAL_COST == 1 */			
			
			acado_shiftStates(strategy, xEnd, uEnd);
			acado_shiftControls( uEnd );
			
			break;
			
		default:
			/* Return an error */
			mexErrMsgTxt("Unknown control code.");
	}
	
#ifndef _DSPACE
	cpuTime = (real_t)acado_toc( &tmr );
#endif
	
	/* Prepare return argument */
	
	plhs[ 0 ] = mxCreateStructMatrix(1, 1, NOO_4, outNames);
		
	setArray(plhs[ 0 ], 0, "x", acadoVariables.x, ACADO_N + 1, ACADO_NX);
	setArray(plhs[ 0 ], 0, "u", acadoVariables.u, ACADO_N, ACADO_NU);
#if ACADO_NXA > 0
	setArray(plhs[ 0 ], 0, "z", acadoVariables.z, ACADO_N, ACADO_NXA);
#endif	
		
#if ACADO_USE_ARRIVAL_COST == 1
	setArray(plhs[ 0 ], 0, "xAC", acadoVariables.xAC, ACADO_NX, 1);
	setArray(plhs[ 0 ], 0, "SAC", acadoVariables.SAC, ACADO_NX, ACADO_NX);
#endif /* ACADO_USE_ARRIVAL_COST */

#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
	setArray(plhs[ 0 ], 0, "sigmaN", acadoVariables.sigmaN, ACADO_NX, ACADO_NX);
#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

	/* Create the info structure. */
	info = mxCreateStructMatrix(1, 1, 5, infoNames);
		
	setArray(info, 0, "status", &status, 1, 1);
	setArray(info, 0, "cpuTime", &cpuTime, 1, 1);
	setArray(info, 0, "kktValue", &kktValue, 1, 1);
	setArray(info, 0, "objValue", &objValue, 1, 1);
#if ( (ACADO_QP_SOLVER == ACADO_QPOASES) || (ACADO_QP_SOLVER == ACADO_QPOASES3) )
	setArray(info, 0, "nIterations", &nIterations, 1, 1);
#endif /* ( (ACADO_QP_SOLVER == ACADO_QPOASES) || (ACADO_QP_SOLVER == ACADO_QPOASES3) ) */
		
	mxSetField(plhs[ 0 ], 0, "info", info);
	
	/* Cleanup of the allocated memory */
	FREE( xEnd );
	FREE( uEnd );
}
