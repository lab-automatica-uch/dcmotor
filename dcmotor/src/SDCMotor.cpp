
extern "C" {

#define S_FUNCTION_NAME  SDCMotor
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "O22SIOMM.h"

/*====================*
 * S-function methods *
 *====================*/


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 1);		// Number of expected parameters

    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;						// Parameter mismatch will be reported by Simulink
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);

    if (!ssSetNumInputPorts(S, 1)) return;
	ssSetInputPortWidth(S,0,1);
	//ssSetInputPortDirectFeedThrough(S,0,0);		// Existen llamadas de la entrada en la funcion mdlOutputs
	ssSetInputPortRequiredContiguous(S,0,1);	// sacado del ejemplo (?)
    
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);	// moya

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);			// reserve element in the float vector
    ssSetNumIWork(S, 0);			// reserve element in the int vector
    ssSetNumPWork(S, 1);			// reserve element in the pointers vector
    ssSetNumModes(S, 0);			// to store a C++ object
    ssSetNumNonsampledZCs(S, 0);	// number of states for which a block detects zero crossings

    ssSetOptions(S, 0);				// set the simulation options that this block implements
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, 0)));	// tiempo de muestreo?
    ssSetOffsetTime(S, 0, 0.0);
}

/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
#define MDL_START
#if defined(MDL_START) 
static void mdlStart(SimStruct *S)
{
	O22SnapIoMemMap *Brain;
	long nResult;

	Brain = new O22SnapIoMemMap();
	nResult = Brain->OpenEnet("192.168.6.101", 2001, 10000, 1);
	//mexPrintf("openenet: %d\n",nResult);

	if ( nResult == SIOMM_OK )
	{
		nResult = Brain->IsOpenDone();
		//mexPrintf("	isopendone: %d\n",nResult);
		while ( nResult == SIOMM_ERROR_NOT_CONNECTED_YET )
		{
			nResult = Brain->IsOpenDone();
			//mexPrintf("  isopendone: %d\n",nResult);
		} 
	}

	// Check for error on OpenEnet() and IsOpenDone()
	if ( nResult != SIOMM_OK )
	{
		ssSetErrorStatus(S,"No se pudo realizar la conexion con exito.");
		return;
	}

	ssGetPWork(S)[0] = (void *) Brain;
}
#endif /*  MDL_START */

/* Function: mdlUpdate ========================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector, ssGetY(S).
 */
#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
	O22SnapIoMemMap *Brain;
	long nResult;

	Brain = (O22SnapIoMemMap *) ssGetPWork(S)[0];

	const real_T *u = ssGetInputPortRealSignal(S,0);

    nResult=Brain->SetAnaPtValue(0,(float)*u);
	if ( nResult != SIOMM_OK )
	{
		//mexPrintf("setanaptvalue: %d\n",nResult);
		ssSetErrorStatus(S,"Error al transmitir el dato de voltaje.");
		return;
	}
}


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector, ssGetY(S).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
	O22SnapIoMemMap *Brain;
	float velocidad;
	long nResult;

	Brain = (O22SnapIoMemMap *) ssGetPWork(S)[0];

	real_T *y = ssGetOutputPortRealSignal(S,0);

	nResult=Brain->GetAnaPtValue(4,&velocidad);
	if ( nResult != SIOMM_OK )
	{
		//mexPrintf("getanaptvalue: %d\n",nResult);
		ssSetErrorStatus(S,"Error al recibir el dato de velocidad.");
		return;
	}

    y[0] = (real_T)velocidad;
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
	O22SnapIoMemMap *Brain;

	Brain = (O22SnapIoMemMap *) ssGetPWork(S)[0];
    Brain->SetAnaPtValue(0,0.0);
	Brain->Close();

	delete Brain;
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


} // end of extern "C" scope

