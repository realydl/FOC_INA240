#include "CurrentSense.h"


// function used with the foc algorihtm
//   calculating DQ currents from phase currents
//   - function calculating park and clarke transform of the phase currents 
//   - using getPhaseCurrents internally

float i_alpha, i_beta;

PhaseCurrent_s realcurrent;

DQCurrent_s getFOCCurrents(float angle_el)
{
	
//	float i_alpha, i_beta;
	float ct,st;
	DQCurrent_s ret;
	
	// read current phase currents
	realcurrent = getPhaseCurrents();
//	realcurrent = getSelectionPhaseCurrents(sec);
	
	// calculate clarke transform
//	if(!current.c)
//	{
		// if only two measured currents
	#if 1
		i_alpha = realcurrent.a;  
		i_beta = _1_SQRT3 * realcurrent.a + _2_SQRT3 * realcurrent.b;//a + b + c = 0
	#endif
	
	#if 0
		i_alpha = _SQRT3_2 * realcurrent.a;
		i_beta = _SQRT3_2 * realcurrent.a + _SQRT3 * realcurrent.b;
	#endif
	
//	}
//	else
//	{
//		// signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
//		float mid = (1.f/3) * (current.a + current.b + current.c);
//		float a = current.a - mid;
//		float b = current.b - mid;
//		i_alpha = a;
//		i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
//	}
	
	// calculate park transform
//	ct = _cos(angle_el);
//	st = _sin(angle_el);
	
	//FPU calculate park transform
	//F407DSP的三角函数计算
	ct = arm_cos_f32(angle_el);
	st = arm_sin_f32(angle_el);
	
	//simplefoc的park变换
	ret.d = i_alpha * ct + i_beta * st;
	ret.q = i_beta * ct - i_alpha * st;
	
	//q轴和d轴互换之后的park变换
//	ret.q = i_alpha * ct + i_beta * st;
//	ret.d = i_beta * ct - i_alpha * st;
	return ret;
}


//DQCurrent_s getFOCCurrents(float angle_el,uint8_t sec)
//{
//	
////	float i_alpha, i_beta;
//	float ct,st;
//	DQCurrent_s ret;
//	
//	// read current phase currents
////	current = getPhaseCurrents();
//	realcurrent = getSelectionPhaseCurrents(sec);
//	
//	// calculate clarke transform
////	if(!current.c)
////	{
//		// if only two measured currents
//		i_alpha = realcurrent.a;  
//		i_beta = _1_SQRT3 * realcurrent.a + _2_SQRT3 * realcurrent.b;//a + b + c = 0
////	}
////	else
////	{
////		// signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
////		float mid = (1.f/3) * (current.a + current.b + current.c);
////		float a = current.a - mid;
////		float b = current.b - mid;
////		i_alpha = a;
////		i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
////	}
//	
//	// calculate park transform
////	ct = _cos(angle_el);
////	st = _sin(angle_el);
//	
//	//FPU calculate park transform
//	//F407DSP的三角函数计算
//	ct = arm_cos_f32(angle_el);
//	st = arm_sin_f32(angle_el);
//	
//	ret.d = i_alpha * ct + i_beta * st;
//	ret.q = i_beta * ct - i_alpha * st;
//	return ret;
//}


Ialphabeta clarkTransform(float angle_el,uint8_t sec)
{
	PhaseCurrent_s current;
//	float i_alpha, i_beta;
//	float ct,st;
	Ialphabeta ret;
	
	current = getSelectionPhaseCurrents(sec);
	// read current phase currents
//	current = getPhaseCurrents();
	
	ret.alpha = current.a;  
	ret.beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;//a + b + c = 0

	return ret;
}

