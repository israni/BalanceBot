/*******************************************************************************
* mb_lqg.h
*******************************************************************************/

#ifndef MB_LQG_H
#define MB_LQG_H

#include <rc_usefulincludes.h> 
#include <roboticscape.h>


typedef struct lqg LQG_t;
struct lqg{
	rc_matrix_t A;
	rc_matrix_t B;
    rc_vector_t C;
	rc_vector_t x_current;
	rc_vector_t x_previous;
	rc_vector_t temp1;
	rc_vector_t temp2;
	rc_vector_t inputVector;
	rc_vector_t medianBuffer;
	float filteredSpeed;
	int positionSpeedFlag;
};


//Initialize the LQG controller with gains
LQG_t* LQG_Init();
     
float LQG_Compute(LQG_t* lqg, float command, float pitchMeasurement, int speedTickMeasurement);

#endif
