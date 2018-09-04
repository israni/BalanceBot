/*******************************************************************************
* mb_odometry.h
*
* 
*******************************************************************************/
#ifndef MB_ODOMETRY_H
#define MB_ODOMETRY_H

#include "mb_defs.h"
#include "mb_structs.h"

rc_ringbuf_t encoderbuff_lo;
rc_ringbuf_t encoderbuff_ro;
rc_ringbuf_t gyrobuff;
rc_filter_t odoFilterz_l;
rc_filter_t odoFilterz_r;

void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta);
void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state);
float mb_clamp_radians(float angle);
int mb_in_range(float num, float min, float max);

#endif