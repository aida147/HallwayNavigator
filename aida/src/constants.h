/*
 * constants.h
 *
 *  Created on: Jun 15, 2014
 *      Author: aida
 */

#ifndef __CONSTANTS_H_INCLUDED__
#define __CONSTANTS_H_INCLUDED__

const int STEPS = 4, NUM_LOCATIONS = 10, MAX_TRACKER = 10;
const double LOOP_RATE = 10;
const float EPS = 1e-8, INF = 1e8;
char FRAME_ID[] = "/camera_depth_optical_frame";

enum {HALLWAY, TURN, RESET, DETECTOR};
enum {LEFT, RIGHT, CENTER};	//based on robot's view {y > 0, y < 0, y = 0}
enum {ERROR_MSG, STATUS_TEXT, VELOCITY_TEXT, CENTER_LINE};
enum {SWEEP, ROTATE};
#endif
