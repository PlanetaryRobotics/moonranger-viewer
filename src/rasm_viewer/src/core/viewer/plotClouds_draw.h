/*
 * Main draw function
 * Iterates over global data and displays it in one of many modes
 *
 * Dominic Jonak
 */
#ifndef PLOT_CLOUDS_DRAW_H
#define PLOT_CLOUDS_DRAW_H

enum drawModes{
  DRAW_MODE_POINT=0,
  DRAW_MODE_WIRE,
  DRAW_MODE_POINT_WIRE,
  DRAW_MODE_MESH,
  DRAW_MODE_CONTOUR,
  NUM_DRAW_MODES
};
#include "helpers.h"

void plotClouds_draw(METERS pointsize);

/* draws unit axes with the specified colors
 * the showAxes global variable affects this function
 */
void drawAxes(float r1, float g1, float b1,
	      float r2, float g2, float b2,
	      float r3, float g3, float b3,
	      bool forceDisplay=false,
	      double scale=1.0);

#endif
