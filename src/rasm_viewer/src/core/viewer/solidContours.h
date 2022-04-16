/*
 * Dominic Jonak (dom@cmu.edu)
 *
 * Helper for rendering a triangle as a set of contour levels.
 * Expected to be used to render an entire mesh as a contour map.
 */
#ifndef SOLID_CONTOURS_H
#define SOLID_CONTOURS_H

#include "helpers.h"

void drawSolidContour(const point &a, const point &b, const point &c,
		      METERS spacing, METERS colorRepeat, int rainbow);

#endif
