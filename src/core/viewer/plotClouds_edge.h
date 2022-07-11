/*
 * Helper functions with simple filters to clean up meshes
 *
 * Dominic Jonak
 */
#ifndef PLOT_CLOUDS_EDGE_H
#define PLOT_CLOUDS_EDGE_H

#include "helpers.h"

/* determine the mean and standard deviation of edge lengths in a mesh */
METERS meanEdgeLength(const triangles *t);
METERS stdEdgeLength(const triangles *t, METERS m);

/* check if an edge of a triangle to long or short
 * compared to the mean and allowed deviation
 */
bool shouldPruneLarge(const triangle &t, METERS mean, METERS allowedDeviation);
bool shouldPruneSmall(const triangle &t, METERS mean, METERS allowedDeviation);


/* check if the triangle is vertical,
 * the normal vector must point at least this far from level
 */
bool shouldPruneVertical(const triangle &t, RADIANS minAngle);


/* given a mesh generate a new mesh without the triangles
 * that have long or short edge, or are too close to vertical
 */
triangles *pruneLongEdges(const triangles *input, float deviations);
triangles *pruneShortEdges(const triangles *input, float deviations);
triangles *pruneVertical(const triangles *input, RADIANS minAngle);

#endif
