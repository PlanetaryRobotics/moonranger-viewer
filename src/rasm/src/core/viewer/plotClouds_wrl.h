#ifndef PLOT_CLOUDS_WRL_H
#define PLOT_CLOUDS_WRL_H

#include "wrl.h"
#include "helpers.h"

/* applies a transform to a wrl model
 * applies the same transform to a raw cloud and mesh
 */
void wrl_applyTransform(const wrl::tuple3f &rotation,
			const wrl::tuple3f &translation,
			wrl *w, points *p=NULL, triangles *t=NULL);

/* attempts to parse arguments starting at position i
 * returns the number of arguments consumed
 * returns 0 if the i'th parameter is not wrl related
 */
unsigned int parseArgs_wrl(unsigned int argc, char **argv, unsigned int i);

void usage_wrl();

#endif
