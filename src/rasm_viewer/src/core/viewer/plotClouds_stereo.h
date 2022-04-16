/*
 * Helper functions to generate and display a point cloud
 *
 * Dominic Jonak
 */
#ifndef PLOT_CLOUDS_STEREO_H
#define PLOT_CLOUDS_STEREO_H

void usage_stereo();

/* attempts to parse arguments starting at position i
 * returns the number of arguments consumed
 * returns 0 if the i'th parameter is not related
 */
unsigned int parseArgs_stereo(unsigned int argc, char **argv,
			      unsigned int i);

/* prepares structures for stereo and gets initial camera images */
void init_stereo();

/* check on the background stereo thread if needed */
void idle_stereo();

/* draws the camera/rectified/disparity image */
void draw_stereo(int width, int height);

#endif
