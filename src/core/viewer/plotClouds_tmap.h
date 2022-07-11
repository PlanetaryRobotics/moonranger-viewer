/*
 * Helper functions to receive and display a tmap
 *
 * Dominic Jonak
 */
#ifndef PLOT_CLOUDS_TMAP_H
#define PLOT_CLOUDS_TMAP_H

void usage_tmap();

/* attempts to parse arguments starting at position i
 * returns the number of arguments consumed
 * returns 0 if the i'th parameter is not related
 */
unsigned int parseArgs_tmap(unsigned int argc, char **argv,
			    unsigned int i);

/* prepares for receiving tmaps */
void init_tmap(const char** argv, int argc);

/* check on tmap data if needed */
void idle_tmap();

/* draws the vehicle pose over the tmap */
void draw_tmap();

#endif
