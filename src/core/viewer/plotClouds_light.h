/*
 * An interface to OpenGL lights
 * Note that this is completely unrelated to shadows, see shadowModel for info
 *
 * Dominic Jonak
 */
#ifndef PLOT_CLOUDS_LIGHT_H
#define PLOT_CLOUDS_LIGHT_H

/* the first time this is called this initializes the lightTable to default */
void initLights();

/* toggles the currently selected light */
void toggleLight();

/* adjusts a lighting parameter in response to a key press */
void adjustLight(char key);

/* sets up the gl context with the lighting parameters */
void lighting();

/* handles a keypress, calls glutPostRedisplay() and returns true
 * returns false if the key is unrelated
 */
bool key_light(unsigned char key,int x, int y);

/* attempts to parse arguments starting at position i
 * returns the number of arguments consumed
 * returns 0 if the i'th parameter is not light related
 */
unsigned int parseArgs_lighting(unsigned int argc, char **argv,
				unsigned int i);

void usage_lighting();

#endif
