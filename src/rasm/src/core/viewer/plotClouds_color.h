/*
 * Declaration of color class
 * This streamlines the OpenGL interface and supports hue/HSV
 *
 * Dominic Jonak
 */
#ifndef PLOT_CLOUDS_COLOR_H
#define PLOT_CLOUDS_COLOR_H

class color {
public:
  float r, g, b;
  color();
  color(float percent);
  void setFromHSV(float h, float s, float v);
  void glInvertColor()const;
  void glColor()const;
  void glColor4()const;
  bool similarTo(color *c)const;
};
void createColors(int clearOld=1);

/* attempts to parse arguments starting at position i
 * returns the number of arguments consumed
 * returns 0 if the i'th parameter is not related
 */
unsigned int parseArgs_color(unsigned int argc, char **argv,
			     unsigned int i);

void usage_color();

/* handles a keypress, calls glutPostRedisplay() and returns true
 * returns false if the key is unrelated
 */
bool key_color(unsigned char key,int x, int y);


#endif
