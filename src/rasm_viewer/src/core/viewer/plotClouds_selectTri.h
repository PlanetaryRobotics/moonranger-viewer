/*
 * Helper functions to find the triangle pointed to by the cursor
 *
 * Dominic Jonak
 */
#ifndef PLOT_CLOUDS_SELECT_TRI_H
#define PLOT_CLOUDS_SELECT_TRI_H

#include "helpers.h"

void findTri(const point &origin, const point &pointOnLine);
void selectTriangle(int mouseX, int mouseY);

/* handles a keypress, calls glutPostRedisplay() and returns true
 * returns false if the key is unrelated
 */
bool key_select(unsigned char key,int x, int y);

#endif
