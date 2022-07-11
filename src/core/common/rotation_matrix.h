/**
 * @file rotation_matrix.h
 *
 * @section LICENSE
 * Copyright 2012, Carnegie Mellon University and ProtoInnovations, LLC. 
 * All rights reserved. This document contains information that is proprietary
 * to Carnegie Mellon University and ProtoInnovations, LLC. Do not distribute
 * without approval.
 *
 */

#ifndef ROTATION_MATRIX_H
#define ROTATION_MATRIX_H

/* computes A = B*C */
void matrixMult(float A[3][3], const float B[3][3], const float C[3][3]);
void matrixMult(float A[4][4], const float B[4][4], const float C[4][4]);

/* builds a rotation matrix, specify angles in radians */
void rotationMatrix(double roll, double pitch, double yaw, float M[3][3]);
void rotationMatrix(double roll, double pitch, double yaw, float M[4][4]);

#endif
