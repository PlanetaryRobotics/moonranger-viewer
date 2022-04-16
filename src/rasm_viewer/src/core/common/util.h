/**
 * @file util.h
 *
 * @section LICENSE
 * Copyright 2012, Carnegie Mellon University and ProtoInnovations, LLC. 
 * All rights reserved. This document contains information that is proprietary
 * to Carnegie Mellon University and ProtoInnovations, LLC. Do not distribute
 * without approval.
 *
 */

#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <sys/time.h> // gettimeofday
#include <util.h>

#ifdef __cplusplus
extern "C" 
{
#endif //__cplusplus

/*
 * Returns the number of seconds since 1970-01-01, down to microseconds
 * resolution. On an error, 0 will be returned.
 */
double now();

bool atob(const char* s);

#ifdef __cplusplus
} // extern "C"
#endif //__cplusplus


#endif /* __UTIL_H__ */
