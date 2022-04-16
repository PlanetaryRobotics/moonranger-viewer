#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <sys/time.h> // gettimeofday
#include <util.h>


double now()
{
  struct timeval tv = {0,0};
  gettimeofday(&tv, NULL);
  return (double)(tv.tv_sec*1.0) + (double)(tv.tv_usec*1.0) / 1000000.0;
}

bool atob(const char* s)
{
  assert(s != NULL);
  if(strcasecmp(s, "true") == 0) 
    {
      return true;
    } 
  else
    {
      return false;
    }
}
