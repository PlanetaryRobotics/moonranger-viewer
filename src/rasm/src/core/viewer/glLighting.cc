#include "glLighting.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

unsigned int lights::selectedLight=0;
unsigned int lightData::selectedAdjustment=0;

lights::lights():numLights(0),data(NULL){}

lights::~lights(){
  for(unsigned int i=0;i<numLights;i++)
    delete data[i];
  free(data);
}

void lights::expandTo(int n){
  while((int)num() < n){
    lightData *l = new lightData((int)num());
    l->on = 0;//sure new lights are initially off
    add(l);
  }
}

void lights::print()const{
  for(unsigned int i=0;i<numLights;i++)
    data[i]->print(i==selectedLight);
}

void lights::gl()const{
  for(unsigned int i=0;i<numLights;i++)
    data[i]->gl();
}

void lights::add(lightData *l){
  numLights++;
  data = (lightData **)realloc(data, numLights*sizeof(lightData *));
  data[numLights-1] = l;
}

lightData *lights::get(int n)const{
  if(n<0 || n>=(int)numLights)return NULL;
  return data[n];
}

lightData *lights::getSelected()const{
  return get(selectedLight);
}

void lights::cycleLight()const{
  if(numLights>1)
    selectedLight = (selectedLight+1)%numLights;
}

void lightData::defaults(){
  glNum = 0;
  on = 1;
  stillDefault = 1;
  pos[0] = pos[1] = pos[2] = 0.0;
  pos[3] = 1.0;
  ambient[0] = ambient[1] = ambient[2] = 0.05;
  ambient[3] = 0.9;
  diffuse[0] = diffuse[1] = diffuse[2] = 0.35;
  diffuse[3] = 0.9;
  specular[0] = specular[1] = specular[2] = 0.5;
  specular[3] = 0.1;
  attenuation[0] = 1.0;
  attenuation[1] = attenuation[2] = attenuation[3] = 0.0;
}

lightData::lightData(int _glNum){
  defaults();
  glNum = GL_LIGHT0 + _glNum;
}

lightData::lightData(){
  defaults();
  glNum = GL_LIGHT0;
}

void lightData::print(int isSelected)const{
  printf("\nlight %d %s %s", glNum-GL_LIGHT0, on?"ON":"OFF",
	 (isSelected)?"(selected)":"");
  const char *adj = getAdjustment(selectedAdjustment);
#define PRINT4(var) \
do{\
  const char *mark = " ";\
  if(!strcmp(adj,#var))mark="*";\
  printf("\n %s%s %0.3f,%0.3f,%0.3f,%0.3f", mark, #var, var[0],var[1],var[2],var[3]);\
}while(0)
  PRINT4(pos);
  PRINT4(ambient);
  PRINT4(diffuse);
  PRINT4(specular);
  PRINT4(attenuation);
#undef PRINT4
  printf("\n");
}

void lightData::gl() const{
  glLightfv(glNum, GL_POSITION, pos);
  glLightfv(glNum, GL_AMBIENT,  ambient);
  glLightfv(glNum, GL_DIFFUSE,  diffuse);
  glLightfv(glNum, GL_SPECULAR, specular);
  glLightfv(glNum, GL_CONSTANT_ATTENUATION,  &(attenuation[0]));
  glLightfv(glNum, GL_LINEAR_ATTENUATION,    &(attenuation[1]));
  glLightfv(glNum, GL_QUADRATIC_ATTENUATION, &(attenuation[2]));
  if(on)glEnable(glNum);
  else glDisable(glNum);
}

void lightData::setpos(float x, float y, float z){
  stillDefault = 0;
  pos[0] = x;
  pos[1] = y;
  pos[2] = z;
}

void lightData::applyLimits(const char *type){
  if(!strcmp(type, "ambient")){
    for(int i=0;i<4;i++){
      if(ambient[i]>=1.0)ambient[i]=1.0;
      if(ambient[i]<=-1.0)ambient[i]=-1.0;
      if(fabs(ambient[i])<0.001)ambient[i]=0;
    }
    return;
  }
  
  if(!strcmp(type, "diffuse")){
    for(int i=0;i<4;i++){
      if(diffuse[i]>=1.0)diffuse[i]=1.0;
      if(diffuse[i]<=-1.0)diffuse[i]=-1.0;
      if(fabs(diffuse[i])<0.001)diffuse[i]=0;
    }
    return;
  }

  if(!strcmp(type, "specular")){
    for(int i=0;i<4;i++){
      if(specular[i]>=1.0)specular[i]=1.0;
      if(specular[i]<=-1.0)specular[i]=-1.0;
      if(fabs(specular[i])<0.001)specular[i]=0;
    }
    return;
  }

  if(!strcmp(type, "attenuation")){
    for(int i=0;i<3 /* 4th is unused */;i++){
      if(attenuation[i]<=0.0)attenuation[i]=0.0;
      if(fabs(attenuation[i])<0.001)attenuation[i]=0;
    }
    return;
  }

  if(!strcmp(type, "pos")){
    for(int i=0;i<4;i++){
      if(fabs(pos[i])<0.01)pos[i]=0;
    }
    return;
  }

  assert(0);
}

void lightData::set(const char *type, float amount[4]){
  stillDefault = 0;
  assert(validAdjustment(type));
  int done=0;
#define SET_LIGHT(var)                                                      \
  if(!done && !strcmp(type, #var)){                                         \
    for(int i=0;i<4;i++)                                                    \
      var[i] = amount[i];                                                   \
    printf("Set %s of light %d to %g,%g,%g,%g\n", #var, glNum-GL_LIGHT0, var[0], var[1], var[2], var[3]);\
    done=1;                                                                 \
  }
  SET_LIGHT(pos)
  SET_LIGHT(ambient)
  SET_LIGHT(diffuse)
  SET_LIGHT(specular)
  SET_LIGHT(attenuation)
#undef SET_LIGHT
  assert(done);
  //print();
}

void lightData::adjust(const char *type, float amount[4]){
  stillDefault = 0;
  assert(validAdjustment(type));
  int done=0;
#define ADJUST_LIGHT(var)                                                   \
  if(!strcmp(type, #var)){                                                  \
    for(int i=0;i<4;i++)                                                    \
      var[i] += amount[i];                                                  \
    applyLimits(#var);                                                      \
    printf("Adjusted %s of light %d to %g,%g,%g,%g\n", #var, glNum-GL_LIGHT0, var[0], var[1], var[2], var[3]);\
    done=1;                                                                 \
  }
  ADJUST_LIGHT(pos)
  ADJUST_LIGHT(ambient)
  ADJUST_LIGHT(diffuse)
  ADJUST_LIGHT(specular)
  ADJUST_LIGHT(attenuation)
#undef ADJUST_LIGHT
  assert(done);
  print(0);
}

int lightData::validAdjustment(const char *type){
  for(unsigned int i=0;i<NUM_ADJUSTMENTS;i++)
    if(!strcmp(type, getAdjustment(i)))
      return 1;
  return 0;
}

const char *lightData::getAdjustment(int type){
  switch(type){
  case 0:return "pos";
  case 1:return "ambient";
  case 2:return "diffuse";
  case 3:return "specular";
  case 4:return "attenuation";
  }
  assert(0);
}
