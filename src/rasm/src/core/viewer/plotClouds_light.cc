#include "plotClouds_light.h"
#include "plotClouds_globals.h"

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include <GL/gl.h>


#include "camera.h"
extern SimpleCamera *cam;


#define DEFAULT_NUM_LIGHTS 3

void initLights(){
  if(lightTable)return;
  lightTable = new lights();
  
  for(int i=0;i<DEFAULT_NUM_LIGHTS;i++)
    lightTable->add(new lightData(i));
  assert(DEFAULT_NUM_LIGHTS == lightTable->num());
}

void toggleLight(){
  lightData *l = lightTable->getSelected();
  if(!l)return;
  l->stillDefault = 0;
  l->on = !(l->on);
  printf("Light %d is now %s\n", lightTable->selected(), ((l->on)?"ON":"OFF"));
  l->print(1);
}

void adjustLight(char key){
  lightData *l = lightTable->getSelected();
  if(!l)return;

  assert(cam);
  const char *adjustment = lightData::getAdjustment(lightData::selectedAdjustment);
  float mag = 0.1;/* default increment */
  if(!strcmp(adjustment, "pos")){
    /* when adjusting position, base magnitude on the camera distance */
    mag = cam->distance/20.0;

    /* for the w component (keys u and j) we only allow 0 or 1 */
    if('u' == key)
      mag = 1.0 - l->pos[3];
    else if('j' == key)
      mag = l->pos[3];

  }else if(!strcmp(adjustment, "attenuation")){
    /* when adjusting attenuation, use small increments */
    if('r'==key || 'f'==key)mag = 0.004;
    if('t'==key || 'g'==key)mag = 0.002;
    if('y'==key || 'h'==key)mag = 0.001;
  }
  float amount[4]={0,0,0,0};
  switch(key){
  case 'r':amount[0]=mag;break;case 'f':amount[0]=-mag;break;
  case 't':amount[1]=mag;break;case 'g':amount[1]=-mag;break;
  case 'y':amount[2]=mag;break;case 'h':amount[2]=-mag;break;
  case 'u':amount[3]=mag;break;case 'j':amount[3]=-mag;break;
  default:assert(0);
  }
  l->adjust(adjustment, amount);
}

void lighting(){
  lightTable->gl();

  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_LIGHTING);
}

bool key_light(unsigned char key,int x, int y){
  /* start of adjustable lights */
  switch(key){
  case 'L':
    toggleLight();
    break;
  case 'K':
    {
      lightData *l = new lightData((int)lightTable->num());
      lightTable->add(l);
      printf("Added light %d:\n", lightTable->num()-1);
      l->print(0);
    }
    break;
  case 'l':
  case 'k':
    if('l' == key)
      lightTable->cycleLight();
    else if('k' == key)
      lightData::cycleAdjustment();
    printf("Current light: %d (of %d) is %s, use l to cycle it\n",
	   lightTable->selected(), lightTable->num(),
	   ((lightTable->getSelected()->on)?"on":"off"));
    printf("Current adjustment is %s (%d of %d), use k to cycle it\n",
	   lightData::getAdjustment(lightData::selectedAdjustment),
	   lightData::selectedAdjustment, lightData::NUM_ADJUSTMENTS);
    printf("Use rftgyhuj to apply and L to toggle the light on/off\n");
    lightTable->getSelected()->print(1);
    break;
  case 'r': case 'f':
  case 't': case 'g':
  case 'y': case 'h':
  case 'u': case 'j':
    adjustLight(key);
    break;
  default:
    return false;
  }

  glutPostRedisplay();
  return true;
}


void usage_lighting(){
  printf("\nLighting options (initially there are %d lights):\n",
	 DEFAULT_NUM_LIGHTS);
  printf("  -light          enable lighting (default)\n");
  printf("  -nolight        disable light (don't use with -mesh)\n");
  printf("  --lightON N             turns on light index N\n");
  printf("  --lightOFF N            turns off light index N\n");
  printf("  --lighting type N val   N is the light index, val is 4 floats, type is one of:\n");
  printf("                          pos - position, default is camera location\n");
  printf("                          ambient - ambient lighting\n");
  printf("                          diffuse - only valid if 4th value is 0\n");
  printf("                          specular - only valid if 4th value is 0\n");
  printf("                          attenuation - A + Bx + Cx^2, 4th is unused\n");

}

unsigned int parseArgs_lighting(unsigned int argc, char **argv,
				unsigned int i){
  /* enable or disable all lighting */
  if(!strcasecmp("-light",   argv[i])){useLighting=true;return 1;}
  if(!strcasecmp("-nolight",   argv[i])){useLighting=false;return 1;}

  /* enable or disable individual lights */
  if(!strcasecmp("--lightON", argv[i])){
    int which = atoi(argv[++i]);
    lightTable->expandTo(1+which);
    lightTable->get(which)->on = 1;
    lightTable->get(which)->stillDefault = 0;
    printf("Enabled light %d\n", which);
    return 2;
  }

  if(!strcasecmp("--lightOFF", argv[i])){
    int which = atoi(argv[++i]);
    lightTable->expandTo(1+which);
    lightTable->get(which)->on = 0;
    lightTable->get(which)->stillDefault = 0;
    printf("Disabled light %d\n", which);
    return 2;
  }
  
  /* set the parameters to a specific light */
  if(!strcasecmp("--lighting", argv[i])){
    if(i+6 >= argc){
      fprintf(stderr, "Insuffient parameters to --lighting\n");
      usage_lighting();
      exit(-1);
    }

    const char *type = argv[++i];
    int which = atoi(argv[++i]);
    float val[4];
    for(unsigned int j=0;j<4;j++)val[j] = atof(argv[++i]);

    lightTable->expandTo(1+which);
    lightData *l = lightTable->get(which);
    if(!lightData::validAdjustment(type) || !l){
      fprintf(stderr, "Bad parameter to --lighting, invalid adjustment or light\n");
      usage_lighting();
      exit(-1);
    }
    
    l->set(type, val);
    return 7;
  }

  return 0;
}
