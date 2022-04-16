#include "plotClouds_color.h"
#include "plotClouds_globals.h"
#include <math.h>
#include <stdlib.h>
#include <strings.h>

#include <GL/gl.h>

static double randBetween(double min, double max){
  return ((double)(rand()) / ((double)RAND_MAX)) * (max - min) + min;
}

static void hsv_to_rgb(float h /*0-360*/, float s /*0-1*/, float v /*0-1*/,
		       float &r, float &g, float &b){
  float Hi = fmod(floor(h/60.0), 6.0);
  float f = h/60.0 - Hi;
  float p = v * (1.0 - s);
  float q = v * (1.0 - f*s);
  float t = v * (1.0 - (1.0 - f) * s);
  switch((int)Hi){
  case 0: r = v; g = t; b = p; break;
  case 1: r = q; g = v; b = p; break;
  case 2: r = p; g = v; b = t; break;
  case 3: r = p; g = q; b = v; break;
  case 4: r = t; g = p; b = v; break;
  case 5: r = v; g = p; b = q; break;
  }
}

#if 1/* unused helpers */
static float rgb_to_hue(float r, float g, float b, float range){
  float delta=0;
  float offset=0;

  if(range<1.0/120.0)return 0.0;
  else if(g>=b && r>=g) {delta = (g-b); offset =   0.0;}
  else if(g< b && r>=b) {delta = (g-b); offset = 360.0;}
  else if(g>=b && g>=r) {delta = (b-r); offset = 120.0;}
  else if(b>=g && b>=r) {delta = (r-g); offset = 240.0;}
  return 60.0*delta/range + offset;
}

static float minMaxComponents(float r, float g, float b,
			      float &minComponent, float &maxComponent){
  minComponent = r;
  if((g<=r) && (g<=b))minComponent = g;
  if((b<=r) && (b<=g))minComponent = b;

  maxComponent = r;
  if((g>=r) && (g>=b))maxComponent = g;
  if((b>=r) && (b>=g))maxComponent = b;

  return maxComponent-minComponent;
}

extern void rgb_to_hsl(float r, float g, float b,
		       float &h, float &s, float &l){
  float minComponent, maxComponent;
  float range = minMaxComponents(r, g, b, minComponent, maxComponent);

  /* HSL luminosity component */
  l = (maxComponent+minComponent)/2.0;

  /* HSL saturation component */
  if(range<=0)s=0;
  else if(l > 0.5)s = range/(2.0-2.0*l);
  else s = range/(2.0*l);

  /* HSL hue component */
  h = rgb_to_hue(r, g, b, range);
}

extern void rgb_to_hsv(float r, float g, float b,
		       float &h, float &s, float &v){
  float minComponent, maxComponent;
  float range = minMaxComponents(r, g, b, minComponent, maxComponent);

  /* HSV value component */
  v = maxComponent;

  /* HSV saturation component */
  s = range/maxComponent;

  /* HSV hue component */
  h = rgb_to_hue(r, g, b, range);
}

extern void hsl_to_rgb(float h, float s, float l,
		       float &r, float &g, float &b){
  if(s<=0.0){
    r = g = b = l;
    return;
  }

  float q = l*(1.0+s);
  if(l>=0.5)q = l+s-(l*s);
  float p = 2.0*l-q;
  float Hk = h/360.0;

  float Tr = Hk + 1.0/3.0;
  float Tg = Hk;
  float Tb = Hk - 1.0/3.0;
  if(Tr<0)Tr++;
  if(Tg<0)Tg++;
  if(Tb<0)Tb++;
  if(Tr>1.0)Tr--;
  if(Tg>1.0)Tg--;
  if(Tb>1.0)Tb--;

  if(Tr<1.0/6.0)r = p + (q-p)*6.0*Tr;
  else if((1.0/6.0 <= Tr) && (Tr < 0.5))r = q;
  else if((0.5 <= Tr) && (Tr < 2.0/6.0))r = p + (q-p)*(2.0/3.0 - Tr)*6.0;
  else r = p;

  if(Tg<1.0/6.0)g = p + (q-p)*6.0*Tg;
  else if((1.0/6.0 <= Tg) && (Tg < 0.5))g = q;
  else if((0.5 <= Tg) && (Tg < 2.0/6.0))g = p + (q-p)*(2.0/3.0 - Tg)*6.0;
  else g = p;

  if(Tb<1.0/6.0)b = p + (q-p)*6.0*Tb;
  else if((1.0/6.0 <= Tb) && (Tb < 0.5))b = q;
  else if((0.5 <= Tb) && (Tb < 2.0/6.0))b = p + (q-p)*(2.0/3.0 - Tb)*6.0;
  else b = p;
}
#endif
/*** end of color helpers ***/

color::color(){
  hsv_to_rgb(randBetween(0, 360), 1, 1, r, g, b);
}

color::color(float percent){
  while(percent>=1.0)percent--;
  hsv_to_rgb(percent*360.0, 1, 1, r, g, b);
}

void color::setFromHSV(float h, float s, float v){
  hsv_to_rgb(h, s, v, r, g, b);
}

void color::glInvertColor()const{glColor3f(1.0-r, 1.0-g, 1.0-b);}
void color::glColor()const{glColor3f(r, g, b);}
void color::glColor4()const{glColor4f(r, g, b, 1.0);}
bool color::similarTo(color *c)const{
  return ((fabs(r - c->r) + fabs(g - c->g) + fabs(b - c->b)) < 0.5);
}

void createColors(int clearOld){
  if(clearOld){
    for(unsigned int i=0;i<colors.size();i++)
      delete colors[i];
    colors.clear();
  }else{
    /* create uniform colors... */
    while(colors.size() < clouds.size())
      colors.push_back(new color((float)(1+colors.size())/(float)clouds.size()));
  }

  while(colors.size() < clouds.size()){
    color *c = new color(); /* random candidate color */
    /* check if it is too close to an existing color */
    bool similar = 0;
    for(unsigned int i=0;!similar && i<colors.size();i++)
      similar = c->similarTo(colors[i]);
    if(similar)
      delete c;
    else /* if it is unique, add it to the collection */
      colors.push_back(c);
  }
}

void usage_color(){
  printf("\nColor options:\n");
  printf("Default is rainbow for datasets and goodness of evaluations\n");
  printf("The nth color you specify is applied to the nth dataset\n");
  printf("  --RGB r g b       set color of the next dataset (values 0-1) \n");
  printf("  --grey v          same as '--RGB v v v'\n");
  printf("  --HSV h s v       set color of the next dataset (values 0-1) \n");
  printf("  --hue h           same as '--HSV h 1 1'\n");
  printf("  --pathHSV h s v   set color of the next evaluation (values 0-1) \n");
}

unsigned int parseArgs_color(unsigned int argc, char **argv,
			     unsigned int i){

  /* add a color, specify r, g and b as 0-1 */
  if(!strcasecmp("--RGB", argv[i])){
    color *c = new color();
    c->r = atof(argv[++i]);
    c->g = atof(argv[++i]);
    c->b = atof(argv[++i]);
    colors.push_back(c);
    return 4;
  }

  /* add a color, specify hue, saturation and value as 0-1 */
  if(!strcasecmp("--HSV", argv[i])){
    color *c = new color();
    float h = atof(argv[++i])*360.0;
    float s = atof(argv[++i]);
    float v = atof(argv[++i]);
    c->setFromHSV(h, s, v);
    colors.push_back(c);
    return 4;
  }

  if(!strcasecmp("--pathHSV", argv[i])){
    color *c = new color();
    float h = atof(argv[++i])*360.0;
    float s = atof(argv[++i]);
    float v = atof(argv[++i]);
    c->setFromHSV(h, s, v);
    pathColors.push_back(c);
    return 4;
  }

  /* add a color, specify a hue 0-359 */
  if(!strcasecmp("--hue", argv[i])){
    colors.push_back(new color(atof(argv[++i])/360.0));
    return 2;
  }

  /* add a color, specify a greyscale value 0-1 */
  if(!strcasecmp("--grey", argv[i])){
    color *c = new color();
    float val = atof(argv[++i]);
    c->r = c->g = c->b = val;
    colors.push_back(c);
    return 2;
  }

  return 0;
}


bool key_color(unsigned char key,int x, int y){
  switch(key){
  case 'c':
    createColors();
    break;

  default:
    return false;
  }

  glutPostRedisplay();
  return true;
}
