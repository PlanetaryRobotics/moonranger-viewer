/*
 * Wrapper around OpenGL lights that exposes most of the functionality
 *
 * Dominic Jonak
 */
#ifndef GL_LIGHTING_H
#define GL_LIGHTING_H

class lightData{
private:
  lightData();
public:
  static const unsigned int NUM_ADJUSTMENTS=5;
  static unsigned int selectedAdjustment;
  static void cycleAdjustment(){
    selectedAdjustment = (selectedAdjustment+1)%NUM_ADJUSTMENTS;
  }

  int glNum;
  int on;
  int stillDefault;
  float pos[4];
  float ambient[4];
  float diffuse[4];
  float specular[4];
  float attenuation[4];

  lightData(int _glNum);
  void defaults();
  void print(int isSelected)const;
  void gl() const;
  void setpos(float x, float y, float z);
  void applyLimits(const char *type);

  /* valid adjustment types are:
   * 0 "pos"
   * 1 "ambient"
   * 2 "diffuse";
   * 3 "specular";
   * 4 "attenuation";
   */
  void set(const char *type, float amount[4]);
  void adjust(const char *type, float amount[4]);

  static int validAdjustment(const char *type);
  static const char *getAdjustment(int type);
};

class lights{
 private:
  unsigned int numLights;
  lightData **data;
 public:
  static unsigned int selectedLight;
  lights();
  ~lights();
  unsigned int num()const{return numLights;}
  unsigned int selected()const{return selectedLight;}

  /* ensures there are atleast n lights
   * if any lights are added, they will be off
   */
  void expandTo(int n);

  void print()const;
  void gl() const;
  void add(lightData *l);
  lightData *get(int n)const;
  lightData *getSelected()const;
  void cycleLight()const;
};

#endif
