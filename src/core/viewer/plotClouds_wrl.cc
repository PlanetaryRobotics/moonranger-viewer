#include "plotClouds_wrl.h"
#include "plotClouds_globals.h"

#include <strings.h>

/* a = b*c */
static void mmult(float a[3][3], float b[3][3], float c[3][3]){
  for(int row=0;row<3;row++){
    for(int col=0;col<3;col++){
      a[row][col] = 0.0;
      for(int k=0;k<3;k++)
	a[row][col] += b[row][k]*c[k][col];
    }
  }
}

static void buildRotation(const wrl::tuple3f &rotation, float rpy[3][3]){
  float sr = sin(rotation.y*M_PI/180.0);
  float cr = cos(rotation.y*M_PI/180.0);
  float sp = sin(rotation.x*M_PI/180.0);
  float cp = cos(rotation.x*M_PI/180.0);
  float sy = sin(rotation.z*M_PI/180.0);
  float cy = cos(rotation.z*M_PI/180.0);

  float roll[3][3];
  float pitch[3][3];
  float yaw[3][3];

  roll[0][0] =  cr; roll[0][1] = 0.0; roll[0][2] =  sr;
  roll[1][0] = 0.0; roll[1][1] = 1.0; roll[1][2] = 0.0;
  roll[2][0] = -sr; roll[2][1] = 0.0; roll[2][2] =  cr;

  pitch[0][0] = 1.0; pitch[0][1] = 0.0; pitch[0][2] = 0.0;
  pitch[1][0] = 0.0; pitch[1][1] =  cp; pitch[1][2] = -sp;
  pitch[2][0] = 0.0; pitch[2][1] =  sp; pitch[2][2] =  cp;

  yaw[0][0] =  cy; yaw[0][1] = -sy; yaw[0][2] = 0.0;
  yaw[1][0] =  sy; yaw[1][1] =  cy; yaw[1][2] = 0.0;
  yaw[2][0] = 0.0; yaw[2][1] = 0.0; yaw[2][2] = 1.0;

  float rp[3][3];
  mmult(rp, pitch, roll);
  mmult(rpy, yaw, rp);

}


void wrl_applyTransform(const wrl::tuple3f &rotation,
			const wrl::tuple3f &translation,
			wrl *w, points *p, triangles *t){

  float x = translation.x, y = translation.y, z = translation.z;
  float rpy[3][3];
  buildRotation(rotation, rpy);

  w->applyTransform(rpy, x, y, z);

  if(p){
    for(unsigned int i=0;i<p->size();i++)
      p->getRef(i).applyTransform(rpy, x, y, z);
  }

  if(t){
    for(unsigned int i=0;i<t->size();i++){
      point a = t->get(i).getA(); a.applyTransform(rpy, x, y, z);
      point b = t->get(i).getB(); b.applyTransform(rpy, x, y, z);
      point c = t->get(i).getC(); c.applyTransform(rpy, x, y, z);
      t->getRef(i) = triangle(a, b, c);
    }
  }
}


void usage_wrl(){
  printf("  --wrlTrans N r p y x y z   transform wrl model N by r/p/y and x/y/z\n");
  printf("  --wrlTransR                as above, but angles in radians\n");
}

static void doWrlTrans(unsigned int argc, char **argv, unsigned int i,
		       int radians){
  if(i+7 >= argc){
    fprintf(stderr, "Insuffient parameters to --wrlTrans\n");
    usage_wrl();
    exit(-1);
  }

  int ind = atoi(argv[++i]);
  if((ind<0) || (ind>=(int)wrls.size())){
    fprintf(stderr, "Invalid index %d, only %lu wrl file(s) loaded\n", 
	    ind, wrls.size());
    exit(-1);
  }

  wrl::tuple3f rotation, translation;
  /* note the rotation order here */
  rotation.y = atof(argv[++i]);
  rotation.x = atof(argv[++i]);
  rotation.z = atof(argv[++i]);
  if(radians){/* applyTransform() needs the angles in degrees */
    rotation.x *= 180.0/M_PI;
    rotation.y *= 180.0/M_PI;
    rotation.z *= 180.0/M_PI;
  }
  translation.x = atof(argv[++i]);
  translation.y = atof(argv[++i]);
  translation.z = atof(argv[++i]);
  printf("Set wrl %d r/p/y to %g/%g/%g deg and x/y/z to %g/%g/%g m\n",
	 ind,
	 rotation.y, rotation.x, rotation.z,
	 translation.x, translation.y, translation.z);
  unsigned int dataIndex = wrls[ind]->getDataIndex();
  wrl_applyTransform(rotation, translation,
		     wrls[ind],
		     clouds[dataIndex],
		     triangleMeshes[dataIndex]);
  printf("Applied transform to index %d (and %d)\n", ind, dataIndex);
}



unsigned int parseArgs_wrl(unsigned int argc, char **argv, unsigned int i){

  if(!strcasecmp("--wrlTrans", argv[i])){
    doWrlTrans(argc, argv, i, 0);
    return 8;
  }

  if(!strcasecmp("--wrlTransR", argv[i])){
    doWrlTrans(argc, argv, i, 1);
    return 8;
  }

  return 0;
}
