#include <obstacleMap.h>
#include <lineHelpers.h>
#include <rasm_common_types.h>

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

static const int USAGE_COUNT=10;

TMAP::obstacleList::~obstacleList(){
  while(head){
    struct node *del = head;
    head = head->next;
    free(del);
  }
}

void TMAP::obstacleList::insert(const RASM::point3d &a, const RASM::point3d &b,
				int count){
  struct node *nn = (struct node *)malloc(sizeof(struct node));
  nn->counter = (count<=0)?USAGE_COUNT:count;
  nn->a = a;
  nn->b = b;
  nn->next = head;
  head = nn;
}

bool TMAP::obstacleList::crosses(const RASM::point3d &a, 
				 const RASM::point3d &b) const{
  for(struct node *n=head;n;n=n->next){
    if(intersects(n->a, n->b, a, b)){
      n->counter++;
      if(n->counter > USAGE_COUNT)n->counter = USAGE_COUNT;
      return true;
    }
  }
  return false;
}

void TMAP::obstacleList::writeToFile(const char *filename)const{
  FILE *f = fopen(filename, "w");
  if(!f){
    printf("Error, could not open %s\n", filename);
    exit(0);
  }

  int i=1;
  for(struct node *n=head;n;n=n->next){
    fprintf(f, "v %f %f %f\n", RASM_TO_METERS(n->a.X()), RASM_TO_METERS(n->a.Y()), RASM_TO_METERS(n->a.Z()));
    fprintf(f, "v %f %f %f\n", RASM_TO_METERS(n->b.X()), RASM_TO_METERS(n->b.Y()), RASM_TO_METERS(n->b.Z()));
    fprintf(f, "v %f %f %f\n", 0.001+RASM_TO_METERS(n->a.X()), 0.001+RASM_TO_METERS(n->a.Y()), 1.0+RASM_TO_METERS(n->a.Z()));
    fprintf(f, "v %f %f %f\n", 0.001+RASM_TO_METERS(n->b.X()), 0.001+RASM_TO_METERS(n->b.Y()), 1.0+RASM_TO_METERS(n->b.Z()));
    fprintf(f, "f %d %d %d\n", i, i+1, i+2);
    fprintf(f, "f %d %d %d\n", i+1, i+2, i+3);
    fprintf(f, "f %d %d %d\n", i+2, i+3, i);
    i+=4;
  }

  fclose(f);
}

void TMAP::obstacleList::readFromFile(const char *filename){
  FILE *f = fopen(filename, "r");
  if(!f){
    printf("Error, could not open %s\n", filename);
    exit(0);
  }

  int n=0;

  /* read groups of 4 vertices
     on every 4th, combine the first two and discard the last two
  */
  while(!feof(f)){
    float x, y, z;
    char buf[64];
    if(NULL == fgets(buf, sizeof(buf), f) ||
       3 != sscanf(buf, "v %f %f %f", &x, &y, &z))continue;
    struct node *nn = (struct node *)malloc(sizeof(struct node));
    if(!nn){
      perror("malloc failed");
      exit(-1);
    }
    nn->a.coord3d[0] = METERS_TO_RASM(x);
    nn->a.coord3d[1] = METERS_TO_RASM(y);
    nn->a.coord3d[2] = METERS_TO_RASM(z);
    nn->next = head;
    head = nn;
    if(0 == (++n)%4){
      nn->counter = 0;

      /* discard the latter two */
      struct node *del = head;
      head = head->next->next;
      free(del->next);
      free(del);

      /* combine the earlier two */
      del = head;
      head = head->next;
      head->b = del->a;
      head->counter = 0;
      free(del);
    }
  }
  if(0 != n%4){
    printf("Error, should be a multiple of 4 vertices, got %d\n", n);
    exit(-1);
  }

  fclose(f);
}

void TMAP::obstacleList::prune(){
#if DEBUG
  int ndel = 0;
  int nleft = 1;
#endif
  while(head && head->counter<=0){
    struct node *del=head;
    head=head->next;
    free(del);
#if DEBUG
    ndel++;
#endif
  }

  if(!head){
#if DEBUG
    printf("Deleted all %d obstacles\n", ndel);
#endif
    return;
  }
  head->counter--;

  for(struct node *n=head;n->next;){
    if(n->next->counter<=0){
      struct node *del=n->next;
      n->next = del->next;
      free(del);
#if DEBUG
      ndel++;
#endif
    }else{
      n->counter--;
      n = n->next;
#if DEBUG
      nleft++;
#endif
    }
  }
#if DEBUG
  printf("Deleted %d obstacles, %d remain\n", ndel, nleft);
#endif
}

TMAP::obstacleMAP::obstacleMAP():obstacles(new TMAP::obstacleList()){}

TMAP::obstacleMAP::~obstacleMAP(){
  assert(obstacles);
  delete obstacles;
  obstacles = NULL;
}

void TMAP::obstacleMAP::clearObstacles(){
  assert(obstacles);
  delete obstacles;
  obstacles = new TMAP::obstacleList();
}
