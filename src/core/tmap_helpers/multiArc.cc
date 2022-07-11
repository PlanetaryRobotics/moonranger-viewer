#include <assert.h>
#include <string.h>
#include <multiArc.h>
#include <util.h>
#include <rasm_common_types.h>


/* a bonus given to forward children of reverse arcs
 * this modifies the effective length
 */
const float TMAP::multiPath::adjustment_stopBackingUp = 1.0;

/* a penalty given to curving branches
 * this modifies the effective length
 */
const float TMAP::multiPath::adjustment_curvedBranches = 1.0;

/* after applying adjustments,
 * ensure each arc is this long
 */
const float TMAP::multiPath::minAdjustedPathLength = 1.0;

/* when adding a child arc,
 * make it sparser by increasing the spacing/resolution this much
 */
const float TMAP::multiPath::adjustment_childResolutionFactor = 4.0;

static const float EPSILON = 0.001;


TMAP::multiPath::~multiPath(){
  if(initialPath)delete initialPath;
  while(head){
    struct pathList *del = head;
    head = head->next;
    delete del->child;
    delete del;
  }
}

void TMAP::multiPath::createMultiPath(PathModel * path_model, unsigned int path_index, float pathLength,
			       float pathResolution, bool forward, float max_heading_change, float headingResolution){

  assert(!initialPath);

  //TODO rename rad & radius - or remove
  rad = path_model->m_paths[path_index];
  len = pathLength;
  res = pathResolution;
  forw = forward;
  initialPath = new tmap_path();
  assert(initialPath);

  //TODO A better way to cast to correct implementation of PointPathInterface
  /*
   * Creates a tmap_path object (aka a tmap_arc)
   */
  initialPath->createPath(path_model, path_index, len, res, forward, max_heading_change, headingResolution);

  bestChild=NULL;
  costOfPath = 0;
  costAfterPath = 0;
}

void TMAP::multiPath::addBranch(PathModel * path_model, unsigned int path_index, bool forward, float max_heading_change, float headingResolution){
  assert(initialPath);

  float radius = path_model->m_paths[path_index];
  struct pathList *add = new pathList();

  add->child = new multiPath();
  float length = len;
  //TODO Push these into deployment code
  if(!(initialPath->isForward()) && forward)/* give a bonus to forward children of reverse arcs */
    length+=adjustment_stopBackingUp;
  else if(fabs(radius-rad)>EPSILON)/* penalize curving branches */
    length-=adjustment_curvedBranches;
  if(length<minAdjustedPathLength)length = minAdjustedPathLength;

  add->child->createMultiPath(path_model, path_index, length,
			res*adjustment_childResolutionFactor, forward, max_heading_change, headingResolution);

  /* move the child appropriately */
  RASM::point3d p(0,0,0);
  float yaw=0;
  initialPath->getFinalPose(p, yaw);
  add->child->translateAndRotate(p, 0,0,yaw);

  /* insert into the list of children */
  add->next = head;
  head = add;
}

void TMAP::multiPath::addBranch(PathModel * path_model, unsigned int path_index, bool forward,
			       const float *radiusList,
			       unsigned int radiusListLen,
			       float max_heading_change, float headingResolution){
  if(radiusListLen<=0){
    addBranch(path_model, path_index, forward, max_heading_change, headingResolution);
    return;
  }

  /* recurse on all children that match the first radius in the list */
  for(struct pathList *ptr = head;ptr;ptr = ptr->next){
    if(fabs(radiusList[0])<EPSILON || 
       fabs(radiusList[0] - ptr->child->rad)<EPSILON)
      ptr->child->addBranch(path_model, path_index, forward, &radiusList[1], radiusListLen-1, max_heading_change, headingResolution);
  }
}

void TMAP::multiPath::evaluateNear(tmap_astar &model, obstacleMAP &obstacles, 
				  bool extraWide, FILE *savedEvaluation){
  printf("[%s:%d %f] calling multiPath::evaluateNear()\n",
	 __FILE__, __LINE__, now());

  /* evaluate the initial arc */
  costOfPath = initialPath->evaluateNear(model, obstacles, extraWide,
				       savedEvaluation);

  if(savedEvaluation)
    fprintf(savedEvaluation, "[multiArc] Near evaluation of %f, inital arc: %f\n", 
	    rad, costOfPath);

  /* all done, no children to consider or this arc failed */
  if(NULL == head || costOfPath<0)return;

  /* evaluate child arcs */
  int numPaths=0;
  for(struct pathList *ptr = head;ptr;ptr = ptr->next){
    numPaths++;
    if(savedEvaluation){
      fprintf(savedEvaluation, "Near evaluating child %s arc %d, radius %f\n", 
	      ptr->child->forw?"":"reverse",
	      numPaths, ptr->child->rad);
    }

    ptr->child->evaluateNear(model, obstacles, extraWide, savedEvaluation);

    if(savedEvaluation){
      fprintf(savedEvaluation, "Near evaluation of %f->%f: %f\n",
	      rad, ptr->child->rad, ptr->child->costOfPath);
    }
  }

}

void TMAP::multiPath::evaluateFar(tmap_astar &model,
				 const obstacleMAP &obstacles, 
				 bool extraWide,
				 FILE *savedEvaluation,
				 bool savePaths){
  /* child arc */
  if(NULL==head){
    costAfterPath = initialPath->evaluateFar(model, obstacles, extraWide,
					   savedEvaluation);
    return;
  }

  /* evaluate child arcs */
#if DEBUG
  int numArcs=0;
#endif
  for(struct pathList *ptr = head;ptr;ptr = ptr->next){
#if DEBUG
    numArcs++;
    printf("Far evaluating child %s arc %d, radius %f\n", 
	   ptr->child->forw?"":"reverse",
	   numArcs, ptr->child->rad);
#endif

    FILE *f = NULL;
    if(savePaths){
      char buf[64];
      sprintf(buf, "path_%s_%d_%d.paths",
	      (forw?"forward":"reverse"),
	      (int)rad, (int)(ptr->child->rad));
      f = fopen(buf, "w");
    }

    ptr->child->evaluateFar(model, obstacles, extraWide, f, savePaths);
    if(f)fclose(f);
#if DEBUG
    printf("Far evaluation of %f->%f: %f and %f => %f\n",
	   rad, ptr->child->rad, ptr->child->costAfterPath);
#endif
  }
}


float TMAP::multiPath::getEvaluation(float &_costOfArc, float &_costAfterArc){
  /* child arc */
  if(NULL == head){
    _costOfArc = costOfPath;
    _costAfterArc = costAfterPath;
    float ret = -1.0;
    if(!(costOfPath<0.0 || costAfterPath<0.0))
      ret=(costAfterPath + arcToPathFactor*costOfPath);
#if DEBUG
    printf("Evaluation of child arc %f: %f and %f => %f\n", rad,
	   costOfPath, costAfterPath, ret);
#endif
    return ret;
  }

  float bestEvaluation, bestCostOfArc, bestCostAfterArc;
  /* pick best child arc */
  if(NULL == bestChild){
    bestChild = head->child;
    bestEvaluation = head->child->getEvaluation(bestCostOfArc,
						bestCostAfterArc);
#if DEBUG
      printf("Evaluation of %f->%f: %f and %f => %f\n",
	     rad, head->child->rad,
	     bestCostOfArc, bestCostAfterArc, bestEvaluation);
#endif
#if DEBUG
    int numArcs=0;
#endif
    for(struct pathList *ptr = head->next;ptr;ptr = ptr->next){
#if DEBUG
      numArcs++;
      printf("Evaluating child %s arc %d, radius %f\n", 
	     ptr->child->forw?"":"reverse",
	     numArcs, ptr->child->rad);
#endif
      float thisEvaluation = ptr->child->getEvaluation(bestCostOfArc,
						       bestCostAfterArc);
      if((bestEvaluation<0)                                       || 
	 ((thisEvaluation>=0) && (thisEvaluation<bestEvaluation)) ){
	bestEvaluation = thisEvaluation;
	bestChild = ptr->child;
      }
#if DEBUG
      printf("Evaluation of %f->%f: %f and %f => %f\n",
	     rad, ptr->child->rad,
	     bestCostOfArc, bestCostAfterArc, thisEvaluation);
#endif
    }
  }

  bestEvaluation = bestChild->getEvaluation(bestCostOfArc, bestCostAfterArc);

#if DEBUG
  printf("Best child: %f->%f: %f and %f => %f\n",
	 rad, bestChild->rad,
	 bestCostOfArc, bestCostAfterArc, bestEvaluation);
#endif

  /* combine the best child with the parent,
   * just add the costs of the arc together
   */
  _costAfterArc = bestCostAfterArc;
  _costOfArc = costOfPath + bestCostOfArc;
  float ret = -1.0;
  if(!((costOfPath<0.0) || (bestEvaluation<0.0)))
    ret = _costAfterArc + arcToPathFactor*_costOfArc;
#if DEBUG
  printf("Evaluated %f, %f and %f => %f\n", rad,
	 _costOfArc, _costAfterArc, ret);
#endif
  return ret;
}

void TMAP::multiPath::appendToTmap(tmap_base &tmap)const{
  const unsigned int numPriorPoints=tmap.numPoints();

  for(unsigned int i=0;i<initialPath->numPoints();i++)
    tmap.addPoint(initialPath->getPoint(i));

  const int n[3]={-1,-1,-1};
  for(unsigned int i=0;i<initialPath->numPoints() - 2;i++){
    const unsigned int tri[3] = {numPriorPoints+i+0,
				 numPriorPoints+i+1,
				 numPriorPoints+i+2};
    tmap.addTriangle(tri, n);
  }

  for(struct pathList *ptr = head;ptr;ptr = ptr->next)
    ptr->child->appendToTmap(tmap);
}

void TMAP::multiPath::writeToFile(const char *name)const{
  FILE *f = NULL;

  if(name){
    f = fopen(name, "w");
    if(!f){
      printf("Error, unable to open %s\n", name);
      exit(0);
    }
  }else{
    char filename[1024];
    sprintf(filename, "arc_%0.1f.smf", rad);
    f = fopen(filename, "w");
    if(!f){
      printf("Error, unable to open %s\n", filename);
      exit(0);
    }
  }
  unsigned int n=0;
  writeToFile(f, n);
  fclose(f);
}

void TMAP::multiPath::writeToFile(FILE *f, unsigned int &numPriorPoints) const{
  for(unsigned int i=0;i<initialPath->numPoints();i++){
    RASM::point3d p = initialPath->getPoint(i);
    fprintf(f, "v %f %f %f\n",
	    RASM_TO_METERS(p.X()),
	    RASM_TO_METERS(p.Y()),
	    RASM_TO_METERS(p.Z()));
  }

  for(unsigned int i=0;i<initialPath->numPoints() - 2;i++)
    fprintf(f, "f %d %d %d\n", 
	    numPriorPoints+i+1,
	    numPriorPoints+i+2,
	    numPriorPoints+i+3);

  numPriorPoints += initialPath->numPoints();
  for(struct pathList *ptr = head;ptr;ptr = ptr->next)
    ptr->child->writeToFile(f, numPriorPoints);
}

void TMAP::multiPath::writeToFileTrack(const char *prefix) const{
  if(!prefix)prefix="arc";
  for(struct pathList *ptr = head;ptr;ptr = ptr->next){
    char buf[64];
    sprintf(buf, "%s_%s_%d_%d.smf", prefix,
	    (forw?"forward":"reverse"),
	    (int)rad, (int)(ptr->child->rad));
    FILE *f = fopen(buf, "w");

    for(unsigned int i=0;i<initialPath->numPoints();i++){
      RASM::point3d p = initialPath->getPoint(i);
      fprintf(f, "v %f %f %f\n",
	      RASM_TO_METERS(p.X()),
	      RASM_TO_METERS(p.Y()),
	      RASM_TO_METERS(p.Z()));
    }

    for(unsigned int i=0;i<initialPath->numPoints() - 2;i++)
      fprintf(f, "f %d %d %d\n", i+1, i+2, i+3);

    unsigned int n = initialPath->numPoints();
    ptr->child->writeToFile(f, n);
    fclose(f);
  }
}

void TMAP::multiPath::translate(const RASM::point3d &p){
  initialPath->translate(p);
  for(struct pathList *ptr = head;ptr;ptr = ptr->next)
    ptr->child->translate(p);
}

void TMAP::multiPath::translateAndRotate(const RASM::point3d &p,
					double roll, double pitch, double yaw){
	assert(initialPath!=NULL);
  initialPath->translateAndRotate(p, roll, pitch, yaw);
  for(struct pathList *ptr = head;ptr;ptr = ptr->next)
    ptr->child->translateAndRotate(p, roll, pitch, yaw);
}

/* returns 1 if this multi arc intersects a circle or ray
 * either parent or all children must intersect
 */
bool TMAP::multiPath::intersectCircle(const RASM::point2d &p,
				     double radius)const{
  assert(initialPath);

  /* check if parent touches circle */
  if(initialPath->intersectCircle(p, radius)){
    if(verboseIntersection){
      printf("multiArc::intersectCircle  %s arc intersects\n",
	     (head?"parent":"leaf"));
    }
    return true;
  }
  if(verboseIntersection){
    printf("multiArc::intersectCircle  %s arc does not intersect\n",
	   (head?"parent":"leaf"));
  }

  /* no child arcs, no intersection possible */
  if(!head){
    if(verboseIntersection){
      printf("multiArc::intersectCircle  no child arcs, does not intersect\n");
    }
    return false;
  }

  /* check if any child arc avoids the circle */
  int i=0;
  for(struct pathList *ptr = head;ptr;ptr=ptr->next){
    if(!ptr->child->intersectCircle(p, radius)){
      if(verboseIntersection){
	printf("multiArc::intersectCircle  child %d does not intersect\n", i);
      }
      return false;
    }
    if(verboseIntersection){
      printf("multiArc::intersectCircle  child %d does intersect\n", i);
    }
    i++;
  }

  if(verboseIntersection){
    printf("multiArc::intersectCircle  all %d children intersect\n", i);
  }

  /* all child arcs intersect the circle */
  return true;
}

bool TMAP::multiPath::intersectRay(const RASM::point2d &p0, double r0,
				  const RASM::point2d &p1, double r1)const{

  assert(initialPath);

  /* check if parent touches ray */
  if(initialPath->intersectRay(p0, r0, p1, r1)){
    if(verboseIntersection){
      printf("multiArc::intersectRay  %s arc intersects\n",
	     (head?"parent":"leaf"));
    }
    return true;
  }
  if(verboseIntersection){
    printf("multiArc::intersectRay  %s arc does not intersect\n",
	   (head?"parent":"leaf"));
  }

  /* no child arcs, no intersection possible */
  if(!head){
    if(verboseIntersection){
      printf("multiArc::intersectRay  no child arcs, does not intersect\n");
    }
    return false;
  }

  /* check if any child arc avoids the ray */
  int i=0;
  for(struct pathList *ptr = head;ptr;ptr=ptr->next){
    if(!ptr->child->intersectRay(p0, r0, p1, r1)){
      if(verboseIntersection){
	printf("multiArc::intersectRay  child %d does not intersect\n", i);
      }
      return false;
    }
    if(verboseIntersection){
      printf("multiArc::intersectRay  child %d does intersect\n", i);
    }
    i++;
  }

  if(verboseIntersection){
    printf("multiArc::intersectRay  all %d children intersect\n", i);
  }

  /* all child arcs intersect the ray */
  return true;
}
