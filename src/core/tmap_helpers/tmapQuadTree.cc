#include <tmapQuadTree.h>
#include <rasm_common_types.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#define DEBUG 0

/* pointinfo2d helper class 
 * associates a 2d point with a collection of indices
 * indices may be either a vertex or triangle index 
 * depending on what the tree is used for
 */
class pointinfo2d:public RASM::point2d{
public:
  unsigned int numreferences;
  unsigned int *indices;

  pointinfo2d(const RASM::point2d &p, unsigned int N, unsigned int *I)
    :RASM::point2d(p), 
     numreferences(N), 
     indices(I){}
  pointinfo2d(const RASM::point2d &p, unsigned int ind)
    :RASM::point2d(p), numreferences(1){
    indices = (unsigned int *)malloc(sizeof(unsigned int));
    assert(indices);
    *indices = ind;
  }

  ~pointinfo2d(){
    if(numreferences>0){
      numreferences=0;
      free(indices);
      indices=NULL;
    }
  }
  void addIndex(unsigned int ind){
    indices = (unsigned int *)realloc(indices, (numreferences+1) * sizeof(unsigned int));
    assert(indices);
    indices[numreferences++]=ind;
  }
};
/* end of class pointinfo2d */


/* the tree is a collection of interior and leaf nodes, as well as a root node
 *  each node contains:
 *  - 2d bounds
 *  - parent pointer 
 *  
 * interior nodes also contain:
 *  - a dividing point
 *  - 4 child pointers (for each quadrant of the dividing point)
 *
 * leaf nodes also contain
 *  - data
 *
 * the root node also cotains
 *  - a pointer to the root leaf or interior node
 *  - a count of the number of points
 * the purpose of the root node is to initialize the bounds of the entire tree
 * and gracefully handle spacial expansion of the tree
 */
TMAP::quadNode::quadNode():parent(NULL){}
TMAP::quadNode::quadNode(quadNode_interiorNode *_parent, const RASM::bounds2d &_bounds)
  :parent(_parent), bounds(_bounds){}

TMAP::quadNode::~quadNode(){}

namespace TMAP {
  class quadNode_interiorNode : public quadNode{
  private:
    /* this helper function is used by the constructor,
     * it makes the bounds twice as big attempting to cover the point
     */
    void doubleBounds(quadNode *&child, const RASM::point2d &p);

    /* private copy constructor */
    quadNode_interiorNode(quadNode_interiorNode *copy);
  
    /* helper to select a child node, 
     * just looks at which side of the dividing point the target falls
     */
    inline int selectChild(const RASM::point2d &p) const{
      if(p.X()<dividingPoint.X()){
	if(p.Y()<dividingPoint.Y())return 0;
	return 1;
      }
      if(p.Y()<dividingPoint.Y())return 2;
      return 3;
    }

  public:
    /* the 'center' of this node
     * this isn't necessarily the average of the corners,
     * instead it is the dividing point when deciding which child to use
     */
    RASM::point2d dividingPoint;

    /* the four children */
    quadNode *child[4];

    /* this constructor takes a node and point that does not fall in the node
     * it creates a parent node that contains the old node
     * as well as three leaf nodes
     */
    quadNode_interiorNode(quadNode *child, const RASM::point2d &p); 

    /* creates an interior node with this data  */
    quadNode_interiorNode(quadNode_interiorNode *_parent,
			  const RASM::bounds2d &_bounds,
			  const RASM::point2d &_dividingPoint,
			  quadNode *children[4]);

    virtual ~quadNode_interiorNode();
    virtual void selfCheck() const;
    virtual void print(int level=0) const;
    virtual bool contains(const RASM::point2d &p)const;
    virtual void insert(const RASM::point2d &p, unsigned int ind, 
			quadNode *&newhead);
    virtual int lookup(const RASM::point2d &p, unsigned int **indices)const;
    virtual void lookup(const RASM::point2d &p, RASM::point2d &closest, 
			double &distSq, unsigned int &numIndices,
			unsigned int **indices) const;
  };/* end of quadNode_interiorNode class declaration */

  class quadNode_leafNode : public quadNode{
  public:
    /* how many points should we stuff into the leaf node of a quad tree
     * 10 is a good value; 
     * small enough so that linear searches of the leaf nodes are still fast,
     * large enough to significantly reduce the number of leaf nodes
     * 
     * In leaf nodes this array will be allocated.
     * In interior nodes, this will be NULL and numPointsInLeaf will be 0
     */
#define MAX_POINTS_IN_LEAF 5
    pointinfo2d **pointsInLeaf;
    unsigned int numPointsInLeaf;
  

    quadNode_leafNode(quadNode_interiorNode *_parent, const RASM::bounds2d &_bounds);
    quadNode_leafNode(quadNode_interiorNode *_parent, const RASM::bounds2d &_bounds,
		      const RASM::point2d &point, unsigned int ind);

    void addPointToLeaf(struct pointinfo2d *addme);
    void addPointToLeaf(const RASM::point2d &p, unsigned int ind,
			int allowDuplicates=0, int forceDuplicates=0);

    /* transforms this leaf node into an interior one
     * by creating 4 evenly spaced child nodes and pushes the data into them 
     * this node becomes one of those child nodes
     * so all references need to be updated
     */
    quadNode_interiorNode *splitLeaf();

    /* increases the bounds to fit this point,
     * returns true if anything was changed
     */
    bool expandBounds(const RASM::point2d &p);

    /* checks if this point should be added to this leaf
     * (ie if there's room)
     */
    bool shouldAddPointToLeaf(const RASM::point2d &p, int allowDuplicates=0,
			      int forceDuplicates=0) const;
    
    virtual ~quadNode_leafNode();
    virtual void selfCheck() const;
    virtual void print(int level=0) const;
    virtual bool contains(const RASM::point2d &p)const;
    virtual void insert(const RASM::point2d &p, unsigned int ind, 
			quadNode *&newhead);
    virtual int lookup(const RASM::point2d &p, unsigned int **indices)const;
    virtual void lookup(const RASM::point2d &p, RASM::point2d &closest, 
			double &distSq, unsigned int &numIndices,
			unsigned int **indices) const;
  };/* end of quadNode_leafNode class declaration */


  class quadNode_rootNode : public quadNode{
  protected:
    unsigned int numPoints;
    quadNode *root;

  public:
    quadNode_rootNode():quadNode(), numPoints(0), root(NULL){}
    quadNode_rootNode(const RASM::bounds2d &initialBounds)
      :quadNode(NULL, initialBounds), numPoints(0), root(NULL){}

    virtual ~quadNode_rootNode(){
      numPoints=0;
      assert(root);
      delete root;
      root = NULL;
    }

    virtual void selfCheck() const{if(root)root->selfCheck();}
    virtual void print(int level=0) const{
      printf("This quad tree contains %d points: \n", numPoints);
      if(root)
	root->print(level+1);
    }

    virtual bool contains(const RASM::point2d &p)const{
      if(!root)return false;
      return root->contains(p);
    }

    virtual void insert(const RASM::point2d &p, unsigned int ind, 
			quadNode *&newhead){
      if(0 == numPoints){
	if(0 == distSq2d(bounds.minPoint, bounds.maxPoint)){
	  /* were not given any initial bounds */
	  bounds.minPoint = p;
	  bounds.maxPoint = p + RASM::point2d(RASM_EPSILON, RASM_EPSILON);
	}else{
	  /* we were given some initial bounds,
	   * make sure they include the first point
	   */
	  bounds.expandTo(p);
	}
	assert(!root);
	root = new quadNode_leafNode(NULL, bounds, p, ind);

      }else if(numPoints < MAX_POINTS_IN_LEAF){
	assert(root);
	assert(((quadNode_leafNode *)root)->shouldAddPointToLeaf(p, ind));
	((quadNode_leafNode *)root)->expandBounds(p);
	assert(root->covers(p));
	((quadNode_leafNode *)root)->addPointToLeaf(p, ind, 1);
	newhead = NULL;

      }else{
	assert(root);

	if(MAX_POINTS_IN_LEAF == numPoints){
	  quadNode_leafNode *oldroot = (quadNode_leafNode *)root;
	  root = oldroot->splitLeaf();
	  assert(root->covers(p) == oldroot->covers(p));
	  delete oldroot;
	}

	while(!root->covers(p))
	  root = new quadNode_interiorNode(root, p);
	root->insert(p, ind, newhead);
	assert(NULL == newhead);	
      }
      
      numPoints++;
    }

    void inline insert(const RASM::point2d &p, unsigned int ind){
      quadNode *dummy;
      insert(p, ind, dummy);
    }
    virtual int lookup(const RASM::point2d &p, unsigned int **indices)const{
      if(!root)return -1;
      return root->lookup(p, indices);
    }
    virtual void lookup(const RASM::point2d &p, RASM::point2d &closest,
			double &distSq, unsigned int &numIndices,
			unsigned int **indices) const{
      distSq = -1;
      numIndices = 0;
      *indices = NULL;
      if(numPoints>0){
	assert(root);
	root->lookup(p, closest, distSq, numIndices, indices);
      }
    }

    virtual bool covers(const RASM::point2d &p)const { 
      if(numPoints<=0)return 0;
      return root->covers(p);
    }
  };/* end of quadNode_rootNode class declaration */

  /* end of declaration of quadNode classes: interior, leaf and root */
}/* end of namespace TMAP */


/* start of selfCheck() definition */  
void TMAP::quadNode_interiorNode::selfCheck() const{
  for(int i=0;i<4;i++)
    assert(child[i]);

  assert(RASM_UNITS_equals(child[0]->bounds.minPoint.X() , bounds.minPoint.X()));
  assert(RASM_UNITS_equals(child[0]->bounds.minPoint.Y() , bounds.minPoint.Y()));
  assert(RASM_UNITS_equals(child[0]->bounds.maxPoint.X() , dividingPoint.X()));
  assert(RASM_UNITS_equals(child[0]->bounds.maxPoint.Y() , dividingPoint.Y()));

  assert(RASM_UNITS_equals(child[1]->bounds.minPoint.X() , bounds.minPoint.X()));
  assert(RASM_UNITS_equals(child[1]->bounds.minPoint.Y() , dividingPoint.Y()));
  assert(RASM_UNITS_equals(child[1]->bounds.maxPoint.X() , dividingPoint.X()));
  assert(RASM_UNITS_equals(child[1]->bounds.maxPoint.Y() , bounds.maxPoint.Y()));

  assert(RASM_UNITS_equals(child[2]->bounds.minPoint.X() , dividingPoint.X()));
  assert(RASM_UNITS_equals(child[2]->bounds.minPoint.Y() , bounds.minPoint.Y()));
  assert(RASM_UNITS_equals(child[2]->bounds.maxPoint.X() , bounds.maxPoint.X()));
  assert(RASM_UNITS_equals(child[2]->bounds.maxPoint.Y() , dividingPoint.Y()));

  assert(RASM_UNITS_equals(child[3]->bounds.minPoint.X() , dividingPoint.X()));
  assert(RASM_UNITS_equals(child[3]->bounds.minPoint.Y() , dividingPoint.Y()));
  assert(RASM_UNITS_equals(child[3]->bounds.maxPoint.X() , bounds.maxPoint.X()));
  assert(RASM_UNITS_equals(child[3]->bounds.maxPoint.Y() , bounds.maxPoint.Y()));
}
  
void TMAP::quadNode_leafNode::selfCheck() const{
  assert(numPointsInLeaf>=0 && numPointsInLeaf<=MAX_POINTS_IN_LEAF);

  /* check all the references */
  assert(pointsInLeaf);
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    assert(pointsInLeaf[i]);
    assert(pointsInLeaf[i]->numreferences>0 && pointsInLeaf[i]->indices!=NULL);
  }
}
/* end of selfCheck() definition */  

/* start of print() definition */
void TMAP::quadNode::print(int level) const{
  for(int i=0;i<level;i++)printf(" ");
  printf("Level %d node ", level);
  printf(" at addres 0x%016lx", (unsigned long int)this);

  printf(" with bounds ");
  bounds.minPoint.print();
  printf(" and ");
  bounds.maxPoint.print();
  printf("\n");
}

void TMAP::quadNode_interiorNode::print(int level) const{
  TMAP::quadNode::print(level);

  for(int i=0;i<level;i++)printf(" ");
  printf("Interior node centered at ");
  dividingPoint.print();
  printf("\n");

  if(level<0)return;
  for(int i=0;i<4;i++){
    assert(child[i]);
    child[i]->print(level+1);
  }
}

void TMAP::quadNode_leafNode::print(int level) const{
  TMAP::quadNode::print(level);

  for(unsigned int p=0;p<numPointsInLeaf;p++){
    for(int i=0;i<level;i++)printf(" ");
    printf("Point %d ", p);
    pointsInLeaf[p]->print();
    printf(" has %d references:", pointsInLeaf[p]->numreferences);
    for(unsigned int i=0;i<pointsInLeaf[p]->numreferences;i++)
      printf(" %d", pointsInLeaf[p]->indices[i]);
    printf("\n");
  }
}
/* end of print() definition */

/* start of contains() definition */
bool TMAP::quadNode_interiorNode::contains(const RASM::point2d &p) const{
  return child[selectChild(p)]->contains(p);
}
bool TMAP::quadNode_leafNode::contains(const RASM::point2d &p) const{
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    if(*pointsInLeaf[i] == p)
      return true;
  }
  return false;
}
/* end of contains() definition */


/* start of constructor definition */
TMAP::quadNode_leafNode::quadNode_leafNode(TMAP::quadNode_interiorNode *_parent, 
					   const RASM::bounds2d &_bounds,
					   const RASM::point2d &point,
					   unsigned int ind)
  :quadNode(_parent, _bounds),
   pointsInLeaf(new pointinfo2d*[MAX_POINTS_IN_LEAF]), 
   numPointsInLeaf(0){
  pointsInLeaf[numPointsInLeaf++] = new pointinfo2d(point, ind);
}

TMAP::quadNode_leafNode::quadNode_leafNode(TMAP::quadNode_interiorNode *_parent,
					   const RASM::bounds2d &_bounds)
  :quadNode(_parent, _bounds),
   pointsInLeaf(new pointinfo2d*[MAX_POINTS_IN_LEAF]), numPointsInLeaf(0){}
/* end of constructor definition */

/* start of destructor definition */
TMAP::quadNode_leafNode::~quadNode_leafNode(){
  for(unsigned int i=0;i<numPointsInLeaf;i++)delete pointsInLeaf[i];
  delete[] pointsInLeaf;
  pointsInLeaf = NULL;
}
/* end of destructor definition */

/* this helper function is used by the constructor,
   it makes the bounds twice as big attempting to cover the point
*/
void TMAP::quadNode_interiorNode::doubleBounds(TMAP::quadNode *&childNode,
					       const RASM::point2d &p){
  RASM_UNITS dx = bounds.maxPoint.X() - bounds.minPoint.X();
  RASM_UNITS dy = bounds.maxPoint.Y() - bounds.minPoint.Y();

  if(p.X()>=bounds.maxPoint.X()){
    /* fill in the X component */
    dividingPoint.coord2d[0] = bounds.maxPoint.X();
    bounds.maxPoint.coord2d[0] += dx;

    if(p.Y()>=bounds.maxPoint.Y()){/* PP */
      dividingPoint.coord2d[1] = bounds.maxPoint.Y();
      bounds.maxPoint.coord2d[1] += dy;
      child[0] = childNode;

    }else{/* PN */
      dividingPoint.coord2d[1] = bounds.minPoint.Y();
      bounds.minPoint.coord2d[1] -= dy;
      child[1] = childNode;
    }

  }else{
    /* fill in the X component */
    dividingPoint.coord2d[0] = bounds.minPoint.X();
    bounds.minPoint.coord2d[0] -= dx;

    if(p.Y()>=bounds.maxPoint.Y()){/* NP */
      dividingPoint.coord2d[1] = bounds.maxPoint.Y();
      bounds.maxPoint.coord2d[1] += dy;
      child[2] = childNode;
    }else{/* NN */
      dividingPoint.coord2d[1] = bounds.minPoint.Y();
      bounds.minPoint.coord2d[1] -= dy;
      child[3] = childNode;
    }
  }

  /* fill in all the other children with empty leaf nodes */
  if(!child[0])child[0] = new TMAP::quadNode_leafNode(this, RASM::bounds2d(RASM::point2d(bounds.minPoint.X(), bounds.minPoint.Y()),
									   RASM::point2d(dividingPoint.X(),   dividingPoint.Y())));
  if(!child[1])child[1] = new TMAP::quadNode_leafNode(this, RASM::bounds2d(RASM::point2d(bounds.minPoint.X(), dividingPoint.Y()),
									   RASM::point2d(dividingPoint.X(),   bounds.maxPoint.Y())));
  if(!child[2])child[2] = new TMAP::quadNode_leafNode(this, RASM::bounds2d(RASM::point2d(dividingPoint.X(),   bounds.minPoint.Y()),
									   RASM::point2d(bounds.maxPoint.X(), dividingPoint.Y())));
  if(!child[3])child[3] = new TMAP::quadNode_leafNode(this, RASM::bounds2d(RASM::point2d(dividingPoint.X(),   dividingPoint.Y()),
									   RASM::point2d(bounds.maxPoint.X(), bounds.maxPoint.Y())));
  childNode->parent = this;
}

/* private copy constructor */
TMAP::quadNode_interiorNode::quadNode_interiorNode(TMAP::quadNode_interiorNode *_parent)
  :quadNode(_parent, _parent->bounds),
   dividingPoint(_parent->dividingPoint){
  for(int i=0;i<4;i++)child[i] = _parent->child[i];
}

/* this constructor takes a node and point that does not fall in the node
 * it creates a parent node that contains the old node
 * as well as three leaf nodes
 */
TMAP::quadNode_interiorNode::quadNode_interiorNode(TMAP::quadNode *childNode, 
						   const RASM::point2d &p)
  :quadNode(NULL, childNode->bounds){
  for(int i=0;i<4;i++)child[i]=NULL;

  assert(!covers(p));
  doubleBounds(childNode, p);
}

/* creates an interior node with this data  */
TMAP::quadNode_interiorNode::quadNode_interiorNode(TMAP::quadNode_interiorNode *_parent,
						   const RASM::bounds2d &_bounds,
						   const RASM::point2d &_dividingPoint,
						   TMAP::quadNode *children[4])
  :quadNode(_parent, _bounds),
   dividingPoint(_dividingPoint){
  for(int i=0;i<4;i++){
    assert(children[i]);
    child[i]=children[i];
    child[i]->parent = this;
  }
}

TMAP::quadNode_interiorNode::~quadNode_interiorNode(){
  for(int i=0;i<4;i++)delete child[i];
}


void TMAP::quadNode_leafNode::addPointToLeaf(struct pointinfo2d *addme){
  assert(numPointsInLeaf < MAX_POINTS_IN_LEAF);
  assert(covers(*addme));
  pointsInLeaf[numPointsInLeaf]=addme;
  numPointsInLeaf++;
}

/* transforms this leaf node into an interior one...
 * creates 4 leaf nodes and pushes the data into them 
 * then creates an interior node with those leaves
 * the resulting interior node is returned and should replace the current one
 */
TMAP::quadNode_interiorNode *TMAP::quadNode_leafNode::splitLeaf(){

  RASM::point2d dividingPoint = bounds.minPoint + bounds.maxPoint;
  dividingPoint /= (RASM_UNITS)2;

  /* create child nodes */
  TMAP::quadNode_leafNode *children[4];
  children[0] = new TMAP::quadNode_leafNode(NULL, RASM::bounds2d(RASM::point2d(bounds.minPoint.X(), bounds.minPoint.Y()),
								 RASM::point2d(dividingPoint.X(),   dividingPoint.Y())));
  children[1] = new TMAP::quadNode_leafNode(NULL, RASM::bounds2d(RASM::point2d(bounds.minPoint.X(), dividingPoint.Y()),
								 RASM::point2d(dividingPoint.X(),   bounds.maxPoint.Y())));
  children[2] = new TMAP::quadNode_leafNode(NULL, RASM::bounds2d(RASM::point2d(dividingPoint.X(),   bounds.minPoint.Y()),
								 RASM::point2d(bounds.maxPoint.X(), dividingPoint.Y())));
  children[3] = new TMAP::quadNode_leafNode(NULL, RASM::bounds2d(RASM::point2d(dividingPoint.X(),   dividingPoint.Y()),
								 RASM::point2d(bounds.maxPoint.X(), bounds.maxPoint.Y())));
  for(int i=0;i<4;i++)
    assert(children[i]);
    
  /* move our data to the right child node */
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    int used=0;
    for(int j=0;j<4;j++){
      if(children[j]->covers(*pointsInLeaf[i])){
	 assert(!used);
	 children[j]->addPointToLeaf(pointsInLeaf[i]);
	 used = 1;
      }
    }
    assert(used);
  }
	   
  /* clear out our own data since we're not a leaf node any more */
  numPointsInLeaf=0;
  delete []pointsInLeaf;
  pointsInLeaf=NULL;

  /* return an interior node that contains all of our data */
  TMAP::quadNode_leafNode **c = children;/* avoid warning in gcc 4.1.1 */
  return new TMAP::quadNode_interiorNode(parent, bounds, dividingPoint,
					 (TMAP::quadNode **)c);
}

bool TMAP::quadNode_leafNode::expandBounds(const RASM::point2d &p){
  bool ret=false;
  if(p.X()+RASM_EPSILON > bounds.maxPoint.X()){
    bounds.maxPoint.coord2d[0] = p.X()+RASM_EPSILON;
    ret=true;
  }
  if(p.X() < bounds.minPoint.X()){
    bounds.minPoint.coord2d[0] = p.X();
    ret=true;
  }
  if(p.Y()+RASM_EPSILON > bounds.maxPoint.Y()){
    bounds.maxPoint.coord2d[1] = p.Y()+RASM_EPSILON;
    ret=true;
  }
  if(p.Y() < bounds.minPoint.Y()){
    bounds.minPoint.coord2d[1] = p.Y();
    ret=true;
  }
  return ret;
}

void TMAP::quadNode_leafNode::addPointToLeaf(const RASM::point2d &p,
					     unsigned int ind, 
					     int allowDuplicates,
					     int forceDuplicates){

  assert(covers(p));
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    if(pointsInLeaf[i]->equals(p)){
      assert(allowDuplicates);
      /* append 'ind' to the list of indices */
#if 0
      printf("Re-adding point %f,%f (index %d)\n",
	     (float)p.coord2d[0], (float)p.coord2d[1], ind);
      printf("Old point %f,%f (index %d)\n",
	     (float)pointsInLeaf[i]->coord2d[0], (float)pointsInLeaf[i]->coord2d[1], pointsInLeaf[i]->indices[0]);
      assert(0);
#endif
      pointsInLeaf[i]->addIndex(ind);
      return;
    }
  }

  assert(!forceDuplicates);
  addPointToLeaf(new pointinfo2d(p, ind));
}
  
bool TMAP::quadNode_leafNode::shouldAddPointToLeaf(const RASM::point2d &p, 
						   int allowDuplicates,
						   int forceDuplicates) const{
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    if(pointsInLeaf[i]->equals(p)){
      assert(allowDuplicates);
      return true;
    }
  }
  assert(!forceDuplicates);
  return (numPointsInLeaf<MAX_POINTS_IN_LEAF);
}

void TMAP::quadNode_interiorNode::insert(const RASM::point2d &p,
					 unsigned int ind,
					 TMAP::quadNode *&newhead){
  assert(covers(p));

  TMAP::quadNode *childNode = child[selectChild(p)];
  assert(childNode);
  childNode->insert(p, ind, newhead);

  if(newhead){
    /* we split the child, update our child pointer... */
    int used = 0;
    for(int i=0;i<4;i++){
      if(child[i]->covers(p)){
	delete child[i];
	child[i] = newhead;
	used++;
      }
    }
    assert(1==used);
    newhead=NULL;
  }
}

void TMAP::quadNode_leafNode::insert(const RASM::point2d &p,
				     unsigned int ind,
				     TMAP::quadNode *&newhead){
  if(shouldAddPointToLeaf(p)){
    //printf("Adding to leaf\n");
    addPointToLeaf(p, ind);
    newhead = NULL;
    return;
  }

  /* need to split the node... */
  newhead = splitLeaf();
  TMAP::quadNode *newnewhead;
  newhead->insert(p, ind, newnewhead);
  assert(NULL == newnewhead);
}

int TMAP::quadNode_interiorNode::lookup(const RASM::point2d &p,
					unsigned int **indices)const{
  return child[selectChild(p)]->lookup(p, indices);
}

int TMAP::quadNode_leafNode::lookup(const RASM::point2d &p,
				    unsigned int **indices)const{
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    if(pointsInLeaf[i]->equals(p)){
      *indices = pointsInLeaf[i]->indices;
      return pointsInLeaf[i]->numreferences;
    }
  }
  return -1;
}

static double minDistSq(const RASM::bounds2d &b, const RASM::point2d &p){
  double dx = 0, dy = 0;
  if     (p.X() <  b.minPoint.X()) dx = RASM_TO_METERS(b.minPoint.X() - p.X());
  else if(p.X() >= b.maxPoint.X()) dx = RASM_TO_METERS(b.maxPoint.X() - p.X());
  if     (p.Y() <  b.minPoint.Y()) dy = RASM_TO_METERS(b.minPoint.Y() - p.Y());
  else if(p.Y() >= b.maxPoint.Y()) dy = RASM_TO_METERS(b.maxPoint.Y() - p.Y());
  return dx*dx + dy*dy;
}

void TMAP::quadNode_interiorNode::lookup(const RASM::point2d &p,
					 RASM::point2d &closest,
					 double &distSq,
					 unsigned int &numIndices,
					 unsigned int **indices)const{

  /* first determine which order to check the children 
     get the minimum potential distance of each and sort them
   */
  double minDists[4];
  int childIndices[4];
  for(int i=0;i<4;i++){
    childIndices[i] = i;
    minDists[i] = minDistSq(child[i]->bounds, p);
  }
  for(int i=0;i<4;i++){
    for(int j=i+1;j<4;j++){
      if(minDists[j]<minDists[i]){
	//swap entries j and i
	int tmpIndex = childIndices[i];
	childIndices[i]=childIndices[j];
	childIndices[j]=tmpIndex;
	double tmpDist = minDists[i];
	minDists[i]=minDists[j];
	minDists[j]=tmpDist;
      }
    }
  }

  /* now recurse on the child in order */
  for(int k=0;k<4;k++){
    int i = childIndices[k];
    double minDist = minDists[k];

    static const double ep = RASM_TO_METERS(RASM_EPSILON);
    if(distSq<0 || minDist < distSq+ep){
#if DEBUG
      printf("Entering subtree with bounds ");
      child[i]->bounds.minPoint.print();
      printf(" and ");
      child[i]->bounds.maxPoint.print();
      printf(" Best possible %lf, only have %lf\n", minDist, distSq);
#endif
      child[i]->lookup(p, closest, distSq, numIndices, indices);      
    }else{
#if DEBUG
      printf("Pruning subtree with bounds ");
      child[i]->bounds.minPoint.print();
      printf(" and ");
      child[i]->bounds.maxPoint.print();
      printf(" Best possible %lf, already have %lf\n", minDist, distSq);
#endif
      //note: we could return here
      //(since we sorted earlier, the remaining children must be further)
    }
  }
}

void TMAP::quadNode_leafNode::lookup(const RASM::point2d &p,
				    RASM::point2d &closest,
				    double &distSq,
				    unsigned int &numIndices,
				    unsigned int **indices)const{
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    double curDistSq = distSq2d(p, *pointsInLeaf[i]);
    if(distSq < 0 || curDistSq < distSq){
      numIndices = pointsInLeaf[i]->numreferences;
      distSq = curDistSq;
      closest = *pointsInLeaf[i];
      *indices = pointsInLeaf[i]->indices;
#if DEBUG
      printf("Looking for ");p.print();printf(" new best %d:", *indices[0]);closest.print();printf(" (%f)\n", distSq);
#endif
    }
  }
}

/* end of TMAP::quadNode definitions */



TMAP::quadTree::quadTree():root(new TMAP::quadNode_rootNode()){}
TMAP::quadTree::quadTree(const RASM::bounds2d &initialBounds)
  :root(new TMAP::quadNode_rootNode(initialBounds)){}
TMAP::quadTree::~quadTree(){assert(root);delete root;root=NULL;}

void TMAP::quadTree::selfCheck()const{
  assert(root);
  root->selfCheck();
}
  
void TMAP::quadTree::insert(const RASM::point2d &p, unsigned int ind){
  selfCheck();
  ((TMAP::quadNode_rootNode *)root)->insert(p, ind);
  selfCheck();

  assert(root->covers(p));
  assert(root->contains(p));
}

void TMAP::quadTree::print() const{
  root->print();
}
  
int TMAP::quadTree::lookup(const RASM::point2d &target,
			   unsigned int **indices) const{
  return ((TMAP::quadNode_rootNode *)root)->lookup(target, indices);
}

int TMAP::quadTree::lookup(const RASM::point2d &target, RASM::point2d &closest,
			   double &distSq, unsigned int **indices) const{
  unsigned int numIndices=0;
  *indices = NULL;
  root->lookup(target, closest, distSq, numIndices, indices);
  if(NULL == *indices)return -1;
  assert(0 != numIndices);
  return (int)numIndices;
}
