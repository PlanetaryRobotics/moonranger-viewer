#include <rasm_common_types.h>
#include <tmapOctTree.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

/* pointinfo3d helper class 
   associates a 3d point with a collection of indices
   indices may be either a vertex or triangle index 
   depending on what the tree is used for
*/
class pointinfo3d:public RASM::point3d{
public:
  unsigned int numreferences;
  unsigned int *indices;

  pointinfo3d(const RASM::point3d &p, unsigned int N, unsigned int *I)
    :RASM::point3d(p), 
     numreferences(N), 
     indices(I){}
  pointinfo3d(const RASM::point3d &p, unsigned int ind)
    :RASM::point3d(p), numreferences(1){
    indices = (unsigned int *)malloc(sizeof(unsigned int));
    assert(indices);
    *indices = ind;
  }

  ~pointinfo3d(){
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
/* end of class pointinfo3d */


/* the tree is a collection of interior and leaf nodes, as well as a root node
   each node contains:
    - 3d bounds
    - parent pointer 
    
   interior nodes also contain:
    - a dividing point
    - 8 child pointers (for each octant of the dividing point)

   leaf nodes also contain
    - data

   the root node also cotains
    - a pointer to the root leaf or interior node
    - a count of the number of points
   the purpose of the root node is to initialize the bounds of the entire tree
   and gracefully handle spacial expansion of the tree
 */
TMAP::octNode::octNode():parent(NULL){}
TMAP::octNode::octNode(octNode_interiorNode *_parent, const RASM::bounds3d &_bounds)
  :parent(_parent), bounds(_bounds){}
TMAP::octNode::~octNode(){}

namespace TMAP {
  class octNode_interiorNode : public octNode{
  private:
    /* this helper function is used by the constructor,
       it makes the bounds twice as big attempting to cover the point
    */
    void doubleBounds(octNode *&child, const RASM::point3d &p);

    /* private copy constructor */
    octNode_interiorNode(octNode_interiorNode *copy);
  
    /* helper to select a child node, 
       just looks at which side of the dividing point the target falls */
    inline int selectChild(const RASM::point3d &p) const{
      if(p.X()<dividingPoint.X()){
	if(p.Y()<dividingPoint.Y()){
	  if(p.Z()<dividingPoint.Z())return 0;
	  return 1;
	}
	if(p.Z()<dividingPoint.Z())return 2;
	return 3;
      }
      if(p.Y()<dividingPoint.Y()){
	if(p.Z()<dividingPoint.Z())return 4;
	return 5;
      }
      if(p.Z()<dividingPoint.Z())return 6;
      return 7;
    }

  public:
    /* the 'center' of this node
       this isn't necessarily the average of the corners,
       instead it is the dividing point when deciding which child to use
    */
    RASM::point3d dividingPoint;

    /* the eight children */
    octNode *child[8];

    /* this constructor takes a node and point that does not fall in the node
     * it creates a parent node that contains the old node
     * as well as seven leaf nodes
     */
    octNode_interiorNode(octNode *child, const RASM::point3d &p); 

    /* creates an interior node with this data  */
    octNode_interiorNode(octNode_interiorNode *_parent,
			 const RASM::bounds3d &_bounds,
			 const RASM::point3d &_dividingPoint,
			 octNode *children[8]);

    virtual ~octNode_interiorNode();
    virtual void selfCheck() const;
    virtual void print(int level=0) const;
    virtual bool contains(const RASM::point3d &p)const;
    virtual void insert(const RASM::point3d &p, unsigned int ind, 
			octNode *&newhead);
    virtual int lookup(const RASM::point3d &p, unsigned int **indices)const;
    virtual void lookup(const RASM::point3d &p, RASM::point3d &closest,
			double &distSq, unsigned int &numIndices,
			unsigned int **indices) const;
  };/* end of octNode_interiorNode class declaration */

  class octNode_leafNode : public octNode{
  public:
    /* how many points should we stuff into the leaf node of an oct tree
     * 10 is a good value; 
     * small enough so that linear searches of the leaf nodes are still fast,
     * large enough to significantly reduce the number of leaf nodes
     *
     * In leaf nodes this array will be allocated.
     * In interior nodes, this will be NULL and numPointsInLeaf will be 0
     */
#define MAX_POINTS_IN_LEAF 5
    pointinfo3d **pointsInLeaf;
    unsigned int numPointsInLeaf;
  

    octNode_leafNode(octNode_interiorNode *_parent, const RASM::bounds3d &_bounds);
    octNode_leafNode(octNode_interiorNode *_parent, const RASM::bounds3d &_bounds,
		     const RASM::point3d &point, unsigned int ind);

    void addPointToLeaf(struct pointinfo3d *addme);
    void addPointToLeaf(const RASM::point3d &p, unsigned int ind,
			int allowDuplicates=0, int forceDuplicates=0);

    /* transforms this leaf node into an interior one 
     * by creating 8 evenly spaced child nodes and pushes the data into them 
     * this node becomes one of those child nodes
     * so all references need to be updated
     */
    octNode_interiorNode *splitLeaf();

    /* increases the bounds to fit this point,
     * returns true if anything was changed
     */
    bool expandBounds(const RASM::point3d &p);

    /* checks if this point should be added to this leaf
     * (ie if there's room)
     */
    bool shouldAddPointToLeaf(const RASM::point3d &p, int allowDuplicates=0,
			      int forceDuplicates=0) const;

    virtual ~octNode_leafNode();
    virtual void selfCheck() const;
    virtual void print(int level=0) const;
    virtual bool contains(const RASM::point3d &p)const;
    virtual void insert(const RASM::point3d &p, unsigned int ind,
			octNode *&newhead);
    virtual int lookup(const RASM::point3d &p, unsigned int **indices)const;
    virtual void lookup(const RASM::point3d &p, RASM::point3d &closest,
			double &distSq, unsigned int &numIndices,
			unsigned int **indices) const;
  };/* end of octNode_leafNode class declaration */


  class octNode_rootNode : public octNode{
  protected:
    unsigned int numPoints;
    octNode *root;

  public:
    octNode_rootNode():octNode(), numPoints(0), root(NULL){}
    octNode_rootNode(const RASM::bounds3d &initialBounds)
      :octNode(NULL, initialBounds), numPoints(0), root(NULL){}

    virtual ~octNode_rootNode(){
      numPoints=0;
      assert(root);
      delete root;
      root = NULL;
    }

    virtual void selfCheck() const{if(root)root->selfCheck();}
    virtual void print(int level=0) const{
      printf("This oct tree contains %d points: \n", numPoints);
      if(root)
	root->print(level+1);
    }

    virtual bool contains(const RASM::point3d &p)const{
      if(!root)return 0;
      return root->contains(p);
    }

    virtual void insert(const RASM::point3d &p, unsigned int ind,
			octNode *&newhead){
      if(0 == numPoints){
	if(0 == distSq2d(bounds.minPoint, bounds.maxPoint)){
	  /* were not given any initial bounds */
	  bounds.minPoint=p;
	  bounds.maxPoint=p+RASM::point3d(RASM_EPSILON, RASM_EPSILON, RASM_EPSILON);
	}else{
	  /* we were given some initial bounds,
	   * make sure they include the first point
	   */
	  bounds.expandTo(p);
	}
	assert(!root);
	root = new octNode_leafNode(NULL, bounds, p, ind);

      }else if(numPoints < MAX_POINTS_IN_LEAF){
	assert(root);
	assert(((octNode_leafNode *)root)->shouldAddPointToLeaf(p, ind));
	((octNode_leafNode *)root)->expandBounds(p);
	assert(root->covers(p));
	((octNode_leafNode *)root)->addPointToLeaf(p, ind, 1);
	newhead = NULL;

      }else{
	assert(root);

	if(MAX_POINTS_IN_LEAF == numPoints){
	  octNode_leafNode *oldroot = (octNode_leafNode *)root;
	  root = oldroot->splitLeaf();
	  assert(root->covers(p) == oldroot->covers(p));
	  delete oldroot;
	}

	while(!root->covers(p))
	  root = new octNode_interiorNode(root, p);
	root->insert(p, ind, newhead);
	assert(NULL == newhead);	
      }
      
      numPoints++;
    }

    void inline insert(const RASM::point3d &p, unsigned int ind){
      TMAP::octNode *dummy;
      insert(p, ind, dummy);
    }
    virtual int lookup(const RASM::point3d &p, unsigned int **indices)const{
      if(!root)return -1;
      return root->lookup(p, indices);
    }
    virtual void lookup(const RASM::point3d &p, RASM::point3d &closest,
			double &distSq, unsigned int &numIndices,
			unsigned int **indices) const{
      distSq = -1;
      *indices = NULL;
      if(numPoints>0)
	root->lookup(p, closest, distSq, numIndices, indices);
    }
    
    virtual bool covers(const RASM::point3d &p)const { 
      if(numPoints<=0)return 0;
      return root->covers(p);
    }
  };/* end of octNode_rootNode class declaration */
  
  /* end of declaration of octNode classes; interior, leaf and root */
}/* end of namespace TMAP */


/* start of selfCheck() definition */  
void TMAP::octNode_interiorNode::selfCheck() const{
  for(int i=0;i<8;i++)
    assert(child[i]);

  assert(RASM_UNITS_equals(child[0]->bounds.minPoint.X(),bounds.minPoint.X()));
  assert(RASM_UNITS_equals(child[0]->bounds.minPoint.Y(),bounds.minPoint.Y()));
  assert(RASM_UNITS_equals(child[0]->bounds.minPoint.Z(),bounds.minPoint.Z()));
  assert(RASM_UNITS_equals(child[0]->bounds.maxPoint.X(),  dividingPoint.X()));
  assert(RASM_UNITS_equals(child[0]->bounds.maxPoint.Y(),  dividingPoint.Y()));
  assert(RASM_UNITS_equals(child[0]->bounds.maxPoint.Z() , dividingPoint.Z()));

  assert(RASM_UNITS_equals(child[1]->bounds.minPoint.X(),bounds.minPoint.X()));
  assert(RASM_UNITS_equals(child[1]->bounds.minPoint.Y(),bounds.minPoint.Y()));
  assert(RASM_UNITS_equals(child[1]->bounds.minPoint.Z(),  dividingPoint.Z()));
  assert(RASM_UNITS_equals(child[1]->bounds.maxPoint.X(),  dividingPoint.X()));
  assert(RASM_UNITS_equals(child[1]->bounds.maxPoint.Y(),  dividingPoint.Y()));
  assert(RASM_UNITS_equals(child[1]->bounds.maxPoint.Z(),bounds.maxPoint.Z()));

  assert(RASM_UNITS_equals(child[2]->bounds.minPoint.X(),bounds.minPoint.X()));
  assert(RASM_UNITS_equals(child[2]->bounds.minPoint.Y(),  dividingPoint.Y()));
  assert(RASM_UNITS_equals(child[2]->bounds.minPoint.Z(),bounds.minPoint.Z()));
  assert(RASM_UNITS_equals(child[2]->bounds.maxPoint.X(),  dividingPoint.X()));
  assert(RASM_UNITS_equals(child[2]->bounds.maxPoint.Y(),bounds.maxPoint.Y()));
  assert(RASM_UNITS_equals(child[2]->bounds.maxPoint.Z(),  dividingPoint.Z()));

  assert(RASM_UNITS_equals(child[3]->bounds.minPoint.X(),bounds.minPoint.X()));
  assert(RASM_UNITS_equals(child[3]->bounds.minPoint.Y(),  dividingPoint.Y()));
  assert(RASM_UNITS_equals(child[3]->bounds.minPoint.Z(),  dividingPoint.Z()));
  assert(RASM_UNITS_equals(child[3]->bounds.maxPoint.X(),  dividingPoint.X()));
  assert(RASM_UNITS_equals(child[3]->bounds.maxPoint.Y(),bounds.maxPoint.Y()));
  assert(RASM_UNITS_equals(child[3]->bounds.maxPoint.Z(),bounds.maxPoint.Z()));

  assert(RASM_UNITS_equals(child[4]->bounds.minPoint.X(),  dividingPoint.X()));
  assert(RASM_UNITS_equals(child[4]->bounds.minPoint.Y(),bounds.minPoint.Y()));
  assert(RASM_UNITS_equals(child[4]->bounds.minPoint.Z(),bounds.minPoint.Z()));
  assert(RASM_UNITS_equals(child[4]->bounds.maxPoint.X(),bounds.maxPoint.X()));
  assert(RASM_UNITS_equals(child[4]->bounds.maxPoint.Y(),  dividingPoint.Y()));
  assert(RASM_UNITS_equals(child[4]->bounds.maxPoint.Z(),  dividingPoint.Z()));

  assert(RASM_UNITS_equals(child[5]->bounds.minPoint.X(),  dividingPoint.X()));
  assert(RASM_UNITS_equals(child[5]->bounds.minPoint.Y(),bounds.minPoint.Y()));
  assert(RASM_UNITS_equals(child[5]->bounds.minPoint.Z(),  dividingPoint.Z()));
  assert(RASM_UNITS_equals(child[5]->bounds.maxPoint.X(),bounds.maxPoint.X()));
  assert(RASM_UNITS_equals(child[5]->bounds.maxPoint.Y(),  dividingPoint.Y()));
  assert(RASM_UNITS_equals(child[5]->bounds.maxPoint.Z(),bounds.maxPoint.Z()));

  assert(RASM_UNITS_equals(child[6]->bounds.minPoint.X(),  dividingPoint.X()));
  assert(RASM_UNITS_equals(child[6]->bounds.minPoint.Y(),  dividingPoint.Y()));
  assert(RASM_UNITS_equals(child[6]->bounds.minPoint.Z(),bounds.minPoint.Z()));
  assert(RASM_UNITS_equals(child[6]->bounds.maxPoint.X(),bounds.maxPoint.X()));
  assert(RASM_UNITS_equals(child[6]->bounds.maxPoint.Y(),bounds.maxPoint.Y()));
  assert(RASM_UNITS_equals(child[6]->bounds.maxPoint.Z(),  dividingPoint.Z()));

  assert(RASM_UNITS_equals(child[7]->bounds.minPoint.X(),  dividingPoint.X()));
  assert(RASM_UNITS_equals(child[7]->bounds.minPoint.Y(),  dividingPoint.Y()));
  assert(RASM_UNITS_equals(child[7]->bounds.minPoint.Z(),  dividingPoint.Z()));
  assert(RASM_UNITS_equals(child[7]->bounds.maxPoint.X(),bounds.maxPoint.X()));
  assert(RASM_UNITS_equals(child[7]->bounds.maxPoint.Y(),bounds.maxPoint.Y()));
  assert(RASM_UNITS_equals(child[7]->bounds.maxPoint.Z(),bounds.maxPoint.Z()));
}
  
void TMAP::octNode_leafNode::selfCheck() const{
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
void TMAP::octNode::print(int level) const{
  for(int i=0;i<level;i++)printf(" ");
  printf("Level %d node", level);
  //printf(" at address 0x%08x", (unsigned int)this);

  printf(" with bounds ");
  bounds.minPoint.print();
  printf(" and ");
  bounds.maxPoint.print();

  printf("\n");
}

void TMAP::octNode_interiorNode::print(int level) const{
  TMAP::octNode::print(level);

  for(int i=0;i<level;i++)printf(" ");
  printf("Interior node centered at ");
  dividingPoint.print();
  printf("\n");

  if(level<0)return;
  for(int i=0;i<8;i++){
    assert(child[i]);
    child[i]->print(level+1);
  }
}

void TMAP::octNode_leafNode::print(int level) const{
  TMAP::octNode::print(level);

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
bool TMAP::octNode_interiorNode::contains(const RASM::point3d &p) const{
  return child[selectChild(p)]->contains(p);
}
bool TMAP::octNode_leafNode::contains(const RASM::point3d &p) const{
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    if(*pointsInLeaf[i] == p)
      return 1;
  }
  return 0;
}
/* end of contains() definition */


/* start of constructor definition */
TMAP::octNode_leafNode::octNode_leafNode(TMAP::octNode_interiorNode *_parent, 
					 const RASM::bounds3d &_bounds,
					 const RASM::point3d &point,
					 unsigned int ind)
  :octNode(_parent, _bounds),
   pointsInLeaf(new pointinfo3d*[MAX_POINTS_IN_LEAF]),
   numPointsInLeaf(0){
  pointsInLeaf[numPointsInLeaf++] = new pointinfo3d(point, ind);
}

TMAP::octNode_leafNode::octNode_leafNode(TMAP::octNode_interiorNode *_parent, 
					 const RASM::bounds3d &_bounds)
  :octNode(_parent, _bounds),
   pointsInLeaf(new pointinfo3d*[MAX_POINTS_IN_LEAF]), numPointsInLeaf(0){}
/* end of constructor definition */

/* start of destructor definition */
TMAP::octNode_leafNode::~octNode_leafNode(){
  for(unsigned int i=0;i<numPointsInLeaf;i++)delete pointsInLeaf[i];
  delete[] pointsInLeaf;
  pointsInLeaf = NULL;
}
/* end of destructor definition */

/* this helper function is used by the constructor,
   it makes the bounds twice as big attempting to cover the point
*/
void TMAP::octNode_interiorNode::doubleBounds(TMAP::octNode *&childNode,
					      const RASM::point3d &p){
  RASM_UNITS minExpansion = METERS_TO_RASM(1.0);
  RASM_UNITS dx = bounds.maxPoint.X() - bounds.minPoint.X();
  RASM_UNITS dy = bounds.maxPoint.Y() - bounds.minPoint.Y();
  RASM_UNITS dz = bounds.maxPoint.Z() - bounds.minPoint.Z();

  if(dx<minExpansion)dx=minExpansion;
  if(dy<minExpansion)dy=minExpansion;
  if(dz<minExpansion)dz=minExpansion;

  if(p.X()>=bounds.maxPoint.X()){
    /* fill in the X component */
    dividingPoint.coord3d[0] = bounds.maxPoint.X();
    bounds.maxPoint.coord3d[0] += dx;

    if(p.Y()>=bounds.maxPoint.Y()){
      /* fill in the Y component */
      dividingPoint.coord3d[1] = bounds.maxPoint.Y();
      bounds.maxPoint.coord3d[1] += dy;

      if(p.Z()>=bounds.maxPoint.Z()){/* PPP */
	dividingPoint.coord3d[2] = bounds.maxPoint.Z();
	bounds.maxPoint.coord3d[2] += dz;
	child[0] = childNode;

      }else{/* PPN */
	dividingPoint.coord3d[2] = bounds.minPoint.Z();
	bounds.minPoint.coord3d[2] -= dz;
	child[1] = childNode;
      }

    }else{
      /* fill in the Y component */
      dividingPoint.coord3d[1] = bounds.minPoint.Y();
      bounds.minPoint.coord3d[1] -= dy;

      if(p.Z()>=bounds.maxPoint.Z()){/* PNP */
	dividingPoint.coord3d[2] = bounds.maxPoint.Z();
	bounds.maxPoint.coord3d[2] += dz;
	child[2] = childNode;

      }else{/* PNN */
	dividingPoint.coord3d[2] = bounds.minPoint.Z();
	bounds.minPoint.coord3d[2] -= dz;
	child[3] = childNode;
      }
    }

  }else{
    /* fill in the X component */
    dividingPoint.coord3d[0] = bounds.minPoint.X();
    bounds.minPoint.coord3d[0] -= dx;

    if(p.Y()>=bounds.maxPoint.Y()){
      /* fill in the Y component */
      dividingPoint.coord3d[1] = bounds.maxPoint.Y();
      bounds.maxPoint.coord3d[1] += dy;

      if(p.Z()>=bounds.maxPoint.Z()){/* NPP */
	dividingPoint.coord3d[2] = bounds.maxPoint.Z();
	bounds.maxPoint.coord3d[2] += dz;
	child[4] = childNode;

      }else{/* NPN */
	dividingPoint.coord3d[2] = bounds.minPoint.Z();
	bounds.minPoint.coord3d[2] -= dz;
	child[5] = childNode;
      }

    }else{
      /* fill in the Y component */
      dividingPoint.coord3d[1] = bounds.minPoint.Y();
      bounds.minPoint.coord3d[1] -= dy;

      if(p.Z()>=bounds.maxPoint.Z()){/* NNP */
	dividingPoint.coord3d[2] = bounds.maxPoint.Z();
	bounds.maxPoint.coord3d[2] += dz;
	child[6] = childNode;

      }else{/* NNN */
	dividingPoint.coord3d[2] = bounds.minPoint.Z();
	bounds.minPoint.coord3d[2] -= dz;
	child[7] = childNode;
      }
    }
  }

  /* fill in all the other children with empty leaf nodes */
  if(!child[0])child[0] = new TMAP::octNode_leafNode(this, RASM::bounds3d(RASM::point3d(bounds.minPoint.X(), bounds.minPoint.Y(), bounds.minPoint.Z()),
									  RASM::point3d(dividingPoint.X(),   dividingPoint.Y(),   dividingPoint.Z())));
  if(!child[1])child[1] = new TMAP::octNode_leafNode(this, RASM::bounds3d(RASM::point3d(bounds.minPoint.X(), bounds.minPoint.Y(), dividingPoint.Z()),
									  RASM::point3d(dividingPoint.X(),   dividingPoint.Y(),   bounds.maxPoint.Z())));
  if(!child[2])child[2] = new TMAP::octNode_leafNode(this, RASM::bounds3d(RASM::point3d(bounds.minPoint.X(), dividingPoint.Y(),   bounds.minPoint.Z()),
									  RASM::point3d(dividingPoint.X(),   bounds.maxPoint.Y(), dividingPoint.Z())));
  if(!child[3])child[3] = new TMAP::octNode_leafNode(this, RASM::bounds3d(RASM::point3d(bounds.minPoint.X(), dividingPoint.Y(),   dividingPoint.Z()),
									  RASM::point3d(dividingPoint.X(),   bounds.maxPoint.Y(), bounds.maxPoint.Z())));
  if(!child[4])child[4] = new TMAP::octNode_leafNode(this, RASM::bounds3d(RASM::point3d(dividingPoint.X(),   bounds.minPoint.Y(), bounds.minPoint.Z()),
									  RASM::point3d(bounds.maxPoint.X(), dividingPoint.Y(),   dividingPoint.Z())));
  if(!child[5])child[5] = new TMAP::octNode_leafNode(this, RASM::bounds3d(RASM::point3d(dividingPoint.X(),   bounds.minPoint.Y(), dividingPoint.Z()),
									  RASM::point3d(bounds.maxPoint.X(), dividingPoint.Y(),   bounds.maxPoint.Z())));
  if(!child[6])child[6] = new TMAP::octNode_leafNode(this, RASM::bounds3d(RASM::point3d(dividingPoint.X(),   dividingPoint.Y(),   bounds.minPoint.Z()),
									  RASM::point3d(bounds.maxPoint.X(), bounds.maxPoint.Y(), dividingPoint.Z())));
  if(!child[7])child[7] = new TMAP::octNode_leafNode(this, RASM::bounds3d(RASM::point3d(dividingPoint.X(),   dividingPoint.Y(),   dividingPoint.Z()),
									  RASM::point3d(bounds.maxPoint.X(), bounds.maxPoint.Y(), bounds.maxPoint.Z())));

  childNode->parent = this;
}

/* private copy constructor */
TMAP::octNode_interiorNode::octNode_interiorNode(TMAP::octNode_interiorNode *_parent)
  :octNode(_parent, _parent->bounds),
  dividingPoint(_parent->dividingPoint){
  for(int i=0;i<8;i++)child[i] = _parent->child[i];
}

/* this constructor takes a node and point that does not fall in the node
 * it creates a parent node that contains the old node
 * as well as three leaf nodes
 */
TMAP::octNode_interiorNode::octNode_interiorNode(TMAP::octNode *childNode, 
						 const RASM::point3d &p)
  :octNode(NULL, childNode->bounds){
  for(int i=0;i<8;i++)child[i]=NULL;

  assert(!covers(p));
  doubleBounds(childNode, p);
}

/* creates an interior node with this data  */
TMAP::octNode_interiorNode::octNode_interiorNode(TMAP::octNode_interiorNode *_parent,
						 const RASM::bounds3d &_bounds,
						 const RASM::point3d &_dividingPoint,
						 TMAP::octNode *children[8])
  :octNode(_parent, _bounds),
  dividingPoint(_dividingPoint){
  for(int i=0;i<8;i++){
    assert(children[i]);
    child[i]=children[i];
    child[i]->parent = this;
  }
}

TMAP::octNode_interiorNode::~octNode_interiorNode(){
  for(int i=0;i<8;i++)delete child[i];
}


void TMAP::octNode_leafNode::addPointToLeaf(struct pointinfo3d *addme){
  assert(numPointsInLeaf < MAX_POINTS_IN_LEAF);
  assert(covers(*addme));
  pointsInLeaf[numPointsInLeaf]=addme;
  numPointsInLeaf++;
}

/* transforms this leaf node into an interior one...
 * creates 8 leaf nodes and pushes the data into them 
 * then creates an interior node with those leaves
 * the resulting interior node is returned and should replace the current one
 */
TMAP::octNode_interiorNode *TMAP::octNode_leafNode::splitLeaf(){

  RASM::point3d dividingPoint = bounds.minPoint + bounds.maxPoint;
  dividingPoint /= (RASM_UNITS)2;

  /* create child nodes */
  TMAP::octNode_leafNode *children[8];
  children[0] = new TMAP::octNode_leafNode(NULL, RASM::bounds3d(RASM::point3d(bounds.minPoint.X(), bounds.minPoint.Y(), bounds.minPoint.Z()),
								RASM::point3d(dividingPoint.X(),   dividingPoint.Y(),   dividingPoint.Z())));
  children[1] = new TMAP::octNode_leafNode(NULL, RASM::bounds3d(RASM::point3d(bounds.minPoint.X(), bounds.minPoint.Y(), dividingPoint.Z()),
								RASM::point3d(dividingPoint.X(),   dividingPoint.Y(),   bounds.maxPoint.Z())));
  children[2] = new TMAP::octNode_leafNode(NULL, RASM::bounds3d(RASM::point3d(bounds.minPoint.X(), dividingPoint.Y(),   bounds.minPoint.Z()),
								RASM::point3d(dividingPoint.X(),   bounds.maxPoint.Y(), dividingPoint.Z())));
  children[3] = new TMAP::octNode_leafNode(NULL, RASM::bounds3d(RASM::point3d(bounds.minPoint.X(), dividingPoint.Y(),   dividingPoint.Z()),
								RASM::point3d(dividingPoint.X(),   bounds.maxPoint.Y(), bounds.maxPoint.Z())));
  children[4] = new TMAP::octNode_leafNode(NULL, RASM::bounds3d(RASM::point3d(dividingPoint.X(),   bounds.minPoint.Y(), bounds.minPoint.Z()),
								RASM::point3d(bounds.maxPoint.X(), dividingPoint.Y(),   dividingPoint.Z())));
  children[5] = new TMAP::octNode_leafNode(NULL, RASM::bounds3d(RASM::point3d(dividingPoint.X(),   bounds.minPoint.Y(), dividingPoint.Z()),
								RASM::point3d(bounds.maxPoint.X(), dividingPoint.Y(),   bounds.maxPoint.Z())));
  children[6] = new TMAP::octNode_leafNode(NULL, RASM::bounds3d(RASM::point3d(dividingPoint.X(),   dividingPoint.Y(),   bounds.minPoint.Z()),
								RASM::point3d(bounds.maxPoint.X(), bounds.maxPoint.Y(), dividingPoint.Z())));
  children[7] = new TMAP::octNode_leafNode(NULL, RASM::bounds3d(RASM::point3d(dividingPoint.X(),   dividingPoint.Y(),   dividingPoint.Z()),
								RASM::point3d(bounds.maxPoint.X(), bounds.maxPoint.Y(), bounds.maxPoint.Z())));
  for(int i=0;i<8;i++)
    assert(children[i]);
    
  /* move our data to the right child node */
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    int used=0;
    for(int j=0;j<8;j++){
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
  TMAP::octNode_leafNode **c = children;/* avoid warning in gcc 4.1.1 */
  return new TMAP::octNode_interiorNode(parent, bounds, dividingPoint, (TMAP::octNode **)c);
}

bool TMAP::octNode_leafNode::expandBounds(const RASM::point3d &p){
  bool ret=false;
  if(p.X()+RASM_EPSILON > bounds.maxPoint.X()){
    bounds.maxPoint.coord3d[0] = p.X()+RASM_EPSILON;
    ret=true;
  }
  if(p.X() < bounds.minPoint.X()){
    bounds.minPoint.coord3d[0] = p.X();
    ret=true;
  }
  if(p.Y()+RASM_EPSILON > bounds.maxPoint.Y()){
    bounds.maxPoint.coord3d[1] = p.Y()+RASM_EPSILON;
    ret=true;
  }
  if(p.Y() < bounds.minPoint.Y()){
    bounds.minPoint.coord3d[1] = p.Y();
    ret=true;
  }
  if(p.Z()+RASM_EPSILON > bounds.maxPoint.Z()){
    bounds.maxPoint.coord3d[2] = p.Z()+RASM_EPSILON;
    ret=true;
  }
  if(p.Z() < bounds.minPoint.Z()){
    bounds.minPoint.coord3d[2] = p.Z();
    ret=true;
  }
  return ret;
}

void TMAP::octNode_leafNode::addPointToLeaf(const RASM::point3d &p,
					    unsigned int ind, 
					    int allowDuplicates,
					    int forceDuplicates){

  assert(covers(p));
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    if(pointsInLeaf[i]->equals(p)){
      assert(allowDuplicates);
      /* append 'ind' to the list of indices */
#if 0
      printf("Re-adding point %f,%f,%f (index %d)\n",
	     (float)p.coord3d[0], (float)p.coord3d[1], (float)p.coord3d[2], ind);
      printf("Old point %f,%f,%f (index %d)\n",
	     (float)pointsInLeaf[i]->coord3d[0], (float)pointsInLeaf[i]->coord3d[1], (float)pointsInLeaf[i]->coord3d[2], pointsInLeaf[i]->indices[0]);
      assert(0);
#endif
      pointsInLeaf[i]->addIndex(ind);
      return;
    }
  }

  assert(!forceDuplicates);
  addPointToLeaf(new pointinfo3d(p, ind));
}
  
bool TMAP::octNode_leafNode::shouldAddPointToLeaf(const RASM::point3d &p, 
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
  
void TMAP::octNode_interiorNode::insert(const RASM::point3d &p,
					unsigned int ind, 
					TMAP::octNode *&newhead){
  assert(covers(p));

  TMAP::octNode *childNode = child[selectChild(p)];
  assert(childNode);
  childNode->insert(p, ind, newhead);

  if(newhead){
    /* we split the child, update our child pointer... */
    int used = 0;
    for(int i=0;i<8;i++){
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

void TMAP::octNode_leafNode::insert(const RASM::point3d &p,
				    unsigned int ind,
				    TMAP::octNode *&newhead){
  if(shouldAddPointToLeaf(p)){
    //printf("Adding to leaf\n");
    addPointToLeaf(p, ind);
    newhead = NULL;
    return;
  }

  /* need to split the node... */
  newhead = splitLeaf();
  TMAP::octNode *newnewhead;
  newhead->insert(p, ind, newnewhead);
  assert(NULL == newnewhead);
}

int TMAP::octNode_interiorNode::lookup(const RASM::point3d &p,
				       unsigned int **indices)const{
  return child[selectChild(p)]->lookup(p, indices);
}

int TMAP::octNode_leafNode::lookup(const RASM::point3d &p,
				   unsigned int **indices)const{
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    if(pointsInLeaf[i]->equals(p)){
      *indices = pointsInLeaf[i]->indices;
      return pointsInLeaf[i]->numreferences;
    }
  }
  return -1;
}

static double minDistSq(const RASM::bounds3d &b, const RASM::point3d &p){
  double dx = 0, dy = 0, dz = 0;
  if     (p.X() <  b.minPoint.X()) dx = RASM_TO_METERS(b.minPoint.X() - p.X());
  else if(p.X() >= b.maxPoint.X()) dx = RASM_TO_METERS(b.maxPoint.X() - p.X());
  if     (p.Y() <  b.minPoint.Y()) dy = RASM_TO_METERS(b.minPoint.Y() - p.Y());
  else if(p.Y() >= b.maxPoint.Y()) dy = RASM_TO_METERS(b.maxPoint.Y() - p.Y());
  if     (p.Z() <  b.minPoint.Z()) dz = RASM_TO_METERS(b.minPoint.Z() - p.Z());
  else if(p.Z() >= b.maxPoint.Z()) dz = RASM_TO_METERS(b.maxPoint.Z() - p.Z());
  return dx*dx + dy*dy + dz*dz;
}

void TMAP::octNode_interiorNode::lookup(const RASM::point3d &p,
					RASM::point3d &closest,
					double &distSq,
					unsigned int &numIndices,
					unsigned int **indices)const{

  /* first determine which order to check the children 
     get the minimum potential distance of each and sort them
   */
  double minDists[8];
  int childIndices[8];
  for(int i=0;i<8;i++){
    childIndices[i] = i;
    minDists[i] = minDistSq(child[i]->bounds, p);
  }
  for(int i=0;i<8;i++){
    for(int j=i+1;j<8;j++){
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
  for(int k=0;k<8;k++){
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

void TMAP::octNode_leafNode::lookup(const RASM::point3d &p,
				    RASM::point3d &closest,
				    double &distSq,
				    unsigned int &numIndices,
				    unsigned int **indices)const{
  for(unsigned int i=0;i<numPointsInLeaf;i++){
    double curDistSq = distSq3d(p, *pointsInLeaf[i]);
    if(distSq < 0 || curDistSq < distSq){
      numIndices = pointsInLeaf[i]->numreferences;
      distSq = curDistSq;
      closest = *pointsInLeaf[i];
      *indices = pointsInLeaf[i]->indices;
      //printf("Looking for ");p.print();printf(" new best ");closest.print();printf(" (%f)\n", distSq);
    }
  }
}

/* end of TMAP::octNode definitions */



TMAP::octTree::octTree():root(new TMAP::octNode_rootNode()){}
TMAP::octTree::octTree(const RASM::bounds3d &initialBounds)
  :root(new TMAP::octNode_rootNode(initialBounds)){}
TMAP::octTree::~octTree(){assert(root);delete root;root=NULL;}

void TMAP::octTree::selfCheck()const{
  assert(root);
  root->selfCheck();
}
  
void TMAP::octTree::insert(const RASM::point3d &p, unsigned int ind){
  selfCheck();
  ((TMAP::octNode_rootNode *)root)->insert(p, ind);
  selfCheck();

  assert(root->covers(p));
  assert(root->contains(p));
}

void TMAP::octTree::print() const{
  root->print();
}
  
int TMAP::octTree::lookup(const RASM::point3d &target,
			  unsigned int **indices) const{
  return ((TMAP::octNode_rootNode *)root)->lookup(target, indices);
}

int TMAP::octTree::lookup(const RASM::point3d &target, RASM::point3d &closest,
			  double &distSq, unsigned int **indices) const{
  unsigned int numIndices=0;
  *indices = NULL;
  root->lookup(target, closest, distSq, numIndices, indices);
  if(NULL == *indices)return -1;
  assert(0 != numIndices);
  return (int)numIndices;
}
