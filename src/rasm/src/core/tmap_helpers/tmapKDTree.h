/*
 * Basic kd-tree implimentation for 3 dimensions.
 * Used for storing a point cloud and finding matches or near matches.
 *
 * Dom@cmu.edu
 */
#ifndef RASM_KD_TREE_H
#define RASM_KD_TREE_H

#include <rasm_common_types.h>
#include <stdlib.h>

namespace TMAP {
  class kdnode{
  public:
    kdnode(){}
    virtual ~kdnode(){}
    virtual void print(FILE *f, unsigned int level) const = 0;
    virtual int lookup(const RASM::point3d &target) const = 0;
    virtual int lookup(const RASM::point3d &target, RASM_UNITS *mindist,
		       double &distSq) const = 0;
  };

  class kdnode_interior:public virtual kdnode{
  public:
    unsigned int axis;/* dividing axis (0, 1, or 2) */
    RASM_UNITS val;/* dividing value */
    kdnode *L, *R;
    kdnode_interior(unsigned int _axis, RASM_UNITS _val):
      kdnode(),axis(_axis), val(_val),L(NULL),R(NULL){}
    ~kdnode_interior(){if(L)delete L;if(R)delete R;}
    void print(FILE *f, unsigned int level) const;
    int lookup(const RASM::point3d &target) const;
    int lookup(const RASM::point3d &target, RASM_UNITS *mindist,
	       double &distSq) const;
  };

  class kdnode_leaf:public virtual kdnode{
  public:
    static const unsigned int MAX_KD_LEAF = 5;
    unsigned int indices[MAX_KD_LEAF];
    RASM::point3d points[MAX_KD_LEAF];
    unsigned int N;
    kdnode_leaf():kdnode(),N(0){}
    ~kdnode_leaf(){}
    void print(FILE *f, unsigned int level) const;
    int lookup(const RASM::point3d &target) const;
    int lookup(const RASM::point3d &target, RASM_UNITS *mindist,
	       double &distSq) const;
    void addToLeaf(const RASM::point3d &p, unsigned int ind);
  };
  /* end of kdnode class declarations */

  class kdTree{
  protected:
    const RASM::point3d *points;
    unsigned int numPoints;

    /* splits the points along the axis
     * fills in the value as the selected division
     * returns the number of points less than the dividing value
     * re-arranges the points such that the first group are less than value
     * and the rest are greater than or equal to the value
     */
    unsigned int splitList(unsigned int *indices, unsigned int len,
			   unsigned int axis, RASM_UNITS &value) const;

    kdnode *insert(unsigned int *indices, unsigned int len, unsigned int axis);

  public:
    kdnode *root;

    kdTree(); 
    ~kdTree();
    
    /* builds a kd-tree of the set of points, no duplicates allowed
     * note that the indices will be the return values for lookups
     */
    void insert(const RASM::point3d *points, unsigned int numPoints);

    void print(FILE *f) const;

    /* finds an exact match (or -1 if not found) */
    int lookup(const RASM::point3d &target) const;

    /* finds the closest match (or -1 if not found) */
    int lookup(const RASM::point3d &target, double &distSq) const;
  }; /* end of kdtree class definition */

} /* end of namespace TMAP */

#endif /* end of TMAP_KD_TREE_H */
