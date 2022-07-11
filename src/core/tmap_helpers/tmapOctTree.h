#ifndef TMAP_OCT_TREE_H
#define TMAP_OCT_TREE_H

#include <rasm_common_types.h>

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
namespace TMAP {
  class octNode;
  class octNode_interiorNode;
  class octNode_leafNode;
  class octNode_rootNode;

  class octNode{
  public:
    octNode_interiorNode *parent;
    RASM::bounds3d bounds;
    octNode();
    octNode(octNode_interiorNode *_parent, const RASM::bounds3d &_bounds);
    virtual ~octNode()=0;

    virtual bool covers(const RASM::point3d &p)const { return bounds.contains(p);}
    virtual bool contains(const RASM::point3d &p) const =0;
    virtual void selfCheck() const =0;
    virtual void print(int level=0) const = 0;
    virtual void insert(const RASM::point3d &p, unsigned int ind,
			octNode *&newhead) = 0;
    virtual int lookup(const RASM::point3d &p, unsigned int **indices) const = 0;
    virtual void lookup(const RASM::point3d &p, RASM::point3d &closest,
			double &distSq, unsigned int &numIndices,
			unsigned int **indices) const = 0;
  };/* end of octNode class declaration */


  class octTree{
  public:
    octNode *root;
    void selfCheck()const;

    octTree();
    octTree(const RASM::bounds3d &initialBounds);
    ~octTree();

    void insert(const RASM::point3d &p, unsigned int ind);
    void print() const;
  
    /* finds an exact match
       returns the number of indices that match it (or -1 if not found)
    */
    int lookup(const RASM::point3d &target, unsigned int **indices) const;

    /* finds the closest point, *indices will be NULL if the tree is empty,
       returns the number of indices (or -1 if not found)
    */
    int lookup(const RASM::point3d &target, RASM::point3d &closest,
	       double &distSq, unsigned int **indices) const;

  }; /* end of octTree class declaration */

}/* end of namespace TMAP */

#endif /* end of TMAP_OCT_TREE_H */
