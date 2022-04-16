#ifndef TMAP_QUADTREE_H
#define TMAP_QUADTREE_H

#include <rasm_common_types.h>

/* the tree is a collection of interior and leaf nodes, as well as a root node
   each node contains:
    - 2d bounds
    - parent pointer

   interior nodes also contain:
    - a dividing point
    - 4 child pointers (for each quadrant of the dividing point)

   leaf nodes also contain
    - data

   the root node also cotains
    - a pointer to the root leaf or interior node
    - a count of the number of points
   the purpose of the root node is to initialize the bounds of the entire tree
   and gracefully handle spacial expansion of the tree
 */
namespace TMAP {
  class quadNode;
  class quadNode_interiorNode;
  class quadNode_leafNode;
  class quadNode_rootNode;

  class quadNode{
  public:
    quadNode_interiorNode *parent;
    RASM::bounds2d bounds;
    quadNode();
    quadNode(quadNode_interiorNode *_parent, const RASM::bounds2d &_bounds);
    virtual ~quadNode()=0;

    virtual bool covers(const RASM::point2d &p)const { return bounds.contains(p);}
    virtual bool contains(const RASM::point2d &p) const =0;
    virtual void selfCheck() const =0;
    virtual void print(int level=0) const = 0;
    virtual void insert(const RASM::point2d &p, unsigned int index, 
			quadNode *&newhead) = 0;
    virtual int lookup(const RASM::point2d &p, unsigned int **indices) const = 0;
    virtual void lookup(const RASM::point2d &p, RASM::point2d &closest, 
			double &distSq, unsigned int &numIndices,
			unsigned int **indices) const = 0;
  };/* end of quadNode class declaration */


  class quadTree{
  public:
    quadNode *root;
    void selfCheck()const;

    quadTree();
    quadTree(const RASM::bounds2d &initialBounds);
    ~quadTree();

    void insert(const RASM::point2d &p, unsigned int ind);
    void print() const;

    /* finds an exact match
       returns the number of indices that match it (or -1 if not found)
    */
    int lookup(const RASM::point2d &target, unsigned int **indices) const;

    /* finds the closest point, *indices will be NULL if the tree is empty,
       returns the number of indices (or -1 if not found)
    */
    int lookup(const RASM::point2d &target, RASM::point2d &closest,
	       double &distSq, unsigned int **indices) const;

  }; /* end of quadTree class declaration */

}/* end of namespace TMAP */

#endif /* end of TMAP_QUADTREE_H */
