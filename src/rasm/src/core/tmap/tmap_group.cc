#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <tmap_group.h>

/*** helper class to do set union operations ***/
class unionHelper{
private:
  struct unionNode{
    /* each element has a parent, the root is its own parent
     * if two elements have the same parent, they are in the same group
     */
    unsigned int parent;
    /* the size of the group, only valid at the root */
    unsigned int size;
  };
  struct unionNode *data;
  unsigned int N;

public:

  unionHelper(unsigned int size){
    assert(size>0);
    N=size;
    data = (struct unionNode *)malloc(N*sizeof(struct unionNode));
    assert(data);

    /* initialize each element to be in it's own group */
    for(unsigned int i=0;i<N;i++){
      data[i].parent = i;
      data[i].size = 1;
    }
  }
  ~unionHelper(){free(data);}

  unsigned int findRoot(unsigned int a){
    /* already the root? */
    if(a == data[a].parent)return a;

    /* recurse and cache the result */
    unsigned int root = findRoot(data[a].parent);
    data[a].parent = root;
    return root;
  }

  void merge(unsigned int a, unsigned int b){
    /* merge the root of these two groups */
    a = findRoot(a);
    b = findRoot(b);

    /* check if we're already in the same group */
    if(a == b)return;

    /* identify which group is smaller */
    const unsigned int smaller = ((data[a].size < data[b].size)?a:b);
    const unsigned int larger  = ((smaller==a)?b:a);

    /* make the smaller group part of the larger one */
    data[smaller].parent = larger;
    data[larger].size = data[a].size + data[b].size;
  }

  unsigned int identifyGroups(unsigned int *groups){
    /* first flatten the data structure by querying all non-root nodes */
    for(unsigned int i=0;i<N;i++)
      if(data[i].parent != i)
	findRoot(i);

    /* fill in the group index for the root of each group */
    unsigned int numGroups = 0;
    for(unsigned int i=0;i<N;i++)
      if(data[i].parent == i)
	groups[i] = numGroups++;

    /* for the rest of each group, copy the group index from the root */
    for(unsigned int i=0;i<N;i++)
      if(data[i].parent != i)
	groups[i] = groups[data[i].parent];

    return numGroups;
  }
};
/*** end of helper class to do set union operations ***/


unsigned int TMAP::tmap_group::markPointGroups(unsigned int numInd,
					       const unsigned int *ind,
					       unsigned int *groups)const{
  assert((numInd>0) && (numInd<numPoints()) && ind && groups);
  assert(triangleAssociations);
  for(unsigned int i=0;i<numInd;i++){assert(ind[i]<numPoints());}

  /* instantiate the data structure to do merge operations on */
  unionHelper u(numInd);

  /* create a mapping from point index to the array we were given
   * mapping[p] is the location of vertex index p in that array
   * a negative value indicates p is not in the array at all
   */
  int *mapping = (int *)alloca(sizeof(int)*numPoints());
  assert(mapping);
  for(unsigned int i=0;i<numPoints();i++)mapping[i] = -1;
  for(unsigned int i=0;i<numInd;i++)mapping[ind[i]] = i;

  /* now scan through the points to identify triangles
   * then check if those triangles contain multiple points
   * if so, then coallesce them into the same group
   */
  for(unsigned int i=0;i<numInd;i++){
    /* the ith point in the array is point p in the mesh */
    const unsigned int p = ind[i];
    assert(mapping[p] == (int)i);/* sanity check the mapping */

    /* point p is part of numTri triangles */
    const unsigned int numTri = triangleAssociations[p][0];
    for(unsigned int j=0;j<numTri;j++){
      /* point p is part of triangle t in the mesh */
      const unsigned int t = triangleAssociations[p][j+1];

      for(unsigned int k=0;k<3;k++){
	/* triangle t also has point p2, 
	 * so there is an edge between p and p2
	 */
	const unsigned int p2 = faces[t].points[k];

	/* if the edge exists (ie, p and p2 are distinct points)
	 * and point p2 is also in the array, then merge them
	 * note that merging is done in array indices, not point indices
	 */
	if((p2!=p) && (mapping[p2]>=0))
	  u.merge(i, (unsigned int)(mapping[p2]));
      }
    }

  }

  /* use the data structure to pick out each independant group */
  const unsigned int ret = u.identifyGroups(groups);

  return ret;
}
