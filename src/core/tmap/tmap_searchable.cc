#include <tmap_searchable.h>
#include <lineHelpers.h>
#include <planeHelpers.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef VERBOSE
#define VERBOSE 0
#endif

#define DEBUG 0 /* very verbose */
#define CHECK_CLOSEST 0
#define CHECK_2D_TRIANGLE 0


TMAP::tmap_searchable::~tmap_searchable(){
	if(treeOfVertices){
		delete treeOfVertices;
		treeOfVertices = NULL;
	}
	if(treeOfCenters){
		delete treeOfCenters;
		treeOfCenters = NULL;
	}
	if(octOfVertices){
		delete octOfVertices;
		octOfVertices = NULL;
	}
	if(octOfCenters){
		delete octOfCenters;
		octOfCenters = NULL;
	}
}

/* builds a quad tree */
void TMAP::tmap_searchable::buildTreeVertices(){
	if(treeOfVertices)delete treeOfVertices;
	if(octOfVertices)delete octOfVertices;
	
	//printf("Building tree of vertices for %d vertices\n", numPoints());
	
	if(0==numPoints()){
		/* just make empty datastructures */
		treeOfVertices = new TMAP::quadTree();
		octOfVertices = new TMAP::octTree();
		return;
	}
	
	RASM::bounds3d b(vertices[0], vertices[0]);
	for(unsigned int i=1;i<numPoints();i++)
		b.expandTo(vertices[i]);
	treeOfVertices = new TMAP::quadTree(b);
	octOfVertices = new TMAP::octTree(b);
	
#if DEBUG
	printf("Set bounds to ");
	b.minPoint.print();
	printf(" and ");
	b.maxPoint.print();
	printf("\n");
#endif
	
	for(unsigned int i=0;i<numPoints();i++){
		unsigned int *indices=NULL;
		int n = treeOfVertices->lookup(vertices[i], &indices);
		if(n != -1){
			printf("error, point %d already in the tree?\n", i);
			printf(" Vertex %d: ", i);
			vertices[i].print();
			printf("\n");
			for(int j=0;j<n;j++){
				printf(" Vertex %d: ", indices[j]);
				vertices[indices[j]].print();
				printf("\n");
			}
			assert(0);
		}
		treeOfVertices->insert(vertices[i], i);
		
		n = octOfVertices->lookup(vertices[i], &indices);
		if(n != -1){
			printf("error, point %d already in the oct tree?\n", i);
			printf(" Vertex %d: ", i);
			vertices[i].print();
			printf("\n");
			for(int j=0;j<n;j++){
				printf(" Vertex %d: ", indices[j]);
				vertices[indices[j]].print();
				printf("\n");
			}
			assert(0);
		}
		octOfVertices->insert(vertices[i], i);
	}
	
	//printf("Built tree of vertices for %d vertices\n", numPoints());
}

void TMAP::tmap_searchable::buildTreeCenters(){
	assert(centers);
	if(treeOfCenters){delete treeOfCenters;treeOfCenters=NULL;}
	if(octOfCenters){delete octOfCenters;octOfCenters=NULL;}
	
	//printf("Building tree of centers for %d faces\n", numTriangles());
	
	if(0==numTriangles()){
		/* just make empty datastructures */
		treeOfCenters = new TMAP::quadTree();
		octOfCenters = new TMAP::octTree();
		return;
	}
	
	RASM::bounds3d b(centers[0], centers[0]);
	for(unsigned int i=1;i<numTriangles();i++)
		b.expandTo(centers[i]);
	treeOfCenters = new TMAP::quadTree(b);
	octOfCenters = new TMAP::octTree(b);
	
#if DEBUG
	printf("Set bounds to ");
	b.minPoint.print();
	printf(" and ");
	b.maxPoint.print();
	printf("\n");
#endif
	
	for(unsigned int i=0;i<numTriangles();i++){
		unsigned int *indices=NULL;
		int n = treeOfCenters->lookup(centers[i], &indices);
		if(-1 != n){
#if VERBOSE
			printf("Warning, not adding center of triangle %d (same as triangle %d)\n",
				   i, indices[0]);
#endif
		}
		
		if(1 < n){
			printf("Error inserting center...");
			printf(" Triangle %d: ", i);
			centers[i].print();
			printf("\n");
			for(int k=0;k<3;k++){
				printf("   point %d: ", faces[i].points[k]);
				vertices[faces[i].points[k]].print();
				printf("\n");
			}
			
			
			printf("Already contains this point %d times\n", n);
			for(int j=0;j<n;j++){
				unsigned int t = indices[j];
				printf("  Center of %d: ", t);
				centers[t].print();
				printf("\n");
				for(int k=0;k<3;k++){
					printf("   point %d: ", faces[t].points[k]);
					vertices[faces[t].points[k]].print();
					printf("\n");
				}
			}
			assert(0);
		}
		int success = octOfCenters->lookup(centers[i], &indices);
    assert(2 > success);
		
		
		//printf("Inserting face %d of %d\n", i, numTriangles());
		
		if(-1 == n){
			treeOfCenters->insert(centers[i], i);
			octOfCenters->insert(centers[i], i);
		}
		
		//printf("Inserted face %d of %d\n", i, numTriangles());
	}
}

/* builds a quad tree */
void TMAP::tmap_searchable::updateTreeVertices(unsigned int n){
	assert(treeOfVertices);
	assert(octOfVertices);
	assert(numPoints()>n);
	
	for(unsigned int i=numPoints()-n;i<numPoints();i++){
		unsigned int *indices=NULL;
		int n = treeOfVertices->lookup(vertices[i], &indices);
		if(n != -1){
			printf("error, point %d already in the tree?\n", i);
			printf(" Vertex %d: ", i);
			vertices[i].print();
			printf("\n");
			for(int j=0;j<n;j++){
				printf(" Vertex %d: ", indices[j]);
				vertices[indices[j]].print();
				printf("\n");
			}
			assert(0);
		}
		treeOfVertices->insert(vertices[i], i);
		
		n = octOfVertices->lookup(vertices[i], &indices);
		if(n != -1){
			printf("error, point %d already in the oct tree?\n", i);
			printf(" Vertex %d: ", i);
			vertices[i].print();
			printf("\n");
			for(int j=0;j<n;j++){
				printf(" Vertex %d: ", indices[j]);
				vertices[indices[j]].print();
				printf("\n");
			}
			assert(0);
		}
		octOfVertices->insert(vertices[i], i);
	}
}


#if CHECK_CLOSEST
void checkClosest(const RASM::point3d *array, unsigned int numPoints,
				  const RASM::point2d &target, unsigned int ind, 
				  const TMAP::tmap_searchable *source){
	/* loop through all the points the slow way to verify this was correct */
	unsigned int closest=0;
	for(unsigned int i=1;i<numPoints;i++)
		if(distSq2d(target, array[i]) < distSq2d(target, array[closest]))
			closest=i;
	
	static const double ep = RASM_TO_METERS(RASM_EPSILON);
	if(closest != ind && (distSq2d(target, array[closest]) + ep < distSq2d(target, array[ind]))){
		printf("Error, closest point in 2d is actually %d, not %d\n", closest, ind);
		printf("Target: ");target.print();printf("\n");
		printf("Point %d: ", closest);array[closest].print();printf(" dist sq = %f\n", distSq2d(target, array[closest]));
		printf("Point %d: ", ind);  array[ind].print();  printf(" dist sq = %f\n", distSq2d(target, array[ind]));
		printf(" dist sq = %f <? %f: %d\n", distSq2d(target, array[closest]), distSq2d(target, array[ind]),
			   (distSq2d(target, array[closest])<distSq2d(target, array[ind])));
		printf(" %f - %f = %g\n", distSq2d(target, array[closest]), distSq2d(target, array[ind]),
			   (distSq2d(target, array[closest]) - distSq2d(target, array[ind])));
		source->writeToFile("failedSearch.smf");
		printf("Saved this as failedSearch.smf\n");
		assert(0);
	}
}
void checkClosest(const RASM::point3d *array, unsigned int numPoints,
				  const RASM::point3d &target, unsigned int ind){
	/* loop through all the points the slow way to verify this was correct */
	int closest=-1;
	for(unsigned int i=0;i<numPoints;i++)
		if(closest<0 || distSq3d(target, array[i]) < distSq3d(target, array[closest]))
			closest=i;
	
	static const float ep = RASM_TO_METERS(RASM_EPSILON);
	if(closest != (int)ind && (distSq3d(target, array[closest]) + ep < distSq3d(target, array[ind]))){
		printf("Error, closest point in 3d is actually %d, not %d\n", closest, ind);
		printf("Target: ");target.print();printf("\n");
		printf("Point %d: ", closest);array[closest].print();printf(" dist sq = %f\n", distSq3d(target, array[closest]));
		printf("Point %d: ", ind);  array[ind].print();  printf(" dist sq = %f\n", distSq3d(target, array[ind]));
		assert(0);
	}
}
#endif


unsigned int TMAP::tmap_searchable::findClosestVertex(const RASM::point2d &target)const{
	assert(numPoints()>0);
	
	assert(treeOfVertices);
	return findClosestVertex2d(target);
}

unsigned int TMAP::tmap_searchable::findClosestVertex(const RASM::point3d &target)const{
	assert(numPoints()>0);
	
	//assert(treeOfVertices);
	//return findClosestVertex2d(target);
	assert(octOfVertices);
	return findClosestVertex3d(target);
}

unsigned int TMAP::tmap_searchable::findClosestCenter(const RASM::point2d &target)const{
	assert(centers && numTriangles()>0);
	
	assert(treeOfCenters);
	return findClosestCenter2d(target);
}

unsigned int TMAP::tmap_searchable::findClosestCenter(const RASM::point3d &target)const{
	assert(centers && numTriangles()>0);
	
	//assert(treeOfCenters);
	//return findClosestCenter2d(target);
	assert(octOfCenters);
	return findClosestCenter3d(target);
}

unsigned int TMAP::tmap_searchable::findClosestVertex2d(const RASM::point2d &target)const{
	unsigned int *indices=NULL;
	RASM::point2d closest;
	double d=-1;
	int ret = treeOfVertices->lookup(target, closest, d, &indices);
	if(1 != ret){
		printf("Error finding closest vertex to ");
		target.print();
		printf("\n");
		treeOfVertices->print();
		printf("lookup() returned %d instead of 1\n", ret);
		printf("This map contains %d points\n", numPoints());
		assert(0);
	}
	
#if CHECK_CLOSEST
	checkClosest(vertices, numPoints(), target, indices[0], this);
#endif
	
	return indices[0];
}

unsigned int TMAP::tmap_searchable::findClosestCenter2d(const RASM::point2d &target)const{
	unsigned int *indices=NULL;
	RASM::point2d closest;
	double d=-1;
	int ret = treeOfCenters->lookup(target, closest, d, &indices);
	assert(1 == ret);
	
#if CHECK_CLOSEST
	checkClosest(centers, numTriangles(), target, indices[0], this);
#endif
	
	return indices[0];
}


unsigned int TMAP::tmap_searchable::findClosestVertex3d(const RASM::point3d &target)const{
	assert(numPoints()>0);
	unsigned int *indices=NULL;
	RASM::point3d closest;
	double d=-1;
	int n = octOfVertices->lookup(target, closest, d, &indices);
	if(1 != n){
		printf("Error, findClosestVertex3d found %d points instead of 1\n", n);
		printf("Have %d vertices\n", numPoints());
		assert(0);
	}
	
#if CHECK_CLOSEST
	checkClosest(vertices, numPoints(), target, indices[0]);
#endif
	
	return indices[0];
}

unsigned int TMAP::tmap_searchable::findClosestCenter3d(const RASM::point3d &target)const{
	unsigned int *indices=NULL;
	RASM::point3d closest;
	double d=-1;
	int ret = octOfCenters->lookup(target, closest, d, &indices);
  assert(ret == 1);
	
#if CHECK_CLOSEST
	checkClosest(centers, numTriangles(), target, indices[0]);
#endif
	
	return indices[0];
}

/* given the indices of three points that define triangle A
 * and indicies of the three points that define triangle B
 * fill in the shared array with the two common points
 * and sets shared[2] to be the unused point in A
 *
 * if triangles A and B do NOT share an edge, returns -1
 */
static int getSharedPoints(const unsigned int pointsA[3],
						   const unsigned int pointsB[3],
						   unsigned int shared[3]){
	/* mark a point if it is in B */
	bool mask[3];
	for(int i=0;i<3;i++)
		mask[i] = ((pointsA[i] == pointsB[0]) ||
				   (pointsA[i] == pointsB[1]) ||
				   (pointsA[i] == pointsB[2]));
	
	int n=0;
	
	/* verify there are exactly 2 points in B */
	for(int i=0;i<3;i++)
		if(mask[i])
			n++;
	if(2!=n)return -1;
	
	/* pick the single point N that is not in B */
	for(int i=0;i<3;i++)
		if(!mask[i])
			n=i;
	
	/* fill in the array such that shared[2] = pointsB[N]
	 * and the other two are in shared[0] and shared[1]
	 */
	for(int i=0;i<3;i++)
		shared[i] = pointsA[(n+1+i)%3];
	
	return true;
}

int TMAP::tmap_searchable::centerSearch(const RASM::point2d &target,
										unsigned int ind, int verbose) const{
	if(contains(vertices, faces[ind], target, verbose)){
		if(verbose){
			printf("centerSearch() immediate success\n");
		}
		return ind;
	}
	
	unsigned int prevIndex = ind;
	unsigned int consideredFaces=0;
#if DEBUG
	unsigned int step=0;
#endif
	while(1){
		consideredFaces++;
#if DEBUG
		printf("Step %d At triangle %d, distance %f\n", ++step, ind, dist2d(centers[ind], target));
#endif
		if(consideredFaces>=(1+numTriangles())){
			if(consideredFaces==(1+numTriangles())){
				printf("Error, infinite loop detected in findTriangle2d\n");
				verbose=1;
			}
			
			printf("Examining triangle %d, with points %d,%d,%d %g away\n", ind,
				   faces[ind].points[0], faces[ind].points[1], faces[ind].points[2], 
				   dist2d(centers[ind], target));
			printf("a=[");
			vertices[faces[ind].points[0]].print();
			printf("];\nb=[");
			vertices[faces[ind].points[1]].print();
			printf("];\nc=[");
			vertices[faces[ind].points[2]].print();
			printf("];\ntri=[a;b;c;a];\ntarget=[");
			target.print();
			printf("];\n");
			printf("hold off;plot(tri(:,1), tri(:,2), 'r*-');\n");
			printf("hold on;plot(target(:,1), target(:,2), 'b*');\n");
			
			if(consideredFaces>=(20+numTriangles())){
				errorCheck();
				/*
				writeToFile("infiniteLoop.smf");
				printf("Saved infiniteLoop.smf\n");
				
				FILE *f = fopen("infiniteLoop.txt", "w");
				fprintf(f, "target point: ");
				target.print(f);
				fprintf(f, "\n");
				fclose(f);
				printf("Saved infiniteLoop.txt");
				*/
				printf("Could not find point: ");
				target.print();
				printf("\n");
				printf("Try re-triangulating\n");
				//TODO Why is this assert here?  What is the right fix??!!
				//assert(0); 
				break;
			}
		}
		
		assert(ind < numTriangles());
		int ret = contains(vertices, faces[ind], target, 0);
		assert(!ret);
		
		/* pick one of the neighbors of triangle index */
		
		/* up to 2 valid neighbors */
		int nA=-1, nB=-1;
		for(int i=0;i<3;i++){
			int neighbor = faces[ind].neighbors[i];
			if(neighbor<0 || neighbor==(int)prevIndex)continue;
			
			unsigned int edge[3];
			if(-1 == getSharedPoints(faces[ind].points,
									 faces[neighbor].points, edge)){
				printf("Error getting shared edge between %d and %d (neighbor %d)\n",
					   ind, neighbor, i);
				printf("%d points and %d faces\n", numPoints(), numTriangles());
				printf("%d has neighbors %d %d %d\n", ind,
					   faces[ind].neighbors[0],
					   faces[ind].neighbors[1],
					   faces[ind].neighbors[2]);
				printf("%d has neighbors %d %d %d\n", neighbor,
					   faces[neighbor].neighbors[0],
					   faces[neighbor].neighbors[1],
					   faces[neighbor].neighbors[2]);
				printf("Triangle %d with points %d, %d, %d\n", ind,
					   faces[ind].points[0],faces[ind].points[1],faces[ind].points[2]);
				printf("Triangle %d with points %d, %d, %d\n", neighbor,
					   faces[neighbor].points[0], faces[neighbor].points[1], faces[neighbor].points[2]);
				//TODO Why is this assert here?  What is the right fix??!!
				//assert(0); 
				return(-1);
			}
			
			if(!sameSide(target, vertices[edge[2]]/* the vertex not on the edge */, 
						 vertices[edge[0]], vertices[edge[1]])){
				
				if(verbose){
					printf("On the same side of edge defined by point %d ",edge[0]);
					vertices[edge[0]].print();
					printf(" and point %d ",edge[1]);
					vertices[edge[1]].print();
					printf("\n");
				}
				
				/* check if this neighbor contains the target point */
				if(contains(vertices, faces[neighbor], target, verbose))return neighbor;
				
				/* oh well, mark it and we'll probably evaluate it next */
				if(-1 == nA){
					nA=neighbor;
				}else{
					if(-1 != nB){
						printf("Error, the target point is across from every edge?!\n");
						printf("Target point ");
						target.print();
						printf("\n");
						printf("Center of triangle %d ", ind);
						centers[ind].print();
						printf("\n");
						for(int i=0;i<3;i++){
							printf("Corner %d of triangle %d, point %d ",
								   i, ind, faces[ind].points[i]);
							vertices[faces[ind].points[i]].print();
							printf("\n");
						}
						//TODO Why is this assert here?  What is the right fix??!!
						//assert(0); 
						return(-1);
					}
					nB=neighbor;
				}
			}
		}
		
		/* if these is no eligible neighbor, then the point is not on the mesh */
		if(nA<0){
			if(verbose)
				printf("None of the neighbors are eligible\n");
			break;
		}
		
		/* pick a neighbor to follow */
		prevIndex = ind;
		ind = (unsigned int)nA;
		
		if(nB<0){
			/* only one neighbor is valid, just follow that one */
			if(verbose)
				printf("Moved to only neighbor, triangle %d\n", ind);
			
		}else{
			/* two valid neighbors, randomly decide to pick the other one */
			if(rand()& (1<<12))
				ind = (unsigned int)nB;
			if(verbose)
				printf("Moved to random neighbor, triangle %d (chose from %d and %d)\n",
					   ind, nA, nB);
		}
	}
	
	return -1;
}

int TMAP::tmap_searchable::findTriangle2d(const RASM::point2d &target,
										  int verbose) const{
	if(verbose){
		printf("Searching for triangle containing ");
		target.print();
		printf("\n");
		
		printf("Here are the quad tree:\n");
		if(treeOfCenters)treeOfCenters->print();
		if(treeOfVertices)treeOfVertices->print();
		printf("\n");
		
		printf("Here is all the data:\n");
		print();
	}
	
	/*** center search method ***/
	assert(centers);
	assert(numTriangles()>0);
	if(verbose)
		printf("Starting the center search method (find closest center and traverse neighbors)\n");
	
	unsigned int ind = findClosestCenter(target);
	if(verbose)
		printf("Closest center is triangle %d\n", ind);
	if(ind>=numTriangles() || ind<0){
		printf("Error: searching for %g,%g, got triangle index %d of %d\n",
			   RASM_TO_METERS(target.X()), RASM_TO_METERS(target.Y()),
			   ind, numTriangles());
	}
	assert(ind < numTriangles());
	
	int ret = centerSearch(target, ind, verbose);
	if(-1 != ret)return ret;
	
	/*** end of center search method ***/
	
#if CHECK_2D_TRIANGLE
	if(verbose)
		printf("Checking all %d faces\n", numTriangles());
	
	for(unsigned int i=0;i<numTriangles();i++){
		if(contains(vertices, faces[i], target, 1)){
			printf("Error, was going to report that no triangle contained ");
			target.print();
			printf(" but triangle %d does:\n", i);
			for(int j=0;j<3;j++){
				printf(" vertex %d ", faces[i].points[j]);
				vertices[faces[i].points[j]].print();
				printf("\n");
			}
			
			assert(contains(vertices, faces[i], target, 1));
			assert(0);
		}
	}
#endif
	
	return -1;
}


RASM::point3d TMAP::tmap_searchable::findClosestSurface(const RASM::point3d &target)const{
	assert(numTriangles()>0);
	
	RASM::point3d tri[3];
	getTrianglePoints(0, tri);
	RASM::point3d bestPoint = TMAP::evaluatePoint(target, tri);
	double bestDist = distSq3d(target, bestPoint);
	
	/*
	 printf("a=[%f %f %f];b=[%f %f %f];c=[%f %f %f];tri=[a;b;c;a];\n",
	 RASM_TO_METERS(tri[0].X()), RASM_TO_METERS(tri[0].Y()), RASM_TO_METERS(tri[0].Z()),
	 RASM_TO_METERS(tri[1].X()), RASM_TO_METERS(tri[1].Y()), RASM_TO_METERS(tri[1].Z()),
	 RASM_TO_METERS(tri[2].X()), RASM_TO_METERS(tri[2].Y()), RASM_TO_METERS(tri[2].Z()));
	 printf("t=[%f %f %f];p=[%f %f %f];index=0 of %d;dist=%f;\n",
	 RASM_TO_METERS(target.X()), RASM_TO_METERS(target.Y()), RASM_TO_METERS(target.Z()),
	 RASM_TO_METERS(bestPoint.X()), RASM_TO_METERS(bestPoint.Y()), RASM_TO_METERS(bestPoint.Z()),
	 numTriangles(), bestDist);
	 printf("hold off;plot(tri(:,1), tri(:,2), 'r*-');hold on;plot(p(1,1), p(1,2), 'b*');plot(t(1,1), t(1,2), 'g*');\n");
	 */
	
	for(unsigned int i=1;i<numTriangles();i++){
		getTrianglePoints(i, tri);
		RASM::point3d p = TMAP::evaluatePoint(target, tri);
		double d = distSq3d(target, p);
		if(d<bestDist){
			bestDist = d;
			bestPoint = p;
			/*
			 printf("a=[%f %f %f];b=[%f %f %f];c=[%f %f %f];tri=[a;b;c;a];\n",
			 RASM_TO_METERS(tri[0].X()), RASM_TO_METERS(tri[0].Y()), RASM_TO_METERS(tri[0].Z()),
			 RASM_TO_METERS(tri[1].X()), RASM_TO_METERS(tri[1].Y()), RASM_TO_METERS(tri[1].Z()),
			 RASM_TO_METERS(tri[2].X()), RASM_TO_METERS(tri[2].Y()), RASM_TO_METERS(tri[2].Z()));
			 printf("p=[%f %f %f];index=%d;dist=%f;\n",
			 RASM_TO_METERS(bestPoint.X()), RASM_TO_METERS(bestPoint.Y()), RASM_TO_METERS(bestPoint.Z()),
			 i, bestDist);
			 printf("hold off;plot(tri(:,1), tri(:,2), 'r*-');hold on;plot(p(1,1), p(1,2), 'b*');plot(t(1,1), t(1,2), 'g*');\n");
			 */
		}
	}
	return bestPoint;
}

int TMAP::tmap_searchable::findSurfaceAt(const RASM::point2d &target,
										 RASM::point3d &result)const{
	assert(numTriangles()>0);
	
	int ind = findTriangle2d(target, 0);
	if(ind<0){
		return -1;
	}
	
	RASM::point3d tri[3];
	getTrianglePoints((unsigned int)ind, tri);
	
	result = RASM::point3d(target.X(), target.Y(), 
						   METERS_TO_RASM(TMAP::getHeightAt(target, tri)));
	return 0;
}

int TMAP::tmap_searchable::findSurfaceAt(const RASM::point2d &target,
										 RASM::point3d &result,
										 int &triangleIndex)const{
	assert(numTriangles()>0);
	
	if(triangleIndex<0)
		triangleIndex = findTriangle2d(target, 0);
	else{
		unsigned int ui = (unsigned int)triangleIndex;
		triangleIndex = centerSearch(target, ui, 0);
	}
	if(triangleIndex<0)return -1;
	
	RASM::point3d tri[3];
	getTrianglePoints((unsigned int)triangleIndex, tri);
	result = RASM::point3d(target.X(), target.Y(), 
						   METERS_TO_RASM(TMAP::getHeightAt(target, tri)));
	return 0;
}
