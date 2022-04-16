/* Helper class to render shadows from a 3d model 
 * Based on the shadow volume method described in Nehe lesson 27
 *
 * Dom@cmu.edu
 */
#include "shadowModel.h"

#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <assert.h>

#define SHADOW_2PASS 0

shadowModel::shadowModel(int nVertices, int nFaces)
  :numVertices(nVertices), numFaces(nFaces){
  v = (vertex *)malloc(sizeof(vertex) * nVertices);
  f = (face *)malloc(sizeof(face) * nFaces);
  sv=NULL;
  numShadowEdges = 0;
  shadowEdgeA = shadowEdgeB = NULL;
  shadowEdgeNext = NULL;
  haveNormals = haveNeighbors = haveSource = 0;
}

shadowModel::~shadowModel(){
  free(v);
  free(f);
  if(sv)free(sv);
  if(shadowEdgeA)free(shadowEdgeA);
  if(shadowEdgeB)free(shadowEdgeB);
  if(shadowEdgeNext)free(shadowEdgeNext);
}

void shadowModel::setSource(float shadowDist, const vertex &source){
  assert(haveNormals && haveNeighbors);
  haveSource = 1;
  
  litFaces(source);
  projectSource(shadowDist, source);
  findShadowEdges();
}

void shadowModel::projectSource(float shadowDist, const vertex &source){
  if(sv)free(sv);
  sv = (vertex *)malloc(sizeof(vertex)*numVertices);
  assert(sv);
  for(int i=0;i<numVertices;i++)
    sv[i] = source + (v[i] - source)*shadowDist;
}

void shadowModel::findShadowEdges(){
  numShadowEdges = 0;
  if(shadowEdgeA)free(shadowEdgeA);
  if(shadowEdgeB)free(shadowEdgeB);
  shadowEdgeA = shadowEdgeB = NULL;

  for(int i=0; i<numFaces; i++){
    if(!f[i].lit)continue;//face i is unlit

    for(int j=0; j<3; j++){
      int n = f[i].neighbors[j];
      if ( n == -1 || !f[n].lit){
	//face n (neighbor j of face i) is unlit (or doesn't exist)

	numShadowEdges++;
	shadowEdgeA = (int *)realloc(shadowEdgeA, numShadowEdges*sizeof(int));
	shadowEdgeB = (int *)realloc(shadowEdgeB, numShadowEdges*sizeof(int));

	//face i and n share an edge defined by v1 and v2
	shadowEdgeA[numShadowEdges-1] = f[i].points[j];
	shadowEdgeB[numShadowEdges-1] = f[i].points[(j+1)%3];
      }
    }
  }

  if(shadowEdgeNext)free(shadowEdgeNext);
  shadowEdgeNext = (int *)malloc(numShadowEdges*sizeof(int));
  for(int i=0;i<numShadowEdges;i++)shadowEdgeNext[i] = i;//initially all edges are unit cycles

  //mask indicating which edges have an incoming link
  int *mask = (int *)malloc(numShadowEdges*sizeof(int));
  for(int i=0;i<numShadowEdges;i++)mask[i] = 0;

  //now optimize the shadow ordering by merging cycles
  for(int i=0;i<numShadowEdges;i++){

    //check the edge between shadowEdgeA[i] and shadowEdgeB[i]
    
    //which one should be next?
    //find another edge that contains B[i]
    int nextIndex = i;

    for(int j=1;j<numShadowEdges;j++){
      int possibleIndex = (i+j)%numShadowEdges;

      //skip masked (ie, already linked ones)
      if(mask[possibleIndex])continue;

      //check if A matches
      if(shadowEdgeB[i] == shadowEdgeA[possibleIndex]){
	nextIndex = possibleIndex;
	break;
      }

      //check if B matches (in this case swap A and B)
      if(shadowEdgeB[i] == shadowEdgeB[possibleIndex]){
	nextIndex = possibleIndex;
	shadowEdgeB[possibleIndex] = shadowEdgeA[possibleIndex];
	shadowEdgeA[possibleIndex] = shadowEdgeB[i];
	break;
      }
    }

    /* mark nextIndex as having an incoming link */
    mask[nextIndex] = 1;
    shadowEdgeNext[i] = nextIndex;
  }

  free(mask);
}

#if SHADOW_2PASS
void shadowModel::doShadowPass()const{
  //mask indicating which edges were drawn
  int *mask = (int *)malloc(numShadowEdges*sizeof(int));
  for(int i=0;i<numShadowEdges;i++)mask[i]=0;

  while(1){
    //find the first undrawn edge
    int index=0;
    while(index<numShadowEdges && mask[index])index++;
    //no more undrawn edges
    if(index>=numShadowEdges)break;

    //starting at index, keep drawing edges until reaching a drawn one
    glBegin(GL_QUAD_STRIP);
    v[shadowEdgeA[index]].glVertex();
    sv[shadowEdgeA[index]].glVertex();
    do{
      v[shadowEdgeB[index]].glVertex();
      sv[shadowEdgeB[index]].glVertex();

      //mark this one as drawn and advance to the next
      mask[index] = 1;
      index = shadowEdgeNext[index];
    }while(!mask[index]);
    glEnd();
  }
  free(mask);
}
#else
void shadowModel::doShadowPass()const{
  for(int i=0;i<numFaces;i++){

    if(!f[i].lit)
      glStencilOp( GL_KEEP, GL_KEEP, GL_INCR );
    else
      glStencilOp( GL_KEEP, GL_KEEP, GL_DECR );
    //glStencilOp( GL_KEEP, GL_KEEP, GL_INVERT );

    glBegin(GL_QUAD_STRIP);
    for(int i=0;i<3;i++){
      int index = f[i].points[i];
      v[index].glVertex();
      sv[index].glVertex();
    }
    glEnd();

  }
}
#endif

void shadowModel::computeNormals(int upward){
  assert(!haveNormals);
  haveNormals = 1;

  for(int i=0;i<numFaces;i++){
    //get the cross product of two edges
    vertex ba = v[ f[i].points[1] ] - v[ f[i].points[0] ];
    vertex ca = v[ f[i].points[2] ] - v[ f[i].points[0] ];
    vertex n = ba.cross(ca);

    if(upward && n.z<0){
      //use the upward face by flipping the normal
      n *= -1.0;

      //swap vertices 0 and 1 to keep consistent handedness
      int swap = f[i].points[0];
      f[i].points[0] = f[i].points[1];
      f[i].points[1] = swap;

      //also swap neighbors 1 and 2
      swap = f[i].neighbors[1];
      f[i].neighbors[1] = f[i].neighbors[2];
      f[i].neighbors[2] = swap;
    }
    f[i].normal = n;
  }
}

void shadowModel::computeNeighbors(){
  assert(!haveNeighbors);
  haveNeighbors = 1;

  /* clear all neighbors */
  for(int i=0;i<numFaces;i++)
    f[i].neighbors[0] = f[i].neighbors[1] = f[i].neighbors[2] = -1;

  /* for each point make room for N faces that use it 
   * (note that we can go over N on any given face, but not in total)
   */
  const int N = 10;
  int *edges = (int *)malloc(N * numVertices *sizeof(int));
  assert(edges);
  for(int i=0;i<N*numVertices;i++)
    edges[i] = -1;

  /*** first mark in the edges array all faces ***/
  for(int j=0;j<numFaces;j++){//process face j
    for(int i=0;i<3;i++){//process point p (i of face j)
      int p = (f[j].points[i]);

      //mark that face j uses point p 
      //(by storing it at an empty index near N*p)
      int index = N * p;

      while(-1 != edges[index])
	index = (index+1)%( N*numVertices );
      edges[index] = j;
    }
  }

  /*** now uses the edges array to find matches ***/
  for(int j=0;j<numFaces;j++){//process face j
    for(int i=0;i<3;i++){//process edge i of face j
      //edge i is defined by points p1 and p2
      int p1 = f[j].points[i];
      int p2 = f[j].points[(i+1)%3];

      assert(p1>=0 && p1<numVertices);
      assert(p2>=0 && p2<numVertices);

      for(int index=p1*N;                       //index into edges of first possible neighbor
	  -1 == f[j].neighbors[i];              //while the neighbor is unknown
	  index = (index+1)%( N*numVertices )){ //next candidate
	//compare against face f2
	int f2 = edges[index];

	//skip current face
	if(j==f2)continue;

	//no more faces to check
	if(-1 == f2)break;

	for(int k=0;k<3;k++){//check edge j of face f2
	  //edge k is defined by points p3 and p4
	  int p3 = f[f2].points[k];
	  int p4 = f[f2].points[(k+1)%3];

	  //is edge i of face j the same as edge k of face f2?
	  if((p1==p3 && p2==p4) || (p1==p4 && p2==p3)){
	    f[j].neighbors[i] = f2;
	    f[f2].neighbors[k] = j;
	  }
	}
      }
    }
  }

  /*** cleanup ***/
  free(edges);
}

void shadowModel::litFaces(const vertex &source){
  /* determine which faces are lit by taking the dot product of the light vector and surface normal */
  for(int i=0;i<numFaces;i++){
    vertex light = source - v[ f[i].points[0] ];
    f[i].lit = ((f[i].normal * light)>0);
  }
}

void shadowModel::shadow(float nearPlane) const{
  assert(haveNormals && haveNeighbors && haveSource);

  glPushAttrib( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_POLYGON_BIT | GL_STENCIL_BUFFER_BIT );
  glDisable( GL_LIGHTING );				// Turn Off Lighting
  glDepthMask( GL_FALSE );				// Turn Off Writing To The Depth-Buffer
  glEnable( GL_STENCIL_TEST );				// Turn On Stencil Buffer Testing
  glColorMask( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE );// Don't Draw Into The Colour Buffer
  glStencilFunc( GL_ALWAYS, 1, 0xFFFFFFFFL );

#if SHADOW_2PASS
  glCullFace(GL_BACK);   // Set Culling Face To Back Face
  glEnable(GL_CULL_FACE);// Enable Culling

  // First Pass. Increase Stencil Value In The Shadow
  glFrontFace( GL_CCW );
  glStencilOp( GL_KEEP, GL_KEEP, GL_INCR );
  doShadowPass();

  // Second Pass. Decrease Stencil Value In The Shadow
  glFrontFace( GL_CW );
  glStencilOp( GL_KEEP, GL_KEEP, GL_DECR );
  doShadowPass();

  glStencilFunc( GL_NOTEQUAL, 0, 0xFFFFFFFFL );
#else
  doShadowPass();

  glStencilFunc( GL_GREATER, 3, 0xFFFFFFFFL );
#endif

  glFrontFace( GL_CCW );
  glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );	// Enable Rendering To Colour Buffer For All Components

  // Draw A Shadowing Rectangle Covering The Entire Screen
  glEnable( GL_BLEND );
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );
  glPushMatrix();
  glLoadIdentity();
  glBegin( GL_TRIANGLE_STRIP );
  float v = nearPlane*1.01;
  glVertex3f(-1, 1,-v);
  glVertex3f(-1,-1,-v);
  glVertex3f( 1, 1,-v);
  glVertex3f( 1,-1,-v);
  glEnd();
  glPopMatrix();
  glPopAttrib();
}
/*** end of shadowModel definitions ***/
