#include "plotClouds_tmap.h"
#include "plotClouds_globals.h"
#include "plotClouds_draw.h"
#include <util.h>
#include <list>

#include <stdlib.h>
#include <strings.h>

using namespace std;

class DrawableRoverState
{
public:
  translation trans;
  rotation    rot;
  double      timestamp;
  DrawableRoverState(const translation& t, const rotation& r, double ts) : trans(t), rot(r), timestamp(ts)
  {
  }
  ~DrawableRoverState() { printf("destructor called\n"); fflush(stdout); }
};

  
list<DrawableRoverState*> g_state_list;


void usage_tmap(){
  printf("\nTMAP options:\n");
  printf("  --tmapData                            subscribe to tmapData\n");
  printf("  --tmapPose                            subscribe to pose and display rover\n");
}

extern void usage(const char *name);

unsigned int parseArgs_tmap(unsigned int argc, char **argv,
			    unsigned int i){

  if(!strcasecmp("--tmapData", argv[i])){
    tmap_subscribe=true;
    return 1;
  }

  if(!strcasecmp("--tmapPose", argv[i])){
    tmap_pose=true;
    return 1;
  }

  return 0;
}

static void transferTMAP(RASM::mesh &_mesh, unsigned int ind)
{
 /* clear the old points and triangles */
  delete clouds[ind];
  delete triangleMeshes[ind];
  
  /* add the new points */
  points *p = new points();
  for(unsigned int i=0;i<_mesh.m_num_vertices;i++)
    p->add(point(RASM_TO_METERS(_mesh.m_vertices[i].X()),
		 RASM_TO_METERS(_mesh.m_vertices[i].Y()),
		 RASM_TO_METERS(_mesh.m_vertices[i].Z())));
  clouds[ind] = p;

  /* add the new triangles */
  triangles *t = new triangles();
  for(unsigned int i=0;i<_mesh.m_num_faces;i++)
    t->add(triangle(p->get( _mesh.m_faces[i].points[0] ),
		    p->get( _mesh.m_faces[i].points[1] ),
		    p->get( _mesh.m_faces[i].points[2] )));
  triangleMeshes[ind] = t;
}

static void transferTMAP()
{
  //  TMAP::tmap mesh;
  RASM::mesh _mesh;
  double dummy=0.0;
  unsigned int seq=0;

  /* check for new world model */
  if(ipc->have_new_map(RASM::MapType::WORLD))
    {
      ipc->get_map(RASM::MapType::WORLD, seq, _mesh, dummy);
      printf("Got new world model sequence %d with %d points/%d triangles\n",
	     seq, _mesh.m_num_vertices, _mesh.m_num_faces);
      transferTMAP(_mesh, tmap_index+0);
    }

   /* check for new rover path */
  if(ipc->have_new_map(RASM::MapType::PATH))
    {
      ipc->get_map(RASM::MapType::PATH, seq, _mesh, dummy);
      printf("Got new rover path sequence %d with %d points/%d triangles\n",
  	     seq, _mesh.m_num_vertices, _mesh.m_num_faces);
      //      transferTMAP(_mesh, tmap_index+1);
      transferTMAP(_mesh, RASM::MapType::PATH);
    }
  
  /* check for new selected arc */
  if(ipc->have_new_map(RASM::MapType::ARC)){
    ipc->get_map(RASM::MapType::ARC, seq, _mesh, dummy);
    printf("Got new arc sequence %d with %d points/%d triangles\n",
	   seq, _mesh.m_num_vertices, _mesh.m_num_faces);
    //    transferTMAP(_mesh, tmap_index+2);
    transferTMAP(_mesh, RASM::MapType::ARC);
  }

  /* check for goal */
  ipc->pause(0);
  RASM::goal g;	
  ipc->update_goal(g);

  printf("[%s:%d] Goal (%0.1f %0.1f) with semimajor/minor axes %0.1f, %0.1f and orientation %0.1f degrees\n",
	 __FILE__, __LINE__,
	 RASM_TO_METERS(g.m_position.X()),
	 RASM_TO_METERS(g.m_position.Y()),
	 g.m_semimajor_axis,
	 g.m_semiminor_axis,
	 g.m_orientation_radians * 180.0/M_PI);
	
  unsigned int v = 0;
  unsigned int steps = 72;
	
  RASM::mesh goal_mesh;
  goal_mesh.m_num_vertices = steps;
  goal_mesh.m_vertices = new RASM::point3d[goal_mesh.m_num_vertices];
	
  // Draw goal origin
  //	goal_mesh.m_vertices[v++].set(g.m_position.X(),g.m_position.Y(),METERS_TO_RASM(4));
  //	goal_mesh.m_vertices[v++].set(g.m_position.X(),g.m_position.Y(),METERS_TO_RASM(0));
	
  double beta = -g.m_orientation_radians; 
  double sinbeta = sin(beta);
  double cosbeta = cos(beta);
	
  double a = METERS_TO_RASM(g.m_semimajor_axis);
  double b = METERS_TO_RASM(g.m_semiminor_axis);
	
  for (int i = 0; i < 360; i += 360/steps) 
    {
      double alpha = i * (M_PI / 180) ;
      double sinalpha = sin(alpha);
      double cosalpha = cos(alpha);
		
      double px = g.m_position.X() + (a * cosalpha * cosbeta - b * sinalpha * sinbeta);
      double py = g.m_position.Y() + (a * cosalpha * sinbeta + b * sinalpha * cosbeta);
		
      goal_mesh.m_vertices[v++].set(px,py,0.0);
    }

  //printf("v=%d, num_vertices=%d\n", v, goal_mesh.m_num_vertices);
  //assert(v == goal_mesh.m_num_vertices);
	
  goal_mesh.m_num_vertices = v;
  goal_mesh.m_num_faces = 0;
  transferTMAP(goal_mesh, RASM::MapType::ARC);
}

void idle_tmap(){
  if(!tmap_subscribe && !tmap_pose)return;

  static RASM::pose pos;
  double t = 0.0;
  if(pos.isValid())t=pos.getTime();

  ipc->pause(1);

  if(tmap_subscribe)
    {
      /* check if new tmapData arrived */
      if(ipc->have_new_map(RASM::MapType::WORLD) ||
	 ipc->have_new_map(RASM::MapType::PATH) ||
	 ipc->have_new_map(RASM::MapType::ARC))
	{
	  transferTMAP();
	  glutPostRedisplay();
	}
    }

  if(tmap_pose)
    {
      /* check if new pose data arrived */
      ipc->update_pose(pos);

      if(pos.getTime() > (t+1e-5))
	{
	  glutPostRedisplay();
	}
    }
}

static void drawRover(const translation &trans,
		      const rotation &rot)
{
  /*
   * start by removing old items from the list
   */
  const double MAX_AGE_SEC = 20.0;
  list<DrawableRoverState*>::iterator it;
  for(it=g_state_list.begin(); (it != g_state_list.end()) && ((now() - (*it)->timestamp) < MAX_AGE_SEC); it++);
  g_state_list.erase(it, g_state_list.end());

  /*
   * Now add the new item
   */
  DrawableRoverState* latest_state = new DrawableRoverState(trans, rot, now());
  g_state_list.push_front(latest_state);

  /*
   * Now draw them.
   */
  for(it=g_state_list.begin(); it != g_state_list.end(); it++)
    {
      DrawableRoverState* drs = *it;

      glPushMatrix();
      
      /* apply the vehicle pose */
      drs->trans.undo();
      drs->rot.undo();
      
      /* draw the vehicle...
       * for now this is just axes with white pointing forward
       * and grey pointing right and up
       */
      drawAxes(0.5, 0.5, 0.5,  1.0, 1.0, 1.0,  0.5, 0.5, 0.5,
	       true/* force display */,
	       0.25);
      
      glPopMatrix();
    }

}

void draw_tmap(){
  if(!tmap_pose)return;

  RASM::pose pos;
  ipc->update_pose(pos);

  /* make sure there is a valid position */
  if(!pos.isValid())return;

  RASM::point3d p(pos.getPosition());
  drawRover(translation(-RASM_TO_METERS(p.X()),
			-RASM_TO_METERS(p.Y()),
			-RASM_TO_METERS(p.Z())),
	    rotation(-pos.getOrientation().roll,
		     -pos.getOrientation().pitch,
		     -pos.getOrientation().yaw));
}

// from plotClouds.cc:
extern RASM::CommsInterface* (*construct_comms_interface)(const char ** argv, int argc);

void init_tmap(const char** argv, int argc){
  if(!tmap_subscribe && !tmap_pose)return;

  printf("Initializing tmap...\n");

  //  if(IPC_isConnected()){
  //    printf("Error, already connected to IPC?!?\n");
  //    assert(0);
  //  }

  //the commons object is now populated in plotClouds.cc
  //so commenting it out here
  //assert(!ipc);
  //ipc = construct_comms_interface(argv, argc);
  
  assert(ipc);

  if(tmap_subscribe){
    ipc->subscribe_to_map();
    ipc->subscribe_to_goal_cmds();

    /* create a virtual file name */
    fileList.push_back(new loadedFile("live_tmap"));

    /* store the data as if it came from a file */
    tmap_index = clouds.size();

    /* world model at tmap_index */
    clouds.push_back(new points());
    triangleMeshes.push_back(new triangles());
    isTracks.push_back(false);

    /* rover path at tmap_index+1 */ 
    clouds.push_back(new points());
    triangleMeshes.push_back(new triangles());
    isTracks.push_back(true);

    /* selected arc at tmap_index+2 */ 
    clouds.push_back(new points());
    triangleMeshes.push_back(new triangles());
    isTracks.push_back(true);

    printf("Waiting for first tmap...\n");
    while(0 == clouds[tmap_index]->size())
      {
	if(ipc->have_new_map(RASM::MapType::WORLD) ||
	   ipc->have_new_map(RASM::MapType::PATH) ||
	   ipc->have_new_map(RASM::MapType::ARC))
	  {
	    transferTMAP();
	  }
	else
	  {
	    ipc->pause(100);
	  }
      }
  }

}
