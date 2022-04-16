/**
 * @file path_model.h
 *
 * @section LICENSE
 * Copyright 2012, Carnegie Mellon University and ProtoInnovations, LLC. 
 * All rights reserved. This document contains information that is proprietary
 * to Carnegie Mellon University and ProtoInnovations, LLC. Do not distribute
 * without approval.
 *
 */

#ifndef __PATH_MODEL_H__
#define __PATH_MODEL_H__

#include <rasm_common_types.h>
#include <rasm_pose.h>
#include <stdint.h>
#include <stdlib.h>
#include <map>
#include <string>

class PathModel
{
 public:
  //virtual void update_path_set(bool use_reverse_paths=false) = 0;
  virtual bool update_path_set(bool use_reverse_paths=false) = 0;

  uint32_t m_path_set_size;

  double* m_path_risk_factor; /* ==1: planner cost unaffected
			       * > 1: planner cost increased (path avoided)
			       * < 1: planner cost decreased (path preferrred)
			       */

  uint32_t* m_num_path_points;

  RASM::point3d** m_front_left_wheel_paths_body;
  RASM::point3d** m_front_right_wheel_paths_body;
  RASM::point3d** m_rear_left_wheel_paths_body;
  RASM::point3d** m_rear_right_wheel_paths_body;

  RASM::point3d** m_front_left_wheel_paths;
  RASM::point3d** m_front_right_wheel_paths;
  RASM::point3d** m_rear_left_wheel_paths;
  RASM::point3d** m_rear_right_wheel_paths;

  static void applyTransformationMatrix(const float M[4][4], 
					RASM::point3d &in,
					RASM::point3d &out)
  {
    for(uint8_t i=0; i < 3; i++)
      {
	out.coord3d[i] = (RASM_UNITS)(M[i][0]*(float)in.X() + M[i][1]*(float)in.Y() + M[i][2]*(float)in.Z() + M[i][3]);
      }

  } // applyTransformMatrix
  

  /*
   * grab points from original, body-frame source, and then store results in
   * m_*_wheel_paths. Note that there will be repeated calls to this method so
   * each call must start from a body frame source.
   */
  void translateAndRotate(RASM::pose& p)
  {
    assert(NULL != m_num_path_points);
    assert(NULL != m_front_left_wheel_paths_body);
    assert(NULL != m_front_left_wheel_paths);
    assert(NULL != m_front_right_wheel_paths_body);
    assert(NULL != m_front_right_wheel_paths);
    assert(NULL != m_rear_left_wheel_paths_body);
    assert(NULL != m_rear_left_wheel_paths);
    assert(NULL != m_rear_right_wheel_paths_body);
    assert(NULL != m_rear_right_wheel_paths);

    RASM::point3d pos = p.getPosition();
    double roll  = p.getOrientation().roll;
    double pitch = p.getOrientation().pitch;
    double yaw   = p.getOrientation().yaw;
    float M[4][4];

    /* initialize and fill in the rotation */
    rotationMatrix(roll, pitch, yaw, M);
    
    /* fill in the translation */
    for(uint8_t i=0; i < 3; i++)
      {
	M[i][3] = pos.coord3d[i];
      }

    /* translate from *_paths_body to *_paths */
    for(uint32_t i = 0; i < m_path_set_size; i++)
      {
	assert(NULL != m_front_left_wheel_paths_body[i]);
	assert(NULL != m_front_left_wheel_paths[i]);
	assert(NULL != m_front_right_wheel_paths_body[i]);
	assert(NULL != m_front_right_wheel_paths[i]);
	assert(NULL != m_rear_left_wheel_paths_body[i]);
	assert(NULL != m_rear_left_wheel_paths[i]);
	assert(NULL != m_rear_right_wheel_paths_body[i]);
	assert(NULL != m_rear_right_wheel_paths[i]);

	for(uint32_t j = 0; j < m_num_path_points[i]; j++)
	  {    
	    applyTransformationMatrix(M, 
				      m_front_left_wheel_paths_body[i][j],
				      m_front_left_wheel_paths[i][j]);
	    applyTransformationMatrix(M, 
				      m_front_right_wheel_paths_body[i][j],
				      m_front_right_wheel_paths[i][j]);
	    applyTransformationMatrix(M, 
				      m_rear_left_wheel_paths_body[i][j],
				      m_rear_left_wheel_paths[i][j]);
	    applyTransformationMatrix(M, 
				      m_rear_right_wheel_paths_body[i][j],
				      m_rear_right_wheel_paths[i][j]);

	    /* printf("[%s:%d] %0.3f --> %0.3f %0.3f --> %0.3f %0.3f --> %0.3f %0.3f --> %0.3f \n", __FILE__, __LINE__, */
	    /* 	   RASM_TO_METERS(m_front_left_wheel_paths_body[i][j].coord3d[0]), */
	    /* 	   RASM_TO_METERS(m_front_left_wheel_paths[i][j].coord3d[0]), */
	    /* 	   RASM_TO_METERS(m_front_right_wheel_paths_body[i][j].coord3d[0]), */
	    /* 	   RASM_TO_METERS(m_front_right_wheel_paths[i][j].coord3d[0]), */
	    /* 	   RASM_TO_METERS(m_rear_left_wheel_paths_body[i][j].coord3d[0]), */
	    /* 	   RASM_TO_METERS(m_rear_left_wheel_paths[i][j].coord3d[0]), */
	    /* 	   RASM_TO_METERS(m_rear_right_wheel_paths_body[i][j].coord3d[0]), */
	    /* 	   RASM_TO_METERS(m_rear_right_wheel_paths[i][j].coord3d[0])); */
	    
	    /* fflush(stdout); */

	  } // for each point in path index i
	
      } // for each path in the set

  } // translateAndRotate()

  //  double *m_paths;
  //  unsigned int m_set_size;
  /* RASM::point3d *m_vertices; */
  /* RASM::triangle *m_faces; */
  /* unsigned int m_size_vertices, m_capacity_vertices; */
  /* unsigned int m_size_faces, m_capacity_faces; */
  /* unsigned int m_num_steps_to_skip; */

  /* virtual void set(unsigned int path_index,  */
  /* 		   float length,  */
  /* 		   float lengthResolution,  */
  /* 		   bool forward, */
  /* 		   float max_heading_change,  */
  /* 		   float headingResolution,  */
  /* 		   float wheelDist,  */
  /* 		   float roverLength,  */
  /* 		   unsigned int pointsPerPath)=0; */

  /* unsigned int inline getPathsCount() {return m_set_size;}; */
  /* unsigned int inline getAllPathsCount() { return 2*m_set_size;}; */

  PathModel(std::map<std::string, std::string>& config) : 
    m_path_set_size(0),
    m_path_risk_factor(NULL),
    m_num_path_points(NULL),
    m_front_left_wheel_paths_body(NULL),
    m_front_right_wheel_paths_body(NULL),
    m_rear_left_wheel_paths_body(NULL),
    m_rear_right_wheel_paths_body(NULL),
    m_front_left_wheel_paths(NULL),
    m_front_right_wheel_paths(NULL),
    m_rear_left_wheel_paths(NULL),
    m_rear_right_wheel_paths(NULL)
    {
    }

  ~PathModel() 
    {
      if(NULL != m_path_risk_factor)
	{
	  delete m_path_risk_factor;
	  m_path_risk_factor = NULL;
	}

      if(NULL != m_num_path_points)
	{
	  delete m_num_path_points;
	  m_num_path_points = 0;
	}

      if(NULL != m_front_left_wheel_paths_body)
	{
	  delete[] m_front_left_wheel_paths_body;
	  m_front_left_wheel_paths_body = NULL;
	}
      if(NULL != m_front_left_wheel_paths)
	{
	  delete[] m_front_left_wheel_paths;
	  m_front_left_wheel_paths = NULL;
	}

      if(NULL != m_front_right_wheel_paths_body)
	{
	  delete[] m_front_right_wheel_paths_body;
	  m_front_right_wheel_paths_body = NULL;
	}
      if(NULL != m_front_right_wheel_paths)
	{
	  delete[] m_front_right_wheel_paths;
	  m_front_right_wheel_paths = NULL;
	}

      if(NULL != m_rear_left_wheel_paths_body)
	{
	  delete[] m_rear_left_wheel_paths_body;
	  m_rear_left_wheel_paths_body = NULL;
	}
      if(NULL != m_rear_left_wheel_paths)
	{
	  delete[] m_rear_left_wheel_paths;
	  m_rear_left_wheel_paths = NULL;
	}

      if(NULL != m_rear_right_wheel_paths_body)
	{
	  delete[] m_rear_right_wheel_paths_body;
	  m_rear_right_wheel_paths_body = NULL;
	}
      if(NULL != m_rear_right_wheel_paths)
	{
	  delete[] m_rear_right_wheel_paths;
	  m_rear_right_wheel_paths = NULL;
	}

    };

  void save_paths(std::string prefix_filename)
  {
    std::string front_left_filename  = prefix_filename + ".front_left.obj";
    std::string front_right_filename = prefix_filename + ".front_right.obj";
    std::string rear_left_filename   = prefix_filename + ".rear_left.obj";
    std::string rear_right_filename  = prefix_filename + ".rear_right.obj";
    printf("[%s, %d]: Saving arc model to files: %s, %s, %s, %s\n", __FILE__,__LINE__, front_left_filename, front_right_filename, rear_left_filename, rear_right_filename);

    FILE* fl = fopen(front_left_filename.c_str(),  "w");
    FILE* fr = fopen(front_right_filename.c_str(), "w");
    FILE* rl = fopen(rear_left_filename.c_str(),   "w");
    FILE* rr = fopen(rear_right_filename.c_str(),  "w");

    if(NULL == fl) return;
    if(NULL == fr) return;
    if(NULL == rl) return;
    if(NULL == rr) return;

    for(uint32_t i=0; i < m_path_set_size; i++)
      {
	for(uint32_t j=0; j < m_num_path_points[i]; j++)
	  {
	    fprintf(fl, "v %0.3f %0.3f %0.3f\n",
		    RASM_TO_METERS(m_front_left_wheel_paths_body[i][j].coord3d[0]),
		    RASM_TO_METERS(m_front_left_wheel_paths_body[i][j].coord3d[1]),
		    RASM_TO_METERS(m_front_left_wheel_paths_body[i][j].coord3d[2]));
	    fprintf(fr, "v %0.3f %0.3f %0.3f\n",
		    RASM_TO_METERS(m_front_right_wheel_paths_body[i][j].coord3d[0]),
		    RASM_TO_METERS(m_front_right_wheel_paths_body[i][j].coord3d[1]),
		    RASM_TO_METERS(m_front_right_wheel_paths_body[i][j].coord3d[2]));
	    fprintf(rl, "v %0.3f %0.3f %0.3f\n",
		    RASM_TO_METERS(m_rear_left_wheel_paths_body[i][j].coord3d[0]),
		    RASM_TO_METERS(m_rear_left_wheel_paths_body[i][j].coord3d[1]),
		    RASM_TO_METERS(m_rear_left_wheel_paths_body[i][j].coord3d[2]));
	    fprintf(rr, "v %0.3f %0.3f %0.3f\n",
		    RASM_TO_METERS(m_rear_right_wheel_paths_body[i][j].coord3d[0]),
		    RASM_TO_METERS(m_rear_right_wheel_paths_body[i][j].coord3d[1]),
		    RASM_TO_METERS(m_rear_right_wheel_paths_body[i][j].coord3d[2]));
	  }
      }

    fclose(fl);
    fclose(fr);
    fclose(rl);
    fclose(rr);

  } // save_paths

};

#endif /* __POINT_MODEL__H_ */
