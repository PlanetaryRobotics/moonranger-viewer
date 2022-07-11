#include "moonranger_arc_model.h"
#include <assert.h>
#include <float.h>
#include <util.h>
#include <list>

/*!
 * 12/05/2020:
 * Herein are the definitions for MoonRanger's arc model (used by the planner
 * for arc evaluation across a terrain mesh). Note that the following functions
 * are called within rasmEval.cc--the file which defines the planner's primary
 * functionalities.
 *
 * The majority of the proceeding calculations stem from the zoe_arc_model
 * interface that is found in the /interfaces directory. Correspondingly, the
 * proceeding calculations are for the drive arcs of Zoe's Ackermann steering
 * model. MoonRanger is a skid-steered rover; these calculations must be
 * updated to fit such an arc model.
 */

/*!
 * A helper function to deduce if a number is positive or negative:
 */
static double sign(double x)
{
  if(x >= 0)
    {
      return 1.0;
    }
  else
    {
      return -1.0;
    }
}

static const uint32_t STRAIGHT_ARC_RADIUS_METERS = 1000;

/*!
 * Old developers' comment:
 * TBD: do we really want to create arcs this way?
 * Potential problems:
 * - total_distance_along_arc may be too big for small-radius arcs.
 */

/*!
 *@function create_arcs currently functions primarily
 * to "fill" the "*_*_wheel_paths_body" variables with
 * xy-coordinates of each wheel.
 */
static void create_arcs(double total_distance_along_arc_meters,
			uint32_t num_path_points,
			RASM::point3d* front_left_wheel_paths_body,
			RASM::point3d* front_right_wheel_paths_body,
			RASM::point3d* rear_left_wheel_paths_body,
			RASM::point3d* rear_right_wheel_paths_body,
			double radius_meters,
			double rover_width_meters,
			double rover_length_meters,
			bool forward,
			double reverse_path_length_scaling_factor)
{
  assert(NULL != front_left_wheel_paths_body);
  assert(NULL != front_right_wheel_paths_body);
  assert(NULL != rear_left_wheel_paths_body);
  assert(NULL != rear_right_wheel_paths_body);
  assert(rover_width_meters > 0);
  assert(rover_length_meters > 0);
  assert(fabs(radius_meters) > FLT_EPSILON);
  assert(num_path_points > 0);

  if(fabs(radius_meters) >= STRAIGHT_ARC_RADIUS_METERS) radius_meters = FLT_MAX;
  /*!
   * Typically, we will make reverse arcs shorter than forward arcs:
   */
  if(!forward) total_distance_along_arc_meters = total_distance_along_arc_meters * reverse_path_length_scaling_factor;

  double total_angle_rad = fabs(total_distance_along_arc_meters / radius_meters);
  double delta_angle_rad = fabs(total_angle_rad / num_path_points);

  // printf("[%s:%d %f] total_distance_along_arc %f, creating arc with %0.3f m radius, rover width: %0.3f m, length: %0.3f m, total arc %0.3f deg\n", 
  // 	 __FILE__, __LINE__, now(),
  // 	 total_distance_along_arc_meters,
  // 	 radius_meters, rover_width_meters, rover_length_meters,
  // 	 total_angle_rad * 180.0 / M_PI);
  // fflush(stdout);

  /*
   * Calculate coordinates of arcs:
   */
  for(uint32_t i=0; i < num_path_points; i++)
    {
      int64_t signed_i = sign(total_distance_along_arc_meters) * i;

      /*
       * Calculate front-wheel arcs:
       */
      double front_current_angle_rad = signed_i * delta_angle_rad + (rover_length_meters/2.0) / fabs(radius_meters); //Rover has moved delta-rads + half its length

      double front_axle_center_x_meters = sign(radius_meters) * (cos(front_current_angle_rad) * fabs(radius_meters) - fabs(radius_meters)); //sign() returns JUST 1 or -1
      double front_axle_center_y_meters = (forward ? 1.0 : -1.0) * sin(front_current_angle_rad) * fabs(radius_meters);

      double front_dx = cos(front_current_angle_rad) * sign(radius_meters);
      double front_dy = (forward ? 1.0 : -1.0) * sin(-front_current_angle_rad);

      RASM_UNITS x_inner_front = METERS_TO_RASM(front_axle_center_x_meters - front_dx * rover_width_meters/2.0);
      RASM_UNITS y_inner_front = METERS_TO_RASM(front_axle_center_y_meters + front_dy * rover_width_meters/2.0);
      RASM_UNITS x_outer_front = METERS_TO_RASM(front_axle_center_x_meters + front_dx * rover_width_meters/2.0);
      RASM_UNITS y_outer_front = METERS_TO_RASM(front_axle_center_y_meters - front_dy * rover_width_meters/2.0);

      /*
       * Calculate rear-wheel arcs:
       */
      double rear_current_angle_rad = signed_i * delta_angle_rad - (rover_length_meters/2.0) / fabs(radius_meters);

      double rear_axle_center_x_meters = sign(radius_meters) * (cos(rear_current_angle_rad) * fabs(radius_meters) - fabs(radius_meters));
      double rear_axle_center_y_meters = (forward ? 1.0 : -1.0) * sin(rear_current_angle_rad) * fabs(radius_meters);

      double rear_dx = cos(rear_current_angle_rad) * sign(radius_meters);
      double rear_dy = (forward ? 1.0 : -1.0) * sin(-rear_current_angle_rad);

      RASM_UNITS x_inner_rear = METERS_TO_RASM(rear_axle_center_x_meters - rear_dx * rover_width_meters/2.0);
      RASM_UNITS y_inner_rear = METERS_TO_RASM(rear_axle_center_y_meters + rear_dy * rover_width_meters/2.0);
      RASM_UNITS x_outer_rear = METERS_TO_RASM(rear_axle_center_x_meters + rear_dx * rover_width_meters/2.0);
      RASM_UNITS y_outer_rear = METERS_TO_RASM(rear_axle_center_y_meters - rear_dy * rover_width_meters/2.0);

      if(radius_meters > 0)
	{
	  front_left_wheel_paths_body[i].coord3d[0]  = forward ? x_inner_front : x_inner_rear;
	  front_left_wheel_paths_body[i].coord3d[1]  = forward ? y_inner_front : y_inner_rear;

	  front_right_wheel_paths_body[i].coord3d[0] = forward ? x_outer_front : x_outer_rear;
	  front_right_wheel_paths_body[i].coord3d[1] = forward ? y_outer_front : y_outer_rear;

	  rear_left_wheel_paths_body[i].coord3d[0]   = forward ? x_inner_rear : x_inner_front;
	  rear_left_wheel_paths_body[i].coord3d[1]   = forward ? y_inner_rear : y_inner_front;

	  rear_right_wheel_paths_body[i].coord3d[0]  = forward ? x_outer_rear : x_outer_front;
	  rear_right_wheel_paths_body[i].coord3d[1]  = forward ? y_outer_rear : y_outer_front;
	}
      else
	{
	  front_left_wheel_paths_body[i].coord3d[0]  = forward ? x_outer_front : x_outer_rear;
	  front_left_wheel_paths_body[i].coord3d[1]  = forward ? y_outer_front : y_outer_rear;

	  front_right_wheel_paths_body[i].coord3d[0] = forward ? x_inner_front : x_inner_rear;
	  front_right_wheel_paths_body[i].coord3d[1] = forward ? y_inner_front : y_inner_rear;

	  rear_left_wheel_paths_body[i].coord3d[0]   = forward ? x_outer_rear : x_outer_front;
	  rear_left_wheel_paths_body[i].coord3d[1]   = forward ? y_outer_rear : y_outer_front;

	  rear_right_wheel_paths_body[i].coord3d[0]  = forward ? x_inner_rear : x_inner_front;
	  rear_right_wheel_paths_body[i].coord3d[1]  = forward ? y_inner_rear : y_inner_front;
	}

      /*!
       * Fill the z-value with zeroes:
       */
      front_left_wheel_paths_body[i].coord3d[2]  = 0;
      front_right_wheel_paths_body[i].coord3d[2] = 0;
      rear_left_wheel_paths_body[i].coord3d[2]   = 0;
      rear_right_wheel_paths_body[i].coord3d[2]  = 0;

    } // for each point in the path

} // create_arcs


/*!
 * MoonrangerArcModel constructor:
 */
MoonrangerArcModel::MoonrangerArcModel(std::map<std::string, std::string>& config) :
  PathModel(config),
  m_radii_fwd(),
  m_radii_rev(),
  m_total_distance_along_arc_meters(0.0),
  m_rover_length_meters(0.0),
  m_rover_width_meters(0.0),
  m_num_points_per_arc(0),
  m_reverse_path_length_scaling_factor(1.0),
  m_max_speed_meters_per_sec(0.0),
  m_reverse_speed_multiplier(0.0),
  m_reverse_path_risk_factor(1.0)
{

  if(config.find("total_distance_along_arc_meters") != config.end()) m_total_distance_along_arc_meters = atof(config["total_distance_along_arc_meters"].c_str());
  if(config.find("rover_length_meters")             != config.end()) m_rover_length_meters             = atof(config["rover_length_meters"].c_str());
  if(config.find("rover_width_meters")              != config.end()) m_rover_width_meters              = atof(config["rover_width_meters"].c_str());
  if(config.find("num_points_per_arc")              != config.end()) m_num_points_per_arc              = atoi(config["num_points_per_arc"].c_str());
  if(config.find("reverse_path_length_scaling_factor") != config.end()) m_reverse_path_length_scaling_factor = atof(config["reverse_path_length_scaling_factor"].c_str());
  if(config.find("max_speed_meters_per_sec")        != config.end()) m_max_speed_meters_per_sec        = atof(config["max_speed_meters_per_sec"].c_str());
  if(config.find("reverse_speed_multiplier")        != config.end()) m_reverse_speed_multiplier        = atof(config["reverse_speed_multiplier"].c_str());
  if(config.find("reverse_path_risk_factor")        != config.end()) m_reverse_path_risk_factor        = atof(config["reverse_path_risk_factor"].c_str());

  assert(m_rover_length_meters > 0.);
  assert(m_rover_width_meters > 0.);
  assert(m_num_points_per_arc > 0);
  assert( (m_reverse_path_length_scaling_factor > 0.) && (m_reverse_path_length_scaling_factor <= 1.0) );
  assert(m_max_speed_meters_per_sec > 0.);
  assert((m_reverse_speed_multiplier > 0.) && (m_reverse_speed_multiplier <= 1.0));
  //assert(m_reverse_path_risk_factor >= 1.0);

  uint32_t i = 0;
  while(1)
  {
    const uint8_t MAX_STRING_SIZE = 255;
    char c_str[MAX_STRING_SIZE];
    sprintf(c_str, "radius_%d_meters", i);
    std::string str(c_str);
    if(config.find(str) != config.end())
    {
      char sp_str[MAX_STRING_SIZE];
      sprintf(sp_str, "speed_%d_meters_per_sec", i);
      std::string speed_str(sp_str);
      if(config.find(speed_str) != config.end()) //If arcs' speeds are enumerated in a config file:
      {                                            /*radius                 "forward"     speed*/
        m_radii_fwd.push_back(new MoonrangerArc(atof(config[str].c_str()),   true,        atof(config[speed_str].c_str())));  // forward arc
        m_radii_rev.push_back(new MoonrangerArc(atof(config[str].c_str()),   false,       m_reverse_speed_multiplier * atof(config[speed_str].c_str()))); // slow, reverse arc
      }
      else //Fill arc model according to max speed
      {
        m_radii_fwd.push_back(new MoonrangerArc(atof(config[str].c_str()),   true,        m_max_speed_meters_per_sec));
        m_radii_rev.push_back(new MoonrangerArc(atof(config[str].c_str()),   false,       m_reverse_speed_multiplier * m_max_speed_meters_per_sec));
      }
    }
    else
    {
      break; // no more consecutive radii found, so break out of loop
    }
    i++;
  }

  printf("[%s:%d %f] read %lu arcs\n", __FILE__, __LINE__, now(),
	 m_radii_fwd.size());

  m_path_set_size = m_radii_fwd.size();
  assert(m_path_set_size > 0);

  m_path_risk_factor = new double[m_path_set_size];

  m_num_path_points = new uint32_t[m_path_set_size];

  m_front_left_wheel_paths_body  = new RASM::point3d*[m_path_set_size];
  m_front_right_wheel_paths_body = new RASM::point3d*[m_path_set_size];
  m_rear_left_wheel_paths_body   = new RASM::point3d*[m_path_set_size];
  m_rear_right_wheel_paths_body  = new RASM::point3d*[m_path_set_size];
  m_front_left_wheel_paths       = new RASM::point3d*[m_path_set_size];
  m_front_right_wheel_paths      = new RASM::point3d*[m_path_set_size];
  m_rear_left_wheel_paths        = new RASM::point3d*[m_path_set_size];
  m_rear_right_wheel_paths       = new RASM::point3d*[m_path_set_size];

  unsigned int counter = 0;
  /*!
   * Constructor's current default is to fill m_*_*_wheel_paths_body
   * with forward arcs:
   */
  m_use_reverse_arcs = false;
  for (std::list<MoonrangerArc*>::iterator i = m_radii_fwd.begin(); i != m_radii_fwd.end(); i++)
  {
    MoonrangerArc* arc   = *i;
    double radius_meters = arc->m_radius_meters;
    bool   forward       = arc->m_forward;
    double speed         = arc->m_speed;

    /*
     * Can't handle point turns yet
     */
    assert(fabs(radius_meters) > FLT_EPSILON);

    /*
     * Set path risk based on whether the path goes forward or not
     */
    if(forward)
    {
      m_path_risk_factor[counter] = 1.0; // cost unchanged
    }
    else
    {
      m_path_risk_factor[counter] = m_reverse_path_risk_factor;
    }

    m_num_path_points[counter] = m_num_points_per_arc;

    m_front_left_wheel_paths_body[counter]  = new RASM::point3d[m_num_path_points[counter]];
    m_front_right_wheel_paths_body[counter] = new RASM::point3d[m_num_path_points[counter]];
    m_rear_left_wheel_paths_body[counter]   = new RASM::point3d[m_num_path_points[counter]];
    m_rear_right_wheel_paths_body[counter]  = new RASM::point3d[m_num_path_points[counter]];

    create_arcs(m_total_distance_along_arc_meters,
      m_num_path_points[counter],
      m_front_left_wheel_paths_body[counter],
      m_front_right_wheel_paths_body[counter],
      m_rear_left_wheel_paths_body[counter],
      m_rear_right_wheel_paths_body[counter],
      radius_meters,
      m_rover_width_meters,
      m_rover_length_meters,
      forward,
      m_reverse_path_length_scaling_factor);
    /*
     * Initialize these to match the body-frame arcs. These will be transformed
     * by later calls to PathModel::translateAndRotate()
     */
    m_front_left_wheel_paths[counter]  = new RASM::point3d[m_num_path_points[counter]];
    m_front_right_wheel_paths[counter] = new RASM::point3d[m_num_path_points[counter]];
    m_rear_left_wheel_paths[counter]   = new RASM::point3d[m_num_path_points[counter]];
    m_rear_right_wheel_paths[counter]  = new RASM::point3d[m_num_path_points[counter]];

    printf("[%s:%d %f] radius[%d] = %f, direction = %s, speed = %f\n",
     __FILE__, __LINE__, now(),
     counter,
     radius_meters, forward ? "forward" : "reverse", speed);
    counter++;

  } // for each radius

  /*!
   * If we want to test the kinematic model, save the paths:
   */
   // save_paths("saved_moonranger_arcs");
  /*!
   * Saved paths should appear in your .ros directory.
   */

} // MoonrangerArcModel::MoonrangerArcModel()


/*!
 * MoonrangerArcModel destructor:
 */
MoonrangerArcModel::~MoonrangerArcModel()
{
  for (std::list<MoonrangerArc*>::iterator i = m_radii_fwd.begin(); i != m_radii_fwd.end(); i++)
  {
    MoonrangerArc* arc = *i;
    delete arc;
  }
  for (std::list<MoonrangerArc*>::iterator i = m_radii_rev.begin(); i != m_radii_rev.end(); i++)
  {
    MoonrangerArc* arc = *i;
    delete arc;
  }
}

/*!
 * This function reinstantiates a MoonrangerArcModel to be comprised of
 * either forward or reverse arcs (depending on what the planner specifies).
 * It is the only moonranger_arc_model function called by the planner, and it
 * incorporates the functionality defined above.
 */
bool MoonrangerArcModel::update_path_set(bool use_reverse_arcs)
{
  /*!
   * If we're already heading in the desired direction,
   * skip unnecessary processing and return
   */
  if (m_use_reverse_arcs == use_reverse_arcs) return false;
  else m_use_reverse_arcs = use_reverse_arcs;

  assert(m_path_set_size > 0);

  uint32_t i=0;

  /*!
   * If we need to re-create_arcs, do so according to whether or not
   * path evaluation is looking forward or backward. If forward:
   */
  if (m_use_reverse_arcs == false) {
    for (std::list<MoonrangerArc*>::iterator iter = m_radii_fwd.begin(); iter != m_radii_fwd.end(); iter++)
      {

        MoonrangerArc* arc = *iter;
        double radius_meters = arc->m_radius_meters;

        assert(fabs(radius_meters) > FLT_EPSILON);

        create_arcs(m_total_distance_along_arc_meters,
              m_num_path_points[i],
              m_front_left_wheel_paths_body[i],
              m_front_right_wheel_paths_body[i],
              m_rear_left_wheel_paths_body[i],
              m_rear_right_wheel_paths_body[i],
              arc->m_radius_meters,
              m_rover_width_meters,
              m_rover_length_meters,
              arc->m_forward,
              m_reverse_path_length_scaling_factor);

        i++; // update 'i' to keep array indices incrementing...

    } // for each forward radius
  }
  else { //If we're instead evaluating the terrain behind us:
    for (std::list<MoonrangerArc*>::iterator iter = m_radii_rev.begin(); iter != m_radii_rev.end(); iter++)
      {

        MoonrangerArc* arc = *iter;
        double radius_meters = arc->m_radius_meters;

        assert(fabs(radius_meters) > FLT_EPSILON);

        create_arcs(m_total_distance_along_arc_meters,
              m_num_path_points[i],
              m_front_left_wheel_paths_body[i],
              m_front_right_wheel_paths_body[i],
              m_rear_left_wheel_paths_body[i],
              m_rear_right_wheel_paths_body[i],
              arc->m_radius_meters,
              m_rover_width_meters,
              m_rover_length_meters,
              arc->m_forward,
              m_reverse_path_length_scaling_factor);

        i++; // update 'i' to keep array indices incrementing...

    } // for each reverse radius
  }
  return true;
} // update_path_set()
