#ifndef __MOONRANGER_ARC_MODEL_H__
#define __MOONRANGER_ARC_MODEL_H__

#include <map>
#include <list>
#include <string>
#include <path_model.h>
#include <math.h>
#include <stdlib.h>
#include <rasm_common_types.h>

/*!
 * 12/07/2020:
 * MoonrangerArc serves as a simple class to define the individual drive-arcs
 * which comprise a MoonrangerArcModel. Note that a path model contains ONLY
 * forward OR reverse arcs at a given time (depending upon the planner's
 * evaluation process).
 *
 * MoonrangerArcModel derives from the PathModel class, which is declared in
 * the /core/common directory.
 *
 * Neither the classes defined herein, nor a PathModel, depend upon the
 * tmap_arc or multi_arc classes.
 *
 * Note that MoonrangerArc does not incorporate dynamic memory,
 * so a destructor isn't needed
 */
class MoonrangerArc
{
 public:
  double m_radius_meters; /*curvature of a drive arc*/
  bool   m_forward; /*1=forward; 0=reverse*/
  double m_speed; /*speed which the rover drives*/

  MoonrangerArc() : m_radius_meters(0.0), m_forward(true), m_speed(0.0) {};
  MoonrangerArc(double radius, bool fwd, double speed) : m_radius_meters(radius), m_forward(fwd), m_speed(speed) {};
};

class MoonrangerArcModel : public PathModel
{
 public:
  MoonrangerArcModel(std::map<std::string, std::string>& config);
  ~MoonrangerArcModel();

  //void update_path_set(bool use_reverse_arcs=false);
  bool update_path_set(bool use_reverse_arcs=false);

  std::list<MoonrangerArc*> m_radii_fwd;
  std::list<MoonrangerArc*> m_radii_rev;

  double   m_total_distance_along_arc_meters;
  double   m_rover_length_meters;
  double   m_rover_width_meters;
  uint32_t m_num_points_per_arc;
  double   m_reverse_path_length_scaling_factor;
  double   m_max_speed_meters_per_sec;
  double   m_reverse_speed_multiplier;
  double   m_reverse_path_risk_factor;

  bool     m_use_reverse_arcs;
};

#endif /* __MOONRANGER_ARC_MODEL_H__ */
