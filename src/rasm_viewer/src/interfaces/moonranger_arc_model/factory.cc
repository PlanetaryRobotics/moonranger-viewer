#include <stdio.h>
#include <stdlib.h>
#include <rasm_common_types.h>
#include <path_model.h>
#include <util.h>
#include "moonranger_arc_model.h"
#include <string>
#include <map>
#include <list>
#include <stdint.h>


#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus


/*!
 * A functor to create a MoonrangerArcModel:
 */
PathModel* construct_path_model(std::map<std::string, std::string>& config)
{
  return new MoonrangerArcModel(config);
}


#ifdef __cplusplus
} // extern "C"
#endif //__cplusplus
