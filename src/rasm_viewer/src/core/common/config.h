#include <map>
#include <string>

#ifndef __RASM_CONFIG_H__
#define __RASM_CONFIG_H__

std::map<std::string, std::string> makeConfig(int argc, const char** argv);
std::map<std::string, std::string> makeConfig(std::string config_filename);

#endif /* __RASM_CONFIG_H__ */


