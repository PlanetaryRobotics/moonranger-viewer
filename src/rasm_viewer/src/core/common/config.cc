#include <stdio.h>
#include <fstream>
#include <assert.h>
#include <util.h>
#include "config.h"

std::map<std::string, std::string> makeConfig(std::string config_filename)
{
  std::map<std::string, std::string> config;

  std::fstream f;
  f.open(config_filename.c_str(), std::fstream::in);
  std::string key, value;
  while(f >> key >> value)
    {
      if((';' == key[0]) || ('%' == key[0]) || ('#' == key[0])) 
	{
	  continue;
	}
      else if(".include" == key)
	{
	  printf("[%s:%d %f] including config file %s\n", 
		 __FILE__, __LINE__, now(),
		 key.c_str());
	  std::map<std::string, std::string> included_config = makeConfig(value);
	  config.insert(included_config.begin(), included_config.end());
	}
      else
	{
	  config[key] = value;
	  // printf("[%s:%d %f] config file '%s' '%s' = '%s'\n",
		//  __FILE__, __LINE__, now(), 
		//  config_filename.c_str(), key.c_str(), value.c_str());
	}
    }
  f.close();

  return config;
}

std::map<std::string, std::string> makeConfig(int argc, const char** argv)
{
  assert(NULL != argv);
  
  std::map<std::string, std::string> config;
  
  /*
   * First, scan through cmdline params
   */
  for(int i=1; i < argc; i++) // skip program name in argv[0]
    {
      assert(NULL != argv[i]);

      std::string key(argv[i]);
      std::string value("");

      if(key.find("--") == std::string::npos)
	{
	  // this doesn't appear to be a cmdline switch. so skip ahead.
	  printf("[%s:%d %f] found something other than a cmdline switch: '%s'\n",
		 __FILE__, __LINE__, now(), key.c_str());
	  continue;
	}


      if(argc > i+1)
	{
	  value = argv[i+1];
	  if(value.find("--") == std::string::npos)
	    {
	      // this is a real value, so skip ahead of it
	      i++;
	    }
	  else
	    {
	      // this is another key, so clear out this value
	      value = "";
	    }
	}

      // strip off leading "--" from key
      key = key.substr(2, std::string::npos);
      
      // add key-value pair to map
      config[key] = value;
      printf("[%s:%d %f] command line '%s' = '%s'\n",
	     __FILE__, __LINE__, now(), key.c_str(), value.c_str());

    } // for each cmdline parameter

  /*
   * Then, if there's a config file, read through it
   */
  
  if(config.find("config") != config.end()) //this will be true if "config" is found
    {
      std::map<std::string, std::string> file_config = makeConfig(config["config"]);

      std::map<std::string,std::string>::iterator it;
      for ( it=file_config.begin() ; it != file_config.end(); it++ )
	{
	  config.insert(*it);
	}
      
    }

  return config;
}
