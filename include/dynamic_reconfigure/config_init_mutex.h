#ifndef __DYNAMIC_RECONFIGURE__CONFIG_INIT_MUTEX_H__
#define __DYNAMIC_RECONFIGURE__CONFIG_INIT_MUTEX_H__

#include <boost/thread/mutex.hpp>

#include <ros/macros.h>

// Import/export for windows dll's and visibility for gcc shared libraries.

#ifdef ROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
  #ifdef dynamic_reconfigure_config_init_mutex_EXPORTS // we are building a shared lib/dll
    #define DYNAMIC_RECONFIGURE_CONFIG_INIT_MUTEX_DECL ROS_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define DYNAMIC_RECONFIGURE_CONFIG_INIT_MUTEX_DECL ROS_HELPER_IMPORT
  #endif
#else // ros is being built around static libraries
  #define DYNAMIC_RECONFIGURE_CONFIG_INIT_MUTEX_DECL
#endif

namespace dynamic_reconfigure
{
  extern DYNAMIC_RECONFIGURE_CONFIG_INIT_MUTEX_DECL boost::mutex __init_mutex__;
}

#endif
