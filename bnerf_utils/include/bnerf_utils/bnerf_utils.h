#ifndef __UTILS_ROS_HEADERS_H__
#define __UTILS_ROS_HEADERS_H__


#include <bnerf_utils/ros/client.hpp>
#include <bnerf_utils/ros/publisher.hpp>
#include <bnerf_utils/ros/cloud_publisher.h>
#include <bnerf_utils/ros/transform_caster.h>

#include <bnerf_utils/logging/logger.h>
#include <bnerf_utils/logging/signal_button.h>


#define GET_OPTIONAL(nh, name, val, def) { \
    val = def; \
    LOG_IF(WARNING, !nh.param(name, val, val)) \
        << "can't find \"" << nh.resolveName(name) \
        << "\"\tusing default " << #def; \
}

#define GET_REQUIRED(nh, name, val) { \
    LOG_ASSERT(nh.getParam(name, val)) << std::endl \
        << "can't find \"" << nh.resolveName(name) \
        << "\"\taborting.. "; \
}


#endif