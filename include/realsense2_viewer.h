//
// Created by sdhm on 7/18/19.
//

#pragma once

#include <string>
#include <ros/console.h>


#define R2_DEFAULT_NS            "camera"

#define R2_TOPIC_IMAGE_RAW       "/image_raw"
#define R2_TOPIC_IMAGE_COLOR     "/color"
#define R2_TOPIC_IMAGE_DEPTH     "/depth"
#define R2_TOPIC_ALIGNED_DEPTH   "/aligned_depth_to_color"

#define R2_TOPIC_COMPRESSED      "/compressed"
#define R2_TOPIC_INFO            "/camera_info"


/// console
#define NO_COLOR        "\033[0m"
#define FG_BLACK        "\033[30m"
#define FG_RED          "\033[31m"
#define FG_GREEN        "\033[32m"
#define FG_YELLOW       "\033[33m"
#define FG_BLUE         "\033[34m"
#define FG_MAGENTA      "\033[35m"
#define FG_CYAN         "\033[36m"

const std::string getFunctionName(const std::string &name)
{
    size_t end = name.rfind('(');
    if(end == std::string::npos)
    {
        end = name.size();
    }
    size_t begin = 1 + name.rfind(' ', end);
    return name.substr(begin, end - begin);
}

#define OUT_AUX(FUNC_COLOR, MSG_COLOR, STREAM, MSG) STREAM(FUNC_COLOR "[" << getFunctionName(__PRETTY_FUNCTION__) << "] " MSG_COLOR << MSG << NO_COLOR)

#define OUT_DEBUG(msg) OUT_AUX(FG_BLUE, NO_COLOR, ROS_DEBUG_STREAM, msg)
#define OUT_INFO(msg) OUT_AUX(FG_GREEN, NO_COLOR, ROS_INFO_STREAM, msg)
#define OUT_WARN(msg) OUT_AUX(FG_YELLOW, FG_YELLOW, ROS_WARN_STREAM, msg)
#define OUT_ERROR(msg) OUT_AUX(FG_RED, FG_RED, ROS_ERROR_STREAM, msg)
