// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <pluginlib/class_list_macros.h>
#include <behavior_common/behavior.h>

// KLUDGE FOR "PLAY_SCRIPT" not on Arduino!
PYTHON_BEHAVIOR_PLUGIN(play_script_behavior, "RUN_SCRIPT"); 
PLUGINLIB_EXPORT_CLASS(play_script_behavior, behavior_common::BehaviorPlugin);
