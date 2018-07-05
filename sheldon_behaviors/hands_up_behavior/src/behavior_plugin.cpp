// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <pluginlib/class_list_macros.h>
#include <behavior_common/behavior.h>

PYTHON_BEHAVIOR_PLUGIN(hands_up_behavior, "HANDS_UP");
PLUGINLIB_EXPORT_CLASS(hands_up_behavior, behavior_common::BehaviorPlugin);
