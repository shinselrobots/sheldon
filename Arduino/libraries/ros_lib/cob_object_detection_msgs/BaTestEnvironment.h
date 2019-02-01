#ifndef _ROS_SERVICE_BaTestEnvironment_h
#define _ROS_SERVICE_BaTestEnvironment_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"

namespace cob_object_detection_msgs
{

static const char BATESTENVIRONMENT[] = "cob_object_detection_msgs/BaTestEnvironment";

  class BaTestEnvironmentRequest : public ros::Msg
  {
    public:
      typedef int32_t _number_points_type;
      _number_points_type number_points;
      typedef float _frame_view_number_type;
      _frame_view_number_type frame_view_number;
      typedef float _cone_points_per_plane_type;
      _cone_points_per_plane_type cone_points_per_plane;
      typedef float _limit_error_matching_type;
      _limit_error_matching_type limit_error_matching;
      typedef float _ba_kernel_param_type;
      _ba_kernel_param_type ba_kernel_param;
      typedef int32_t _ba_num_iter_type;
      _ba_num_iter_type ba_num_iter;
      typedef float _ba_initial_mu_type;
      _ba_initial_mu_type ba_initial_mu;
      typedef float _ba_final_mu_factor_type;
      _ba_final_mu_factor_type ba_final_mu_factor;
      typedef float _ba_tau_type;
      _ba_tau_type ba_tau;
      typedef float _angle_sigma_degree_type;
      _angle_sigma_degree_type angle_sigma_degree;
      typedef float _translate_sigma_m_type;
      _translate_sigma_m_type translate_sigma_m;
      typedef float _obs_point_sigma_m_type;
      _obs_point_sigma_m_type obs_point_sigma_m;
      typedef float _world_point_sigma_m_type;
      _world_point_sigma_m_type world_point_sigma_m;

    BaTestEnvironmentRequest():
      number_points(0),
      frame_view_number(0),
      cone_points_per_plane(0),
      limit_error_matching(0),
      ba_kernel_param(0),
      ba_num_iter(0),
      ba_initial_mu(0),
      ba_final_mu_factor(0),
      ba_tau(0),
      angle_sigma_degree(0),
      translate_sigma_m(0),
      obs_point_sigma_m(0),
      world_point_sigma_m(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_number_points;
      u_number_points.real = this->number_points;
      *(outbuffer + offset + 0) = (u_number_points.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_number_points.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_number_points.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_number_points.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->number_points);
      union {
        float real;
        uint32_t base;
      } u_frame_view_number;
      u_frame_view_number.real = this->frame_view_number;
      *(outbuffer + offset + 0) = (u_frame_view_number.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frame_view_number.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frame_view_number.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frame_view_number.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frame_view_number);
      union {
        float real;
        uint32_t base;
      } u_cone_points_per_plane;
      u_cone_points_per_plane.real = this->cone_points_per_plane;
      *(outbuffer + offset + 0) = (u_cone_points_per_plane.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cone_points_per_plane.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cone_points_per_plane.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cone_points_per_plane.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cone_points_per_plane);
      union {
        float real;
        uint32_t base;
      } u_limit_error_matching;
      u_limit_error_matching.real = this->limit_error_matching;
      *(outbuffer + offset + 0) = (u_limit_error_matching.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_limit_error_matching.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_limit_error_matching.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_limit_error_matching.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->limit_error_matching);
      union {
        float real;
        uint32_t base;
      } u_ba_kernel_param;
      u_ba_kernel_param.real = this->ba_kernel_param;
      *(outbuffer + offset + 0) = (u_ba_kernel_param.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ba_kernel_param.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ba_kernel_param.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ba_kernel_param.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ba_kernel_param);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_num_iter;
      u_ba_num_iter.real = this->ba_num_iter;
      *(outbuffer + offset + 0) = (u_ba_num_iter.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ba_num_iter.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ba_num_iter.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ba_num_iter.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ba_num_iter);
      union {
        float real;
        uint32_t base;
      } u_ba_initial_mu;
      u_ba_initial_mu.real = this->ba_initial_mu;
      *(outbuffer + offset + 0) = (u_ba_initial_mu.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ba_initial_mu.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ba_initial_mu.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ba_initial_mu.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ba_initial_mu);
      union {
        float real;
        uint32_t base;
      } u_ba_final_mu_factor;
      u_ba_final_mu_factor.real = this->ba_final_mu_factor;
      *(outbuffer + offset + 0) = (u_ba_final_mu_factor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ba_final_mu_factor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ba_final_mu_factor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ba_final_mu_factor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ba_final_mu_factor);
      union {
        float real;
        uint32_t base;
      } u_ba_tau;
      u_ba_tau.real = this->ba_tau;
      *(outbuffer + offset + 0) = (u_ba_tau.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ba_tau.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ba_tau.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ba_tau.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ba_tau);
      union {
        float real;
        uint32_t base;
      } u_angle_sigma_degree;
      u_angle_sigma_degree.real = this->angle_sigma_degree;
      *(outbuffer + offset + 0) = (u_angle_sigma_degree.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_sigma_degree.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_sigma_degree.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_sigma_degree.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_sigma_degree);
      union {
        float real;
        uint32_t base;
      } u_translate_sigma_m;
      u_translate_sigma_m.real = this->translate_sigma_m;
      *(outbuffer + offset + 0) = (u_translate_sigma_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_translate_sigma_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_translate_sigma_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_translate_sigma_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->translate_sigma_m);
      union {
        float real;
        uint32_t base;
      } u_obs_point_sigma_m;
      u_obs_point_sigma_m.real = this->obs_point_sigma_m;
      *(outbuffer + offset + 0) = (u_obs_point_sigma_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_obs_point_sigma_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_obs_point_sigma_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_obs_point_sigma_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->obs_point_sigma_m);
      union {
        float real;
        uint32_t base;
      } u_world_point_sigma_m;
      u_world_point_sigma_m.real = this->world_point_sigma_m;
      *(outbuffer + offset + 0) = (u_world_point_sigma_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_world_point_sigma_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_world_point_sigma_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_world_point_sigma_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->world_point_sigma_m);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_number_points;
      u_number_points.base = 0;
      u_number_points.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_number_points.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_number_points.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_number_points.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->number_points = u_number_points.real;
      offset += sizeof(this->number_points);
      union {
        float real;
        uint32_t base;
      } u_frame_view_number;
      u_frame_view_number.base = 0;
      u_frame_view_number.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_frame_view_number.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_frame_view_number.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_frame_view_number.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->frame_view_number = u_frame_view_number.real;
      offset += sizeof(this->frame_view_number);
      union {
        float real;
        uint32_t base;
      } u_cone_points_per_plane;
      u_cone_points_per_plane.base = 0;
      u_cone_points_per_plane.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cone_points_per_plane.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cone_points_per_plane.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cone_points_per_plane.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cone_points_per_plane = u_cone_points_per_plane.real;
      offset += sizeof(this->cone_points_per_plane);
      union {
        float real;
        uint32_t base;
      } u_limit_error_matching;
      u_limit_error_matching.base = 0;
      u_limit_error_matching.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_limit_error_matching.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_limit_error_matching.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_limit_error_matching.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->limit_error_matching = u_limit_error_matching.real;
      offset += sizeof(this->limit_error_matching);
      union {
        float real;
        uint32_t base;
      } u_ba_kernel_param;
      u_ba_kernel_param.base = 0;
      u_ba_kernel_param.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ba_kernel_param.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ba_kernel_param.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ba_kernel_param.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ba_kernel_param = u_ba_kernel_param.real;
      offset += sizeof(this->ba_kernel_param);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_num_iter;
      u_ba_num_iter.base = 0;
      u_ba_num_iter.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ba_num_iter.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ba_num_iter.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ba_num_iter.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ba_num_iter = u_ba_num_iter.real;
      offset += sizeof(this->ba_num_iter);
      union {
        float real;
        uint32_t base;
      } u_ba_initial_mu;
      u_ba_initial_mu.base = 0;
      u_ba_initial_mu.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ba_initial_mu.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ba_initial_mu.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ba_initial_mu.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ba_initial_mu = u_ba_initial_mu.real;
      offset += sizeof(this->ba_initial_mu);
      union {
        float real;
        uint32_t base;
      } u_ba_final_mu_factor;
      u_ba_final_mu_factor.base = 0;
      u_ba_final_mu_factor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ba_final_mu_factor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ba_final_mu_factor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ba_final_mu_factor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ba_final_mu_factor = u_ba_final_mu_factor.real;
      offset += sizeof(this->ba_final_mu_factor);
      union {
        float real;
        uint32_t base;
      } u_ba_tau;
      u_ba_tau.base = 0;
      u_ba_tau.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ba_tau.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ba_tau.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ba_tau.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ba_tau = u_ba_tau.real;
      offset += sizeof(this->ba_tau);
      union {
        float real;
        uint32_t base;
      } u_angle_sigma_degree;
      u_angle_sigma_degree.base = 0;
      u_angle_sigma_degree.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_sigma_degree.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_sigma_degree.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_sigma_degree.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_sigma_degree = u_angle_sigma_degree.real;
      offset += sizeof(this->angle_sigma_degree);
      union {
        float real;
        uint32_t base;
      } u_translate_sigma_m;
      u_translate_sigma_m.base = 0;
      u_translate_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_translate_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_translate_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_translate_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->translate_sigma_m = u_translate_sigma_m.real;
      offset += sizeof(this->translate_sigma_m);
      union {
        float real;
        uint32_t base;
      } u_obs_point_sigma_m;
      u_obs_point_sigma_m.base = 0;
      u_obs_point_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_obs_point_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_obs_point_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_obs_point_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->obs_point_sigma_m = u_obs_point_sigma_m.real;
      offset += sizeof(this->obs_point_sigma_m);
      union {
        float real;
        uint32_t base;
      } u_world_point_sigma_m;
      u_world_point_sigma_m.base = 0;
      u_world_point_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_world_point_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_world_point_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_world_point_sigma_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->world_point_sigma_m = u_world_point_sigma_m.real;
      offset += sizeof(this->world_point_sigma_m);
     return offset;
    }

    const char * getType(){ return BATESTENVIRONMENT; };
    const char * getMD5(){ return "65aeb9eaa6cbc248861e82cf7f7464c0"; };

  };

  class BaTestEnvironmentResponse : public ros::Msg
  {
    public:
      typedef float _mean_error_type;
      _mean_error_type mean_error;
      typedef float _std_dev_type;
      _std_dev_type std_dev;
      typedef float _min_error_type;
      _min_error_type min_error;
      typedef float _max_error_type;
      _max_error_type max_error;
      typedef float _runs_count_type;
      _runs_count_type runs_count;
      typedef float _runs_sum_type;
      _runs_sum_type runs_sum;
      typedef float _runs_sum2_type;
      _runs_sum2_type runs_sum2;
      typedef float _time_duration_type;
      _time_duration_type time_duration;
      typedef int32_t _observations_type;
      _observations_type observations;
      typedef int32_t _false_matchings_type;
      _false_matchings_type false_matchings;
      typedef std_msgs::String _result_type;
      _result_type result;

    BaTestEnvironmentResponse():
      mean_error(0),
      std_dev(0),
      min_error(0),
      max_error(0),
      runs_count(0),
      runs_sum(0),
      runs_sum2(0),
      time_duration(0),
      observations(0),
      false_matchings(0),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_mean_error;
      u_mean_error.real = this->mean_error;
      *(outbuffer + offset + 0) = (u_mean_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mean_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mean_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mean_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mean_error);
      union {
        float real;
        uint32_t base;
      } u_std_dev;
      u_std_dev.real = this->std_dev;
      *(outbuffer + offset + 0) = (u_std_dev.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_std_dev.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_std_dev.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_std_dev.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->std_dev);
      union {
        float real;
        uint32_t base;
      } u_min_error;
      u_min_error.real = this->min_error;
      *(outbuffer + offset + 0) = (u_min_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_error);
      union {
        float real;
        uint32_t base;
      } u_max_error;
      u_max_error.real = this->max_error;
      *(outbuffer + offset + 0) = (u_max_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_error);
      union {
        float real;
        uint32_t base;
      } u_runs_count;
      u_runs_count.real = this->runs_count;
      *(outbuffer + offset + 0) = (u_runs_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_runs_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_runs_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_runs_count.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->runs_count);
      union {
        float real;
        uint32_t base;
      } u_runs_sum;
      u_runs_sum.real = this->runs_sum;
      *(outbuffer + offset + 0) = (u_runs_sum.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_runs_sum.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_runs_sum.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_runs_sum.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->runs_sum);
      union {
        float real;
        uint32_t base;
      } u_runs_sum2;
      u_runs_sum2.real = this->runs_sum2;
      *(outbuffer + offset + 0) = (u_runs_sum2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_runs_sum2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_runs_sum2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_runs_sum2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->runs_sum2);
      union {
        float real;
        uint32_t base;
      } u_time_duration;
      u_time_duration.real = this->time_duration;
      *(outbuffer + offset + 0) = (u_time_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time_duration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_duration);
      union {
        int32_t real;
        uint32_t base;
      } u_observations;
      u_observations.real = this->observations;
      *(outbuffer + offset + 0) = (u_observations.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_observations.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_observations.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_observations.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->observations);
      union {
        int32_t real;
        uint32_t base;
      } u_false_matchings;
      u_false_matchings.real = this->false_matchings;
      *(outbuffer + offset + 0) = (u_false_matchings.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_false_matchings.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_false_matchings.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_false_matchings.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->false_matchings);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_mean_error;
      u_mean_error.base = 0;
      u_mean_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mean_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mean_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mean_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mean_error = u_mean_error.real;
      offset += sizeof(this->mean_error);
      union {
        float real;
        uint32_t base;
      } u_std_dev;
      u_std_dev.base = 0;
      u_std_dev.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_std_dev.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_std_dev.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_std_dev.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->std_dev = u_std_dev.real;
      offset += sizeof(this->std_dev);
      union {
        float real;
        uint32_t base;
      } u_min_error;
      u_min_error.base = 0;
      u_min_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_error = u_min_error.real;
      offset += sizeof(this->min_error);
      union {
        float real;
        uint32_t base;
      } u_max_error;
      u_max_error.base = 0;
      u_max_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_error = u_max_error.real;
      offset += sizeof(this->max_error);
      union {
        float real;
        uint32_t base;
      } u_runs_count;
      u_runs_count.base = 0;
      u_runs_count.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_runs_count.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_runs_count.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_runs_count.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->runs_count = u_runs_count.real;
      offset += sizeof(this->runs_count);
      union {
        float real;
        uint32_t base;
      } u_runs_sum;
      u_runs_sum.base = 0;
      u_runs_sum.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_runs_sum.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_runs_sum.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_runs_sum.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->runs_sum = u_runs_sum.real;
      offset += sizeof(this->runs_sum);
      union {
        float real;
        uint32_t base;
      } u_runs_sum2;
      u_runs_sum2.base = 0;
      u_runs_sum2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_runs_sum2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_runs_sum2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_runs_sum2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->runs_sum2 = u_runs_sum2.real;
      offset += sizeof(this->runs_sum2);
      union {
        float real;
        uint32_t base;
      } u_time_duration;
      u_time_duration.base = 0;
      u_time_duration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time_duration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time_duration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time_duration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->time_duration = u_time_duration.real;
      offset += sizeof(this->time_duration);
      union {
        int32_t real;
        uint32_t base;
      } u_observations;
      u_observations.base = 0;
      u_observations.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_observations.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_observations.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_observations.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->observations = u_observations.real;
      offset += sizeof(this->observations);
      union {
        int32_t real;
        uint32_t base;
      } u_false_matchings;
      u_false_matchings.base = 0;
      u_false_matchings.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_false_matchings.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_false_matchings.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_false_matchings.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->false_matchings = u_false_matchings.real;
      offset += sizeof(this->false_matchings);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return BATESTENVIRONMENT; };
    const char * getMD5(){ return "73db4f13e99b7e554aa13b596abbef41"; };

  };

  class BaTestEnvironment {
    public:
    typedef BaTestEnvironmentRequest Request;
    typedef BaTestEnvironmentResponse Response;
  };

}
#endif
