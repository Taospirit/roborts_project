#ifndef ROBORTS_LOCALIZATION_AMCL_CONFIG_H
#define ROBORTS_LOCALIZATION_AMCL_CONFIG_H

#include <ros/ros.h>

namespace roborts_localization {

    struct AmclConfig {
        void GetParam(ros::NodeHandle *nh) {
            node_name = ros::this_node::getName();
            nh->param<bool>(node_name + "/use_map_topic", use_map_topic, true);
            nh->param<bool>(node_name + "/first_map_only", use_map_topic, true);
            nh->param<int>(node_name + "/gui_publish_rate", gui_publish_rate, 10);
            nh->param<double>(node_name + "/laser_min_range", laser_min_range, 0.15);
            nh->param<double>(node_name + "/laser_max_range", laser_max_range, 8.0);
            nh->param<int>(node_name + "/laser_max_beams", laser_max_beams, 30);
            nh->param<int>(node_name + "/min_particles", min_particles, 500);
            nh->param<int>(node_name + "/max_particles", max_particles, 5000);
            nh->param<double>(node_name + "/kld_error", kld_err, 0.05);
            nh->param<double>(node_name + "/kld_z", kld_z, 0.99);
            nh->param<double>(node_name + "/z_hit", z_hit, 0.5);
            nh->param<double>(node_name + "/z_rand", z_rand, 0.5);
            nh->param<double>(node_name + "/sigma_hit", sigma_hit, 0.2);
            nh->param<double>(node_name + "/lambda_short", lambda_short, 0.1);
            nh->param<double>(node_name + "/laser_likelihood_max_dist", laser_likelihood_max_dist, 2.0);
            nh->param<bool>(node_name + "/do_beamskip", do_beamskip, true);
            nh->param<double>(node_name + "/beam_skip_distance", beam_skip_distance, 0.5);
            nh->param<double>(node_name + "/beam_skip_threshold", beam_skip_threshold, 0.3);
            nh->param<double>(node_name + "/beam_skip_error_threshold", beam_skip_error_threshold, 0.9);
            nh->param<double>(node_name + "/odom_alpha1", odom_alpha1, 0.005);
            nh->param<double>(node_name + "/odom_alpha2", odom_alpha2, 0.005);
            nh->param<double>(node_name + "/odom_alpha3", odom_alpha3, 0.01);
            nh->param<double>(node_name + "/odom_alpha4", odom_alpha4, 0.005);
            nh->param<double>(node_name + "/odom_alpha5", odom_alpha5, 0.003);
            nh->param<double>(node_name + "/update_min_d", update_min_d, 0.2);
            nh->param<double>(node_name + "/update_min_a", update_min_a, 0.5);
            nh->param<int>(node_name + "/resample_interval", resample_interval, 1);
            nh->param<double>(node_name + "/transform_tolerance", transform_tolerance, 1);
            nh->param<double>(node_name + "/recovery_alpha_slow", recovery_alpha_slow, 0.1);
            nh->param<double>(node_name + "/recovery_alpha_fast", recovery_alpha_fast, 0.001);
            nh->param<bool>(node_name + "/use_global_localization", use_global_localization, false);
            nh->param<bool>(node_name + "/random_heading", random_heading, false);
            nh->param<int>(node_name + "/max_uwb_particles", max_uwb_particles, 10);
            nh->param<double>(node_name + "/uwb_cov_x", uwb_cov_x, 0.09);
            nh->param<double>(node_name + "/uwb_cov_y", uwb_cov_y, 0.09);
            nh->param<double>(node_name + "/resample_uwb_factor", resample_uwb_factor, 3.0);
        };

        std::string node_name;
        bool use_map_topic;
        bool first_map_only;
        int gui_publish_rate;

        double laser_min_range;
        double laser_max_range;
        int laser_max_beams;

        int min_particles;
        int max_particles;

        double kld_err;
        double kld_z;

//  LaserModel laser_model = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
        double z_hit;
        double z_rand;
        double sigma_hit;
        double lambda_short;
        double laser_likelihood_max_dist;
        bool do_beamskip;
        double beam_skip_distance;
        double beam_skip_threshold;
        double beam_skip_error_threshold;

//  OdomModel odom_model = ODOM_MODEL_OMNI;
        double odom_alpha1;
        double odom_alpha2;
        double odom_alpha3;
        double odom_alpha4;
        double odom_alpha5;

        double update_min_d;
        double update_min_a;

        int resample_interval;
        double transform_tolerance;
        double recovery_alpha_slow;
        double recovery_alpha_fast;

        bool use_global_localization;
        bool random_heading;
        double laser_filter_weight;

        int max_uwb_particles;
        double uwb_cov_x;
        double uwb_cov_y;
        double resample_uwb_factor;
    };

}// roborts_localization

#endif // ROBORTS_LOCALIZATION_AMCL_CONFIG_H