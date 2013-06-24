/**
  @file keyframe_livemapper.h
  @ GNU Licensed. Please refer LICENCE file in the package
 **/
#ifndef TURTLEBOT_RGBDSLAM_KEYFRAME_LIVEMAPPER_H_
#define TURTLEBOT_RGBDSLAM_KEYFRAME_LIVEMAPPER_H_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>

#include <rgbdtools/rgbdtools.h>

namespace turtlebot_rgbdslam {
  /** @brief Builds a 3D map from a series of RGBD keyframes.

    The KeyframeLiveMapper receives RGBD keyframes, optimize graphs, and publishes live octomap
   **/

  class KeyframeLiveMapper {
    public:
      KeyframeLiveMapper(const ros::NodeHandle& n,const ros::NodeHandle& priv_n);
      ~KeyframeLiveMapper();

    protected:
      void processRGBDMsg(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg,const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);

      void initParams();
      void initGraphDetector() 
      void setSubscribers();
      void setPublishers();

      bool processFrame(const rgbdtools::RGBDFrame& frame, const AffineTransform& pose);
      void addKeyframe(const rgbdtools::RGBDFrame& frame, const AffineTransform& pose);
    private:
      ros::NodeHandle _n;
      ros::NodeHandle _priv_n;
        
      int _rgbd_frame_index;

      image_transport::SubscriberFilter                     _sub_rgb;
      image_transport::SubscriberFilter                     _sub_depth;
      message_filters::Subscriber<sensor_msgs::CameraInfo>  _sub_camerainfo;

      rgbdtools::KeyframeGraphDetector                      _graph_detector;
      rgbdtools::keyframeGraphSolverG2O                     _graph_solver;
      rgbdtools::KeyframeAssociationVector                  _associations;        


      //// parameters
      bool          _verbose;
      unsigned int  _sub_queue_size;
      std::string   _fixed_frame;
      std::string   _odom_frame;

      // octomap 
      double        _octomap_res;
      bool          _octomap_with_color;

      // kf
      double        _kf_dist_eps;
      double        _kf_angle_eps;

      // boundary
      double        _max_range;
      double        _max_stddev;
      double        _max_map_z;

      // image transport hints
      std::string _rgb_transporthint;
      std::string _depth_transporthint;

      // graph configuration
      unsigned int _graph_n_keypoints;
      unsigned int _graph_n_candidates;
      unsigned int _graph_k_nearest_neighbors;
      bool _graph_matcher_use_desc_ratio_test;
      std::string _graph_output_path;
  };
}
