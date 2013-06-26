/**
  @file keyframe_livemapper.h
  @ GNU Licensed. Please refer LICENCE file in the package
 **/
#ifndef TURTLEBOT_RGBDSLAM_KEYFRAME_LIVEMAPPER_H_
#define TURTLEBOT_RGBDSLAM_KEYFRAME_LIVEMAPPER_H_

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>


#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/nonfree/features2d.hpp>
#include <rgbdtools/rgbdtools.h>
#include <tbb/concurrent_vector.h>

#include <ccny_rgbd/util.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

namespace turtlebot_rgbdslam {
  /** @brief Builds a 3D map from a series of RGBD keyframes.

    The KeyframeLiveMapper receives RGBD keyframes, optimize graphs, and publishes live octomap
   **/
   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> RGBDSyncPolicy3;
   typedef message_filters::Synchronizer<RGBDSyncPolicy3> RGBDSynchronizer3;

   typedef pcl::PointXYZ    PointXYZ;
   typedef pcl::PointXYZRGB PointXYZRGB;
   typedef pcl::PointCloud<PointXYZ>     PointCloudXYZ;
   typedef pcl::PointCloud<PointXYZRGB> PointCloudXYZRGB;

  class KeyframeLiveMapper {
    public:
      KeyframeLiveMapper(const ros::NodeHandle& n,const ros::NodeHandle& priv_n);
      ~KeyframeLiveMapper();

    protected:
      void processRGBDMsg(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg,const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);

      void initParams();
      void initRGBDParams();
      void initFilter();
      void setSubscribers();
      void setPublishers();

      rgbdtools::RGBDKeyframe processFrame(rgbdtools::RGBDFrame& frame);
        void prepareFeaturesForRANSAC(rgbdtools::RGBDKeyframe& keyframe);
        void prepareMatcher(rgbdtools::RGBDKeyframe& keyframe);
      void buildCorrespondenceMatrix(rgbdtools::RGBDKeyframe& keyframe,tbb::concurrent_vector<rgbdtools::RGBDKeyframe>& keyframes,tbb::concurrent_vector<rgbdtools::KeyframeAssociation>& associations);
        int pairwiseMatchingRANSAC(const rgbdtools::RGBDKeyframe& query,const rgbdtools::RGBDKeyframe& train,rgbdtools::DMatchVector& best_inlier_matches,Eigen::Matrix4f& best_transformation);
        void getCandidateMatches(const rgbdtools::RGBDKeyframe& query,const rgbdtools::RGBDKeyframe& train, rgbdtools::DMatchVector& candidate_matches);
        
      void addKeyframe(const rgbdtools::RGBDFrame& frame, const Eigen::Affine3f& pose);

      void publishMapTransform();

      void updateOctomap();
      double generateAndSolveGraph();
      void buildColorOctomap(octomap::ColorOcTree& tree);
        void buildOctoCloud(octomap::Pointcloud& octomap_cloud,const PointCloudXYZRGB& cloud);
        void insertColor(octomap::ColorOcTree& tree,const PointCloudXYZRGB& cloud,const Eigen::Affine3f pose);
        void publishOctomap(octomap::ColorOcTree& tree);
//      octomap::pose6d poseTfToOctomap(tf::Pose pose_tf);

    private:
      ros::NodeHandle _n;
      ros::NodeHandle _priv_n;
        
      int _rgbd_frame_index;

      ros::Publisher                                        _pub_octomap;

      image_transport::SubscriberFilter                     _sub_rgb;
      image_transport::SubscriberFilter                     _sub_depth;
      message_filters::Subscriber<sensor_msgs::CameraInfo>  _sub_camerainfo;

      rgbdtools::KeyframeGraphDetector                      _graph_detector;
      rgbdtools::KeyframeGraphSolverG2O                     _graph_solver;

      tf::Transform             _map_to_odom;
      tf::TransformListener     _tf_listener;
      tf::TransformBroadcaster  _tf_broadcaster;

      nav_msgs::Path            _path_msg;

      tbb::concurrent_vector<rgbdtools::RGBDKeyframe>             _keyframes;
      tbb::concurrent_vector<rgbdtools::KeyframeAssociation>  _associations;        

      //// parameters
      bool          _verbose;
      int           _sub_queue_size;
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
      int _graph_n_keypoints;
      int _graph_n_candidates;
      int _graph_k_nearest_neighbors;

      bool _graph_matcher_use_desc_ratio_test;
      double _graph_matcher_max_desc_ratio;
      double _graph_matcher_max_desc_dist;

      std::string _graph_output_path;

      boost::shared_ptr<RGBDSynchronizer3> _sync;
      boost::thread                       _thread_map_to_odom;
      boost::thread                       _thread_octomap;
      bool                                _stop_thread;

      pcl::PassThrough<PointXYZRGB>     _pass;

      // RANSAC params
      double          _sac_min_inliers;                
      double          _sac_max_eucl_dist_sq;
      double          _sac_reestimate_tf;          

      double          _ransac_confidence;             
      int             _ransac_max_iterations;
      double          _ransac_sufficient_inlier_ratio;
      double          _log_one_minus_ransac_confidence;
  };
}

#endif
