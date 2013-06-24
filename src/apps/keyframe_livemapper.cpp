/**
  keyframe_livemapper.cpp 
  @ GPL Licensed. Please refer LICENCE file in the package
 **/

#include "turtlebot_rgbdslam/keyframe_livemapper.h"

namespace turtlebot_rgbdslam {
  KeyframeLiveMapper::KeyframeLiveMapper(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private)
    : _n(nh), _priv_n(nh_private), _rgbd_frame_index(0)
  {
    initParams();
    initGraphDetector();

    setPublishers();
    setSubscribers();

    // optimization thread
  }

  KeyframeLiveMapper::~KeyframeLiveMapper()
  {
  }

  void KeyframeLiveMapper::initParams()
  {
    _priv_n.param<bool>("verbose",_verbose,false);
    _priv_n.param<int>("sub_queue_size",_sub_queue_size,5);

    // transport hints
    _priv_n.param<std::string>("rgb_transporthint",_rgb_transporthint,"compressed");
    _priv_n.param<std::string>("depth_transporthint",_depth_transporthint,"compressedDepth");

    // frame 
    _priv_n.param<std::string>("fixed_frame",_fixed_frame,"map");
    _priv_n.param<std::string>("odom_frame",_odom_frame,"odom");

    // octomap
    _priv_n.param<double>("octomap_res",_octomap_res,0.05);
    _priv_n.param<bool>("octomap_with_color",_octomap_with_color,true);

    // kf
    _priv_n.param<double>("kf_dist_eps",_kf_dist_eps,0.1);
    _priv_n.param<double>("kf_angle_eps",_kf_angle_eps,10.0 * M_PI / 180.0);


    // boundary
    _priv_n.param<double>("max_range",_max_range,5.5);
    _priv_n.param<double>("max_stddev",_max_stddev,0.03);
    _priv_n.param<double>("max_map_z",_max_map_z,std::numeric_limits<double>::infinity());

    // configure graph detection 
    _priv_n.param<int>("graph/n_keypoints",_graph_n_keypoints,500);
    _priv_n.param<int>("graph/n_candidates",_graph_n_candidates,15);
    _priv_n.param<int>("graph/k_nearest_neighbors",_graph_k_nearest_neighbors,4);
    _priv_n.param<bool>("graph_matcher_use_desc_ratio_test",_graph_matcher_use_desc_ratio_test,true);
    _priv_n.param<std::string>("graph_output_path",_graph_output_path,"~/mapping_debug");
  }

  void KeyframeLiveMapper::initGraphDetector() 
  {
    _graph_detector.setNKeypoints(_graph_n_keypoints);
    _graph_detector.setNCandidates(_graph_n_candidates);   
    _graph_detector.setKNearestNeighbors(_graph_k_nearest_neighbors);    
    _graph_detector.setMatcherUseDescRatioTest(_graph_matcher_use_desc_ratio_test);
    
    _graph_detector.setSACReestimateTf(false);
    _graph_detector.setSACSaveResults(true);
    _graph_detector.setVerbose(_verbose);
    _graph_detector.setOutputPath(_graph_output_path);
  }

  void KeyframeLiveMapper::setPublishers()
  {
  }

  void KeyframeLiveMapper::setSubscribers()
  {
    image_transport::ImageTransport rgb_it(_n);
    image_transport::ImageTransport depth_it(_n);

    image_transport::TransportHints rgb_th(_rgb_transporthint);
    image_transport::TransportHints depth_th(_depth_transporthint);


    _sub_rgb.subscribe   (rgb_it,     "rgbd/rgb",   _sub_queue_size, rgb_th);
    _sub_depth.subscribe (depth_it,   "rgbd/depth", _sub_queue_size, depth_th);
    _sub_camerainfo.subscribe  (_n,         "rgbd/info",  _sub_queue_size);

    // Synchronize inputs.
    _sync.reset(new RGBDSynchronizer3(RGBDSyncPolicy3(_sub_queue_size), _sub_rgb, _sub_depth, _sub_camerainfo));
     
    _sync->registerCallback(boost::bind(&KeyframeLiveMapper::processRGBDMsg, this, _1, _2, _3));  
  }

  void KeyframeLiveMapper::processRGBDMsg(const sensor_msgs::Image::ConstPtr& rgb_msg, const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    tf::StampedTransform transform;

    const ros::Time& time = rgb_msg->header.stamp;
    bool result;

    try {
      _tf_listener.waitForTransform(_fixed_frame, rgb_msg->header.frame_id, time, ros::Duration(0.1));
      _tf_listener.lookupTransform(_fixed_frame,rgb_msg->header.frame_id, time, transform);
    }
    catch(...)
    {
      ROS_WARN("Caught while waiting for tfs");
      return;
    }

    // create a new frame and increment the counter
    rgbdtools::RGBDFrame frame;
    Eigen::Affine3f pose;


    // create RGBD data from ROS msgs
  //  createRGBDFrameFromROSMessages(rgb_msg,depth_msg, info_msg, frame);
    /*
    frame.index = rgbd_frame_index_;
    rgbd_frame_index_++;

    // affine transform from tf msg
    pose = eigenAffineFromTf(transform);

    result = processFrame(frame, pose);
    if(result) addKeyframe(frame, pose); 

    ROS_INFO("[RGBDSLAM] processing : %s",result?"SUCCESS":"FAILED");
    // publish debuging data
    //
  //  if(result) {
  //    publishKeyframeData(keyframes_.size() - 1);
  //  }*/
  }

  bool KeyframeLiveMapper::processFrame(const rgbdtools::RGBDFrame& frame, const Eigen::Affine3f& pose)
  {
    /*
    bool result;      // determine if a new keyframe is needed
    geometry_msgs::PoseStamped frame_pose;   // add the frame pose to the path vector
    tf::Transforrm frame_tf = tfFromEigenAffine(pose);

    tf::poseTFToMsg(frame_tf, frame_pose.pose);

    // update the header of the pose for the path
    frame_pose.header.frame_id  = fixed_frame_;
    frame_pose.header.seq       = frame.header.seq;
    frame_pose.header.stamp     = frame.header.stamp;

    path_msg_.poses.push_back(frame_pose);
    
    if(keyframe_.empty())
    {
      result = true;
    }
    else {
      double dist, angle;
      getTfDifference(tfFromEigenAffine(pose),tfFromEigenAffine(keyframe_.back().pose),dist,angle);
      
      if(dist > kf_dist_eps || angle > kf_angle_eps_)
      {
        result = true;
      }
      else {
        result = false;
      }
    }
*/
    return false;
  }

  void KeyframeLiveMapper::addKeyframe(const rgbdtools::RGBDFrame& frame, const Eigen::Affine3f& pose)
  {
    /*
    rgbdtools::RGBDKeyframe keyframe(frame);
    keyframe.pose = pose;
    
    keyframes_.push_back(keyframe);
    */
  }


}
