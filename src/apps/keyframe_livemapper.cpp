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
    initFilter();

    setPublishers();
    setSubscribers();

    // publish map to odom transform periodicallyu
    _stop_thread = false;
    _thread_map_to_odom = boost::thread(boost::bind(&KeyframeLiveMapper::publishMapTransform,this));
    _thread_octomap     = boost::thread(boost::bind(&KeyframeLiveMapper::updateOctomap,this));
  }

  KeyframeLiveMapper::~KeyframeLiveMapper()
  {
    ROS_INFO("Stopping threads...");
    _stop_thread = true;
    _thread_map_to_odom.join();
    ROS_INFO("Threads Stopped");
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
    _priv_n.param<double>("graph_matcher_max_desc_ratio",_graph_matcher_max_desc_ratio,0.75);
    _priv_n.param<double>("graph_matcher_max_desc_dist",_graph_matcher_max_desc_dist,0.5);
    _priv_n.param<std::string>("graph_output_path",_graph_output_path,"~/mapping_debug");

    _map_to_odom.setIdentity();
  }

  void KeyframeLiveMapper::setPublishers()
  {
    _pub_octomap = _n.advertise<octomap_msgs::Octomap>("octomap_binary",1,true);
  }


  void KeyframeLiveMapper::setSubscribers()
  {
    image_transport::ImageTransport rgb_it(_n);
    image_transport::ImageTransport depth_it(_n);

    image_transport::TransportHints rgb_th(_rgb_transporthint);
    image_transport::TransportHints depth_th(_depth_transporthint);


    _sub_rgb.subscribe   (rgb_it,     "rgbd/rgb",   _sub_queue_size, rgb_th);
    _sub_depth.subscribe (depth_it,   "rgbd/depth", _sub_queue_size, depth_th);
    _sub_camerainfo.subscribe  (_n,   "rgbd/info",  _sub_queue_size);

    // Synchronize inputs.
    _sync.reset(new RGBDSynchronizer3(RGBDSyncPolicy3(_sub_queue_size), _sub_rgb, _sub_depth, _sub_camerainfo));
     
    _sync->registerCallback(boost::bind(&KeyframeLiveMapper::processRGBDMsg, this, _1, _2, _3));  
  }

  void KeyframeLiveMapper::processRGBDMsg(const sensor_msgs::Image::ConstPtr& rgb_msg, const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    ROS_INFO_STREAM("[RGBDSLAM] message received");
    tf::StampedTransform transform;
    // create a new frame and increment the counter
    rgbdtools::RGBDFrame frame;
    Eigen::Affine3f pose;
    const ros::Time& time = rgb_msg->header.stamp;

    try {
      _tf_listener.waitForTransform(_fixed_frame, rgb_msg->header.frame_id, time, ros::Duration(0.1));
      _tf_listener.lookupTransform(_fixed_frame,rgb_msg->header.frame_id, time, transform);
    }
    catch(...)
    {
      ROS_ERROR("Tranfrorm between %s and %s not found", _fixed_frame.c_str(), rgb_msg->header.frame_id.c_str());
      return;
    }

    // create RGBD data from ROS msgs
    ccny_rgbd::createRGBDFrameFromROSMessages(rgb_msg,depth_msg, info_msg, frame);
    frame.index = _rgbd_frame_index++;
    rgbdtools::RGBDKeyframe keyframe;

    // affine transform from tf msg
    pose = ccny_rgbd::eigenAffineFromTf(transform);

    keyframe = processFrame(frame);

    _mutex_keyframe.lock();
    buildCorrespondenceMatrix(keyframe,_keyframes,_associations);
    _mutex_keyframe.unlock();

    ROS_INFO_STREAM("Finished processing frame " << _keyframes.size() << " " << _associations.size() << std::endl);
  }

  rgbdtools::RGBDKeyframe KeyframeLiveMapper::processFrame(rgbdtools::RGBDFrame& frame)
  {
    rgbdtools::RGBDKeyframe keyframe(frame);
    prepareFeaturesForRANSAC(keyframe);
    prepareMatcher(keyframe);

    // What would these two do?
    // buildMatchMatrixSurfTree();
    // buildCandidateMatrixSurfTree();
    
    return keyframe;
  }

  void KeyframeLiveMapper::publishMapTransform() 
  {
    ROS_INFO("Initialized map to odom transform sender");
    ros::Rate r(10);

    while(ros::ok() && !(_stop_thread)) {
      boost::mutex::scoped_lock(_mutex_map_to_odom);
      tf::StampedTransform transform_msg(_map_to_odom, ros::Time::now(), _fixed_frame, _odom_frame);

      _tf_broadcaster.sendTransform(transform_msg);
      r.sleep();
    }
  }

  void KeyframeLiveMapper::updateOctomap()
  {
    int current_keyframes_size;
    int current_associations_size;
    //double elapse;
    rgbdtools::Pose pose_before_optimization;
    rgbdtools::Pose pose_after_optimization;

    ROS_INFO("Initialising update octomap thread");

    while(ros::ok() && !(_stop_thread)) {
      boost::mutex::scoped_lock(_mutex_keyframe);

      current_keyframes_size   = _keyframes.size();
      current_associations_size = _associations.size();
      pose_before_optimization = _keyframes[current_keyframes_size-1].pose;

      //elapse = solveGraph(current_keyframes_size,current_associations_size);
      solveGraph(current_keyframes_size,current_associations_size);

      // updating keyframes
      _mutex_keyframe.lock();
      updateKeyframe(current_keyframes_size);
      _mutex_keyframe.unlock();

      pose_after_optimization = _keyframes[current_keyframes_size-1].pose;

      // update map to odom 
      _mutex_map_to_odom.lock();
      _map_to_odom = ccny_rgbd::tfFromEigenAffine(pose_after_optimization * pose_before_optimization.inverse() * ccny_rgbd::eigenAffineFromTf(_map_to_odom));
      _mutex_map_to_odom.unlock();

      // build octomap
      octomap::ColorOcTree tree(_octomap_res);

      buildColorOctomap(tree,current_keyframes_size);
      publishOctomap(tree);


      ros::Duration(1).sleep();
    }
  }

  double KeyframeLiveMapper::solveGraph(int current_keyframes_size,int current_associations_size)
  {
    int i;
    ros::Time begin;
    ros::Time end; 
    
    begin = ros::Time::now(); 
    // adding verticies
    for(i = 0; i < current_keyframes_size; i++) {
      addVertexToOptimizer(_keyframes[i].pose,i);
    }

    // adding ransac edges
    for(i = 0; i < current_associations_size; i++) {
      if(_associations[i].type != rgbdtools::KeyframeAssociation::RANSAC)
        addEdgeTopOptimizer(_associations[i]);
    }

    // Optimization
    optimizeGraph();

    end = ros::Time::now();
    ROS_INFO("Done");

    double elapse = (end.toNSec() - begin.toNSec()) * 0.000001;
    return elapse;
  }

  void KeyframeLiveMapper::updateKeyframe(int current_keyframes_size)
  {
    
    for(int i = 0; i < current_keyframes_size; i ++)
    {
      Eigen::Affine3f pose;

      getOptimizedPose(pose,i); 
      _keyframes[i].pose = pose;
    }
  }

}
