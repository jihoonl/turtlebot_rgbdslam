/**
  keyframe_livemapper.cpp 
  @ GNU Licensed. Please refer LICENCE file in the package
 **/

namespace turtlebot_rgbdslam {
  KeyframeLiveMapper::KeyframeLiveMapper(const ros::NodeHandle& nh,const ros::NodeHandle& nh_private)
    : _n(nh), _priv_n(nh_private), rgbd_frame_index_(0)
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
    _priv_n.param<unsigned int>("sub_queue_size",_sub_queue_size,5);

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
    _priv_n.param<double>("max_map_z",_max_map_z,std::number_limits<double>::infinity());

    // configure graph detection 
      
    bool graph_matcher_use_desc_ratio_test = true;
      
    _priv_n.param<unsigned int>("graph/n_keypoints",_graph_n_keypoints,500);
    _priv_n.param<unsigned int>("graph/n_candidates",_graph_n_candidates,15);
    _priv_n.param<unsigned int>("graph/k_nearest_neighbors",_graph_k_nearest_neighbors,4);
    _priv_n.param<bool>("graph_matcher_use_desc_ratio_test",_graph_matcher_use_desc_ratio_test,true);
    _priv_n.param<std::string>("graph_output_path",_graph_output_path,"~/mapping_debug");
  }

  void KeyframeLiveMapper::initGraphDetector() 
  {
    graph_detector_.setNKeypoints(_graph_n_keypoints);
    graph_detector_.setNCandidates(_graph_n_candidates);   
    graph_detector_.setKNearestNeighbors(_graph_k_nearest_neighbors);    
    graph_detector_.setMatcherUseDescRatioTest(_graph_matcher_use_desc_ratio_test);
    
    graph_detector_.setSACReestimateTf(false);
    graph_detector_.setSACSaveResults(true);
    graph_detector_.setVerbose(_verbose);
    graph_detector_.setOutputPath(_graph_output_path);
  }

  void KeyframeLiveMapper::setPublishers()
  {
  }

  void KeyframeLiveMapper::setSubscribers()
  {
    /*
    ImageTransport rgb_it(nh_);
    ImageTransport depth_it(nh_);

    image_transport::TransportHints rgb_th("compressed");
    image_transport::TransportHints depth_th("compressedDepth");


    sub_rgb_.subscribe(rgb_it,     "/rgbd/rgb",   queue_size_, rgb_th);
    sub_depth_.subscribe(depth_it, "/rgbd/depth", queue_size_, depth_th);
    sub_info_.subscribe(nh_,       "/rgbd/info",  queue_size_);

    // Synchronize inputs.
    sync_.reset(new RGBDSynchronizer3(RGBDSyncPolicy3(queue_size_), sub_rgb_, sub_depth_, sub_info_));
     
    sync_->registerCallback(boost::bind(&KeyframeMapper::RGBDCallback, this, _1, _2, _3));  
    */
  }

  void KeyframeMapper::RGBDCallback(const ImageMsg::ConstPtr& rgb_msg, const ImageMsg::ConstPtr& depth_msg, const CameraInfoMsg::ConstPtr& info_msg)
  {
    /*
    tf::StampedTransform transform;

    const ros::Time& time = rgb_msg->header.stamp;
    bool result;

    try {
      tf_listener_.waitForTransform(fixed_frame_, rgb->msg->header.frame_id, time, ros::Duration(0.1));
      tf_listener_.lookupTransform(fixed_frame_,rgb_msg->header.frame_id, time, transform);
    }
    catch(...)
    {
      ROS_WARN("Caught while waiting for tfs");
      return;
    }

    // create a new frame and increment the counter
    rgbdtools::RGBDFrame frame;
    AffineTransform pose;


    // create RGBD data from ROS msgs
    createRGBDFrameFromROSMessages(rgb_msg,depth_msg, info_msg, frame);
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
  //  }
*/
  }

  bool KeyframeLiveMapper::processFrame(const rgbdtools::RGBDFrame& frame, const AffineTransform& pose)
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
  }

  void KeyframeLiveMapper::addKeyframe(const rgbdtools::RGBDFrame& frame, const AffineTransform& pose)
  {
    /*
    rgbdtools::RGBDKeyframe keyframe(frame);
    keyframe.pose = pose;
    
    keyframes_.push_back(keyframe);
    */
  }



