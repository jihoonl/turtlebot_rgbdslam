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
    _priv_n.param<std::string>("graph_output_path",_graph_output_path,"~/mapping_debug");

    _map_to_odom.setIdentity();
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

  void KeyframeLiveMapper::initFilter()
  {
    _pass.setFilterFieldName("z"); 
    _pass.setFilterLimits(-std::numeric_limits<double>::infinity(), _max_map_z);
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
    bool result;

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
    frame.index = _rgbd_frame_index;
    _rgbd_frame_index++;

    // affine transform from tf msg
    pose = ccny_rgbd::eigenAffineFromTf(transform);

    result = processFrame(frame, pose);
    ROS_INFO("[RGBDSLAM] processing : %s",result?"SUCCESS":"FAILED");
    if(result) addKeyframe(frame, pose); 
  }

  bool KeyframeLiveMapper::processFrame(const rgbdtools::RGBDFrame& frame, const Eigen::Affine3f& pose)
  {
    bool result;      // determine if a new keyframe is needed
    geometry_msgs::PoseStamped frame_pose;   // add the frame pose to the path vector
    tf::Transform frame_tf;
    
    frame_tf = ccny_rgbd::tfFromEigenAffine(pose);
    tf::poseTFToMsg(frame_tf, frame_pose.pose);

    // update the header of the pose for the path
    frame_pose.header.frame_id      = _fixed_frame;
    frame_pose.header.seq           = frame.header.seq;
    frame_pose.header.stamp.sec     = frame.header.stamp.sec;
    frame_pose.header.stamp.nsec    = frame.header.stamp.nsec;

//    _path_msg.poses.push_back(frame_pose);
    
    if(_keyframes.empty())
    {
      result = true;
    }
    else {
      double dist, angle;

      ccny_rgbd::getTfDifference(ccny_rgbd::tfFromEigenAffine(pose),ccny_rgbd::tfFromEigenAffine(_keyframes.back().pose),dist,angle);
      
      if(dist > _kf_dist_eps || angle > _kf_angle_eps)
      {
        result = true;
      }
      else 
      {
        result = false;
      }
    }
    return result;
  }

  void KeyframeLiveMapper::addKeyframe(const rgbdtools::RGBDFrame& frame, const Eigen::Affine3f& pose)
  {
    rgbdtools::RGBDKeyframe keyframe(frame);
    keyframe.pose = pose;
    
    _keyframes.push_back(keyframe);
  }

  void KeyframeLiveMapper::publishMapTransform() 
  {
    ROS_INFO("Initialized map to odom transform sender");
    ros::Rate r(10);

    while(ros::ok() && !(_stop_thread)) {
      tf::StampedTransform transform_msg(_map_to_odom, ros::Time::now(), _fixed_frame, _odom_frame);
      _tf_broadcaster.sendTransform(transform_msg);
      r.sleep();
    }
  }

  void KeyframeLiveMapper::updateOctomap()
  {
    double elapse;
    ROS_INFO("Initialising update octomap thread");

    while(ros::ok() && !(_stop_thread)) {

      if(_keyframes.size() > 15) {
      
        elapse = generateAndSolveGraph();

        ROS_INFO("Graph solve took %.2f",elapse);

        if(_octomap_with_color) {
          octomap::ColorOcTree tree(_octomap_res);
          buildColorOctomap(tree);
          publishOctomap(tree);
        }
        else {
          ROS_WARN("no color octomap is not implemented");
        }
      }

      ros::Duration(1).sleep();
    }
  }

  double KeyframeLiveMapper::generateAndSolveGraph()
  {
    ros::Time begin;
    ros::Time end; 
    
    begin = ros::Time::now(); 
    _associations.clear();
    _graph_detector.generateKeyframeAssociations(_keyframes,_associations);
    _graph_solver.solve(_keyframes,_associations);
    end = ros::Time::now();

    double elapse = (end.toNSec() - begin.toNSec()) * 0.000001;
    return elapse;
  }

  void KeyframeLiveMapper::buildColorOctomap(octomap::ColorOcTree& tree)
  {
    octomap::point3d sensor_origin(0.0,0.0,0.0);

    for(unsigned int kf_idx = 0; kf_idx < _keyframes.size(); kf_idx++)
    {
      const rgbdtools::RGBDKeyframe& keyframe = _keyframes[kf_idx];
      PointCloudXYZRGB::Ptr cloud_unf(new PointCloudXYZRGB());
      PointCloudXYZRGB cloud;
      octomap::pose6d frame_origin;

      // construct the pointcloud
      keyframe.constructDensePointCloud(*cloud_unf,_max_range,_max_stddev);

      // perform filtering for max z
      _pass.setInputCloud(cloud_unf);
      _pass.filter(cloud);
      pcl::transformPointCloud(cloud, cloud, keyframe.pose.inverse());

      frame_origin = octomap::poseTfToOctomap(ccny_rgbd::tfFromEigenAffine(keyframe.pose));

      octomap::Pointcloud octomap_cloud;

      // build octomap cloud from pcl cloud
      buildOctoCloud(octomap_cloud,cloud);

      // insert scan (only xyz)
      tree.insertScan(octomap_cloud, sensor_origin, frame_origin);

      // insert colors
      insertColor(tree,cloud,keyframe.pose);
      
      tree.updateInnerOccupancy();

    }
  }

  void KeyframeLiveMapper::buildOctoCloud(octomap::Pointcloud& octomap_cloud,const PointCloudXYZRGB& cloud)
  {
    for(unsigned int pt_idx = 0; pt_idx < cloud.points.size(); ++pt_idx)
    {
      const PointXYZRGB& p = cloud.points[pt_idx];
      if(!std::isnan(p.z)) octomap_cloud.push_back(p.x,p.y,p.z);
    }
  }

  void KeyframeLiveMapper::insertColor(octomap::ColorOcTree& tree,const PointCloudXYZRGB& cloud,const Eigen::Affine3f pose)
  {
    PointCloudXYZRGB cloud_tf;;
    pcl::transformPointCloud(cloud,cloud_tf, pose);
    for(unsigned int i = 0; i < cloud_tf.points.size(); i++)
    {
      const PointXYZRGB& p = cloud_tf.points[i];
      if(!std::isnan(p.z))
      {
        octomap::point3d endpoint(p.x,p.y,p.z);
        octomap::ColorOcTreeNode* n = tree.search(endpoint);
        if (n) n->setColor(p.r,p.g,p.b);
      }
    }
  }

  void KeyframeLiveMapper::publishOctomap(octomap::ColorOcTree& tree)
  {
    octomap_msgs::Octomap map; 
    map.header.frame_id = _fixed_frame;
    map.header.stamp    = ros::Time::now();

    if(octomap_msgs::binaryMapToMsg(tree,map))
    {
      _pub_octomap.publish(map);
    }
    else
    {
      ROS_ERROR("[RGBDSLAM] Error serializing octomap");
    }
  }


  /*
  octomap::pose6d KeyframeLiveMapper::poseTfToOctomap(tf::Pose pose_tf)
  {
    tf::Point& point_tf = pose_tf.getOrigin();
    tf::Quaternion quat_tf = pose_tf.getRotation();

    octomap::point3d point(point_tf.x(),point_tf.y(),point_tf.z());
    octomath::Quaternion quat(quat_tf.w(),quat_tf.x(),quat_tf.y(),quat_tf.z());

    return octomap::pose6d(point,quat);
  }*/

}
