#include "turtlebot_rgbdslam/keyframe_livemapper.h"

namespace turtlebot_rgbdslam {

  void KeyframeLiveMapper::initFilter()
  {
    _pass.setFilterFieldName("z"); 
    _pass.setFilterLimits(-std::numeric_limits<double>::infinity(), _max_map_z);
  }

  void KeyframeLiveMapper::buildColorOctomap(octomap::ColorOcTree& tree,int current_keyframes_size)
  {
    int i;
    octomap::point3d sensor_origin(0,0,0);
    
    for(i = 0; i < current_keyframes_size; i++)
    {
      const rgbdtools::RGBDKeyframe& keyframe = _keyframes[i];
      octomap::pose6d frame_origin;
      octomap::Pointcloud octomap_cloud;

      PointCloudXYZRGB::Ptr cloud_unf(new PointCloudXYZRGB());
      PointCloudXYZRGB      cloud;
      keyframe.constructDensePointCloud(*cloud_unf, _max_range,_max_stddev);


      // perform filtering for max z
      pcl::transformPointCloud(*cloud_unf,*cloud_unf,keyframe.pose);
      _pass.setInputCloud(cloud_unf);
      _pass.filter(cloud);
      pcl::transformPointCloud(cloud,cloud,keyframe.pose.inverse());

      // build octomap cloud from pcl cloud
      frame_origin = octomap::poseTfToOctomap(ccny_rgbd::tfFromEigenAffine(keyframe.pose));

      // insert points
      buildOctoCloud(octomap_cloud,cloud);

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

      if(!std::isnan(p.z) && !std::isnan(p.x) && !std::isnan(p.y)) octomap_cloud.push_back(p.x,p.y,p.z);
    }
  }

  void KeyframeLiveMapper::insertColor(octomap::ColorOcTree& tree,const PointCloudXYZRGB& cloud,const Eigen::Affine3f pose)
  {
    PointCloudXYZRGB cloud_tf;;
    pcl::transformPointCloud(cloud,cloud_tf, pose);
    for(unsigned int i = 0; i < cloud_tf.points.size(); i++)
    {
      const PointXYZRGB& p = cloud_tf.points[i];

      if(!std::isnan(p.z) && !std::isnan(p.x) && !std::isnan(p.y))
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
}
