
#include "turtlebot_rgbdslam/keyframe_livemapper.h"

namespace turtlebot_rgbdslam {
  void KeyframeLiveMapper::initRGBDParams()
  {
    _sac_min_inliers = 20;
    _sac_max_eucl_dist_sq = 0.03 * 0.03;
    _sac_reestimate_tf = false;
                                           
    _ransac_confidence = 0.99;
    _ransac_max_iterations = 1000;
    _ransac_sufficient_inlier_ratio = 0.75;

    _log_one_minus_ransac_confidence = log(1.0 - _ransac_confidence);
  }

  void KeyframeLiveMapper::prepareFeaturesForRANSAC(rgbdtools::RGBDKeyframe& keyframe)
  {
    bool upright = true;
    double min_surf_threshold = 25;
    double surf_threshold = 400;

    cv::SurfDescriptorExtractor extractor;

    while(surf_threshold >= min_surf_threshold) 
    {
      cv::SurfFeatureDetector detector(surf_threshold, 4 , 2, true, upright);
      keyframe.keypoints.clear();
      detector.detect(keyframe.rgb_img, keyframe.keypoints);

      if((int)keyframe.keypoints.size() < _graph_n_keypoints) {
        surf_threshold /= 2.0;
      }
      else {
        keyframe.keypoints.resize(_graph_n_keypoints);
        break;
      }
    }

    extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
    keyframe.computeDistributions();
  }

  void KeyframeLiveMapper::prepareMatcher(rgbdtools::RGBDKeyframe& keyframe)
  {
    int search_param = 32;
    cv::Ptr<cv::flann::IndexParams> index_params;
    cv::Ptr<cv::flann::SearchParams> search_params;

    // build matcher
    index_params  = new cv::flann::KDTreeIndexParams();
    search_params = new cv::flann::SearchParams(search_param);

    keyframe.matcher.reset(new cv::FlannBasedMatcher(index_params, search_params));

    // train
    std::vector<cv::Mat> descriptors_vector;
    descriptors_vector.push_back(keyframe.descriptors);
    keyframe.matcher->add(descriptors_vector);
    keyframe.matcher->train();
  }

  void KeyframeLiveMapper::buildCorrespondenceMatrix(rgbdtools::RGBDKeyframe& keyframe,tbb::concurrent_vector<rgbdtools::RGBDKeyframe>& keyframes,tbb::concurrent_vector<rgbdtools::KeyframeAssociation>& associations)
  {
    tbb::concurrent_vector<rgbdtools::RGBDKeyframe>::iterator current_keyframe_it, it;
    
    current_keyframe_it = keyframes.push_back(keyframe);

    for(it = keyframes.begin() ; it != current_keyframe_it; it++)
    {
      std::vector<cv::DMatch> inlier_matches;

      // perform ransac matching, b onto a
      Eigen::Matrix4f transformation;
      //int iterations = pairwiseMatchingRANSAC(*it, *current_keyframe_it, inlier_matches, transformation);
      pairwiseMatchingRANSAC(*it, *current_keyframe_it, inlier_matches, transformation);

      
      if(inlier_matches.size() >= (unsigned int)_sac_min_inliers) {
        // add an associations
        rgbdtools::KeyframeAssociation association;
        association.type = rgbdtools::KeyframeAssociation::RANSAC;
        association.it_a = current_keyframe_it;
        association.it_b = it;
        association.matches = inlier_matches;

        associations.push_back(association);
      }
    }

  }

  int KeyframeLiveMapper::pairwiseMatchingRANSAC(const rgbdtools::RGBDKeyframe& query,const rgbdtools::RGBDKeyframe& train,rgbdtools::DMatchVector& best_inlier_matches,Eigen::Matrix4f& best_transformation)
  {
    // constants
    int min_sample_size = 3;
    unsigned int i;
    rgbdtools::DMatchVector candidate_matches;
    getCandidateMatches(query,train, candidate_matches);

    // check if enough matches are present
    if(candidate_matches.size() < (unsigned int)min_sample_size)  return 0;
    if(candidate_matches.size() < (unsigned int)_sac_min_inliers) return 0;

    // ROS_INFO("[RGBDSLAM] Candidate matches %d",(int)candidate_matches.size());

    // *** build 3d features for SVD ***
    PointCloudXYZ features_t, features_q;

    features_t.resize(candidate_matches.size());
    features_q.resize(candidate_matches.size());

    for(i =0; i < candidate_matches.size(); i++)
    {
      const cv::DMatch& match = candidate_matches[i];
      int i_query = match.queryIdx;
      int i_train = match.trainIdx;

      PointXYZ& p_t = features_t[i];
      p_t.x = train.kp_means[i_train](0,0);
      p_t.y = train.kp_means[i_train](1,0);
      p_t.z = train.kp_means[i_train](2,0);

      PointXYZ& p_q = features_q[i];
      p_q.x = query.kp_means[i_query](0,0);
      p_q.y = query.kp_means[i_query](1,0);
      p_q.z = query.kp_means[i_query](2,0);
    }

    // *** main RANSAC loop ***
    pcl::registration::TransformationEstimationSVD<PointXYZ,PointXYZ> svd;
    Eigen::Matrix4f transformation;
    best_inlier_matches.clear();
    int iteration = 0;

    std::set<int> mask;
    while(ros::ok())
    {
      // generate random indices
      std::vector<int> i_sample;
      rgbdtools::get3RandomIndices(candidate_matches.size(), mask, i_sample);

      // build initial inliers from random indices
      std::vector<int> i_inlier;
      std::vector<cv::DMatch> inlier_matches;

      for(i = 0; i < i_sample.size(); i ++)
      {
        int i_m = i_sample[i];
        i_inlier.push_back(i_m);
        inlier_matches.push_back(candidate_matches[i_m]);
      }

      // estimate transformation from minimum set of random samples
      svd.estimateRigidTransformation(features_q,i_inlier,features_t,i_inlier,transformation);

      // evaluate transformation fitness by checking distance to all points
      PointCloudXYZ features_q_tf;
      pcl::transformPointCloud(features_q,features_q_tf,transformation);

      for(i = 0; i < candidate_matches.size(); i++)
      {
        // euclidean distance test
        const PointXYZ& p_train = features_t[i];
        const PointXYZ& p_query = features_q_tf[i];
        float eucl_dist_sq = rgbdtools::distEuclideanSq(p_train,p_query);

        if(eucl_dist_sq < _sac_max_eucl_dist_sq)
        {
          i_inlier.push_back(i);
          inlier_matches.push_back(candidate_matches[i]);

          // reestimate transformation from all inliers
          if(_sac_reestimate_tf)
          {
            svd.estimateRigidTransformation(features_q,i_inlier,features_t,i_inlier,transformation);
            pcl::transformPointCloud(features_q,features_q_tf,transformation);
          }
        }
      }

      // check if inliers are better than the best model so far
      if(inlier_matches.size() > best_inlier_matches.size())
      {
        svd.estimateRigidTransformation(features_q,i_inlier,features_t,i_inlier,transformation);
        best_transformation = transformation;
        best_inlier_matches = inlier_matches;
      }

      double best_inlier_ratio = (double)best_inlier_matches.size() / (double)candidate_matches.size();

      // ** termination : iterations + inlier ratio
      if(best_inlier_matches.size() < _sac_min_inliers)
      {
        if(iteration >= _ransac_max_iterations) break;
      }
      else 
      {
        double h = _log_one_minus_ransac_confidence / log(1.0 - pow(best_inlier_ratio, min_sample_size));

        if(iteration > (int)(h+1)) break;
      }
      iteration++;

    }
    
    return iteration;
  }

  void KeyframeLiveMapper::getCandidateMatches(const rgbdtools::RGBDKeyframe& query,const rgbdtools::RGBDKeyframe& train, rgbdtools::DMatchVector& candidate_matches)
  {
    // *** build candidate matches ***
    // asumes detectors and distributions are computed
    // establish all matches from b to a
    unsigned int i;

    if(_graph_matcher_use_desc_ratio_test)
    {
      std::vector<rgbdtools::DMatchVector> all_matches2;

      train.matcher->knnMatch(query.descriptors, all_matches2, 2);

      for(i= 0; i < all_matches2.size(); i++)
      {
        const cv::DMatch& match1 = all_matches2[i][0];
        const cv::DMatch& match2 = all_matches2[i][1];

        double ratio = match1.distance / match2.distance;

        // remove bad matches - ratio test, valid keypoints
        if(ratio < _graph_matcher_max_desc_ratio) 
        {
          int i_query = match1.queryIdx;
          int i_train = match1.trainIdx;

          if(train.kp_valid[i_train] && query.kp_valid[i_query]) candidate_matches.push_back(match1);
        }
      }
    }
    else {
      rgbdtools::DMatchVector all_matches;
      
      train.matcher->match(query.descriptors, all_matches);

      for(i = 0; i < all_matches.size(); i++)
      {
        const cv::DMatch& match = all_matches[i];

        // remove bad matches - descriptor distance, valid keypoints
        if(match.distance < _graph_matcher_max_desc_dist)
        {
          int i_query = match.queryIdx;
          int i_train = match.trainIdx;

          if(train.kp_valid[i_train] && query.kp_valid[i_query]) candidate_matches.push_back(match);
        }
      }

    }
  }
}
