#include "turtlebot_rgbdslam/keyframe_livemapper.h"

namespace turtlebot_rgbdslam {
  void KeyframeLiveMapper::initSolover()
  {
    bool verbose = true;

    _optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
    _optimizer.setVerbose(verbose);
    _linear_solver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
    _solver_ptr    = new g2o::BlockSolverX(&_optimizer, _linear_solver);
    _optimizer.setSolver(_solver_ptr);
    
  }
  void KeyframeLiveMapper::addVertexToOptimizer(Eigen::Affine3f& vertex_pose, int i)
  {
    g2o::SE3Quat   pose;

    generateG2OQuat(pose,vertex_pose);
    addToOptimizer(pose,i);

  }

  void KeyframeLiveMapper::generateG2OQuat(g2o::SE3Quat& pose,Eigen::Affine3f& affine_pose)
  {
    double yaw,pitch,roll;
    yaw   = atan2f(affine_pose(1,0),affine_pose(0,0));
    pitch = asinf(-affine_pose(2,0));
    roll  = atan2f(affine_pose(2,1),affine_pose(2,2));

    g2o::Vector3d t(affine_pose(0,3),affine_pose(1,3),affine_pose(2,3));
    g2o::Quaterniond q;
    q.w()=cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);      
    q.x()=sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2); 
    q.y()=cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);  
    q.z()=cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2); 

    pose = g2o::SE3Quat(q,t);
  }

  void KeyframeLiveMapper::addToOptimizer(g2o::SE3Quat& pose, int index)
  {
    g2o::VertexSE3 *vc = new g2o::VertexSE3();
    vc->estimate() = pose;
    vc->setId(index);

    // set first pose fixed
    if(index == 0)
      vc->setFixed(true);

    _optimizer.addVertex(vc);
  }

  void KeyframeLiveMapper::addEdgeTopOptimizer(rgbdtools::KeyframeAssociation& association)
  {
    g2o::SE3Quat pose;
    generateG2OQuat(pose,association.a2b);

    addToOptimizer(pose,association.it_a->index, association.it_b->index);
  }

  void KeyframeLiveMapper::addToOptimizer(g2o::SE3Quat& pose,int from,int to)
  {
    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
    edge->vertices()[0] = _optimizer.vertex(from);
    edge->vertices()[1] = _optimizer.vertex(to);
    edge->setMeasurement(pose);

    // set the information_atrix
    rgbdtools::InformationMatrix inf;
    edge->setInformation(inf);

    _optimizer.addEdge(edge);
  }

  void KeyframeLiveMapper::optimizeGraph()
  {
    double lambda = 0.01;
    int iteration = 20;
    // preparation
    _optimizer.initializeOptimization();

    // set the initial Levenberg-Marquardt lambda
    _optimizer.setUserLambdaInit(lambda);

    // run optimization
    _optimizer.optimize(iteration);
  }

  void KeyframeLiveMapper::getOptimizedPose(Eigen::Affine3f& pose,int index)
  {
    // Transform the vertex pose from G2O quaternion to Eigen::Matrix4f
    double optimized_pose_quat[7];
    g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(_optimizer.vertex(index));

    vertex->getEstimateData(optimized_pose_quat);

    double qx,qy,qz,qr,qx2,qy2,qz2,qr2; 
    qx = optimized_pose_quat[3];
    qy = optimized_pose_quat[4];
    qz = optimized_pose_quat[5];
    qr = optimized_pose_quat[6];

    qx2 = qx * qx;
    qy2 = qy * qy;
    qz2 = qz * qz;
    qr2 = qr * qr;

    pose(0,0) = qr2+qx2-qy2-qz2;                                                              
    pose(0,1) = 2*(qx*qy-qr*qz);                                                              
    pose(0,2) = 2*(qz*qx+qr*qy);                                                              
    pose(0,3) = optimized_pose_quat[0];                                                       
    pose(1,0) = 2*(qx*qy+qr*qz);                                                              
    pose(1,1) = qr2-qx2+qy2-qz2;                                                              
    pose(1,2) = 2*(qy*qz-qr*qx);                                                              
    pose(1,3) = optimized_pose_quat[1];                                                       
    pose(2,0) = 2*(qz*qx-qr*qy);                                                              
    pose(2,1) = 2*(qy*qz+qr*qx);                                                              
    pose(2,2) = qr2-qx2-qy2+qz2;
    pose(2,3) = optimized_pose_quat[2];                                                       
  }


}
