/**
* This file is part of RGBID-SLAM.
*
* Copyright (C) 2015 Daniel Gutiérrez Gómez <danielgg at unizar dot es> (Universidad de Zaragoza)
*
* RGBID-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* RGBID-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with RGBID-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/


#include "PnP_manager.h"
#include <g2o/core/sparse_block_matrix.h>

RGBID_SLAM::PnPGraphOptimiser::PnPGraphOptimiser()
{
  linearSolver_ = (new LoopLinearSolver);
  blockSolver_ = (new LoopBlockSolver(linearSolver_));
  algSolver_ = (new OptimAlg(blockSolver_));
  
  //algSolver_->setUserLambdaInit(1e-14);
  optimizer_.setAlgorithm(algSolver_);
  optimizer_.setVerbose(true);
  
  multilevel_optim_flag_ = true;
  trafo_id_ = 0;
  dummy_id_ = 0;
}

RGBID_SLAM::PnPGraphOptimiser::~PnPGraphOptimiser()
{
  std::cout << "  reset optimizer" << std::endl;
  optimizer_.clear();
}

void 
RGBID_SLAM::PnPGraphOptimiser::loadSettings(const Settings& settings)
{
  Section PnP_section;
  
  if (settings.getSection("PnP", PnP_section))
  {
    std::cout << "PnP" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    
    Entry entry;
    
    std::cout << std::endl;
    std::cout << std::endl;
  }  
}  

void
RGBID_SLAM::PnPGraphOptimiser::buildGraph3Dto2D(const Eigen::Matrix3d K, const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > &points3D_a,
                                               const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > &projections2D_b, 
                                               const Eigen::Affine3d& aTb_init,
                                               const std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > &cov3D_a,
                                               const std::vector<Eigen::Matrix2d,Eigen::aligned_allocator<Eigen::Matrix2d> > &cov2D_b)   
{
  optimizer_.clear();  
  
  g2o::ParameterCamera* camera = new g2o::ParameterCamera;
  camera->setKcam(K(0,0),K(1,1),K(0,2),K(1,2));
  camera->setId(0);
  optimizer_.addParameter(camera);
  
  g2o::SE3Quat trafo(aTb_init.linear().cast<double>(), aTb_init.translation().cast<double>());
  
  Eigen::Vector3d dummy_point = Eigen::Vector3d(0.0,0.0,0.0);
  
  int vertex_id = 0;
  
  {
    g2o::VertexPointXYZ* vXYZ = new g2o::VertexPointXYZ();
    vXYZ->setEstimate(dummy_point);
    vXYZ->setId(vertex_id);
    vXYZ->setFixed(true);
    optimizer_.addVertex(vXYZ);
    dummy_id_ = vertex_id;
    vertex_id++;
  }
  
  {
    g2o::VertexSE3* vSE3 = new g2o::VertexSE3();
    vSE3->setEstimate(trafo);
    vSE3->setId(vertex_id);    
    optimizer_.addVertex(vSE3);
    trafo_id_ = vertex_id;
    vertex_id++;
  }
  
  //COPY VERTICES OF 3D POINTS
  for (int i=0; i<points3D_a.size(); i++) 
  {    
    Eigen::Vector3d yW = points3D_a[i].cast<double>();
    Eigen::Vector2d z_obs = projections2D_b[i].cast<double>();
    
    g2o::VertexPointXYZ* vXYZ = new g2o::VertexPointXYZ();
    vXYZ->setEstimate(yW);
    vXYZ->setId(vertex_id);
    vXYZ->setFixed(false);
    optimizer_.addVertex(vXYZ);
    
    g2o::EdgeSE3PointXYZProjection* eSE3_XYZ = new g2o::EdgeSE3PointXYZProjection();
    eSE3_XYZ->vertices()[0] = optimizer_.vertex(trafo_id_);
    eSE3_XYZ->vertices()[1] = optimizer_.vertex(vertex_id);
    
    eSE3_XYZ->setMeasurement(z_obs);
    
    eSE3_XYZ->setInformation(cov2D_b[i].inverse().cast<double>());
    eSE3_XYZ->setParameterId(0,camera->id());
    
    //g2o::RobustKernelHuber* rk_SE3_XYZ = new g2o::RobustKernelHuber;
    //eSE3_XYZ->setRobustKernel(rk_SE3_XYZ);
    //eSE3_XYZ->robustKernel()->setDelta(1.345);
            
    optimizer_.addEdge(eSE3_XYZ);    
    
    g2o::EdgePointXYZ* eXYZ = new g2o::EdgePointXYZ();
    eXYZ->vertices()[0] = optimizer_.vertex(dummy_id_);
    eXYZ->vertices()[1] = optimizer_.vertex(vertex_id);
    
    eXYZ->setMeasurement(yW);
    eXYZ->setInformation(cov3D_a[i].inverse().cast<double>());
    
    //g2o::RobustKernelHuber* rk_XYZ = new g2o::RobustKernelHuber;
    //eXYZ->setRobustKernel(rk_XYZ);
    //eXYZ->robustKernel()->setDelta(1.345);
    
    optimizer_.addEdge(eXYZ);   
    vertex_id++;
  }
}


void
RGBID_SLAM::PnPGraphOptimiser::buildGraph3Dto3D(const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > &points3D_a,
                                               const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > &points3D_b, 
                                               const Eigen::Affine3d& aTb_init,
                                               const std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > &cov3D_a,
                                               const std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > &cov3D_b)   
{
  std::cout << "building graph" << std::endl;
  optimizer_.clear(); 
  
  g2o::ParameterSE3Offset* offset = new g2o::ParameterSE3Offset;
  offset->setId(0);
  optimizer_.addParameter(offset);
  
  g2o::SE3Quat trafo(aTb_init.linear().cast<double>(), aTb_init.translation().cast<double>());
  
  Eigen::Vector3d dummy_point = Eigen::Vector3d(0.0,0.0,0.0);
  
  int vertex_id = 0;
  
  {
    g2o::VertexPointXYZ* vXYZ = new g2o::VertexPointXYZ();
    vXYZ->setEstimate(dummy_point);
    vXYZ->setId(vertex_id);
    vXYZ->setFixed(true);
    optimizer_.addVertex(vXYZ);
    dummy_id_ = vertex_id;
    vertex_id++;
  }
  
  {
    g2o::VertexSE3* vSE3 = new g2o::VertexSE3();
    vSE3->setEstimate(trafo);
    vSE3->setId(vertex_id);    
    optimizer_.addVertex(vSE3);
    trafo_id_ = vertex_id;
    vertex_id++;
  }
  
  //COPY VERTICES OF 3D POINTS
  for (int i=0; i<points3D_a.size(); i++) 
  {    
    Eigen::Vector3d yA = points3D_a[i].cast<double>();
    Eigen::Vector3d yB = points3D_b[i].cast<double>();
    
    g2o::VertexPointXYZ* vXYZ = new g2o::VertexPointXYZ();
    vXYZ->setEstimate(yA);
    vXYZ->setId(vertex_id);
    vXYZ->setFixed(false);
    optimizer_.addVertex(vXYZ);
    
    g2o::EdgeSE3PointXYZ* eSE3_XYZ = new g2o::EdgeSE3PointXYZ();
    eSE3_XYZ->vertices()[0] = optimizer_.vertex(trafo_id_);
    eSE3_XYZ->vertices()[1] = optimizer_.vertex(vertex_id);
    
    eSE3_XYZ->setMeasurement(yB);
    
    eSE3_XYZ->setInformation(cov3D_b[i].inverse().cast<double>());
    eSE3_XYZ->setParameterId(0,offset->id());
    
    g2o::RobustKernelHuber* rk_SE3_XYZ = new g2o::RobustKernelHuber;
    eSE3_XYZ->setRobustKernel(rk_SE3_XYZ);
    eSE3_XYZ->robustKernel()->setDelta(1.345);
            
    optimizer_.addEdge(eSE3_XYZ);  
    
    g2o::EdgePointXYZ* eXYZ = new g2o::EdgePointXYZ();
    eXYZ->vertices()[0] = optimizer_.vertex(dummy_id_);
    eXYZ->vertices()[1] = optimizer_.vertex(vertex_id);
    
    eXYZ->setMeasurement(yA);
    eXYZ->setInformation(cov3D_a[i].inverse().cast<double>());
    
    g2o::RobustKernelHuber* rk_XYZ = new g2o::RobustKernelHuber;
    eXYZ->setRobustKernel(rk_XYZ);
    eXYZ->robustKernel()->setDelta(1.345);
    
    optimizer_.addEdge(eXYZ);   
    vertex_id++;
  }
}

void
RGBID_SLAM::PnPGraphOptimiser::optimiseGraphAndGetPnPTrafo(Eigen::Affine3d& pose_final, Eigen::Matrix<double, 6, 6>& cov_pose)
{
  //Optimise in all levels
  std::cout << "optmising PnP graph" << std::endl;
  optimizer_.initializeOptimization(-1); //negative level->use edges in all levels
  optimizer_.optimize(10);
  
  g2o::VertexSE3* vSE3 = dynamic_cast<g2o::VertexSE3*>(optimizer_.vertex(trafo_id_));
  if (vSE3 != 0)
  {
    pose_final.linear() = vSE3->estimate().rotation();
    pose_final.translation() = vSE3->estimate().translation();    
    
    g2o::SparseBlockMatrix<> spinv;
    optimizer_.computeMarginals(spinv, vSE3);
    
    Eigen::Matrix<double, 6, 6> cov_g2o = *(spinv.block(vSE3->hessianIndex(),vSE3->hessianIndex()));
    
    cov_pose.block<3,3>(0,0) = cov_g2o.block<3,3>(3,3);
    cov_pose.block<3,3>(3,0) = cov_g2o.block<3,3>(0,3);
    cov_pose.block<3,3>(0,3) = cov_g2o.block<3,3>(3,0);
    cov_pose.block<3,3>(3,3) = cov_g2o.block<3,3>(0,0);
    
    std::cout << "Covariance of loop constraint" << std::endl;
    std::cout << cov_pose << std::endl;
    //(*kf_it)->computeAlignedPointCloud((*kf_it)->pose_);
  }
}
  

