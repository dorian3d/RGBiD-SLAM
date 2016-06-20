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

#ifndef PnPMANAGER_HPP_
#define PnPMANAGER_HPP_

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/io/ply_io.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <vector>
#include <deque>
//#include <boost/thread/thread.hpp>

#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/parameter_camera.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/edge_pointxyz.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/edge_se3_pointxyz_projection.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/batch_stats.h>
#include <g2o/core/robust_kernel_impl.h>

#include "keyframe.h"
#include "settings.h"

#include "types.h"

namespace RGBID_SLAM 
{      
  class PnPGraphOptimiser
  {
    public:

      PnPGraphOptimiser(/*TODO: arguments*/);
      ~PnPGraphOptimiser(/*TODO: arguments*/);
      
      void loadSettings(const Settings& settings);    
            
      void buildGraph3Dto2D(const Eigen::Matrix3d K, const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > &points3D_a,
                     const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > &projections2D_b, 
                     const Eigen::Affine3d& aTb_init,
                     const std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > &cov3D_a,
                     const std::vector<Eigen::Matrix2d,Eigen::aligned_allocator<Eigen::Matrix2d> > &cov2D_b); 
                     
       void buildGraph3Dto3D (const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > &points3D_a,
                               const std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > &points3D_b, 
                               const Eigen::Affine3d& aTb_init,
                               const std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > &cov3D_a,
                               const std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > &cov3D_b);   
                                                                      
      void optimiseGraphAndGetPnPTrafo (Eigen::Affine3d& pose_final, Eigen::Matrix<double, 6, 6>& cov_pose) ;
      
      int trafo_id_;
      int dummy_id_;
    
    private:  
    
      typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  LoopBlockSolver;
      
      typedef g2o::LinearSolverCSparse<LoopBlockSolver::PoseMatrixType> LoopLinearSolver;        
      //typedef g2o::LinearSolverPCG<LoopBlockSolver::PoseMatrixType> LoopLinearSolver;
      //typedef g2o::LinearSolverCholmod<LoopBlockSolver::PoseMatrixType> LoopLinearSolver;          
        
      //typedef g2o::OptimizationAlgorithmLevenberg OptimAlg;
      typedef g2o::OptimizationAlgorithmGaussNewton OptimAlg;
      //typedef g2o::OptimizationAlgorithmDogleg OptimAlg;
      
      typedef LoopBlockSolver* LoopBlockSolverPtr;
      typedef LoopLinearSolver* LoopLinearSolverPtr;
      typedef OptimAlg* OptimAlgPtr;
      
      g2o::SparseOptimizer optimizer_;	
      LoopLinearSolverPtr linearSolver_ ;
      LoopBlockSolverPtr blockSolver_ ;
      OptimAlgPtr algSolver_;
      
      bool multilevel_optim_flag_;
      
      g2o::BatchStatisticsContainer  batch_stats_;
      
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  
}

#endif /* PnPMANAGER_HPP_ */
