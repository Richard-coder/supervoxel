#include "feature.h"

namespace pcl
{
template <typename PointT>
inline void
computePointNormal(const pcl::PointCloud<PointT> &cloud,
                   Eigen::Matrix3f &covariance_matrix,
                   Eigen::Vector4f &plane_parameters, float &curvature)
{
    // Placeholder for the 3x3 covariance matrix at each surface patch
    //EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;

    if (cloud.size() < 3 ||
        computeMeanAndCovarianceMatrix(cloud, covariance_matrix, xyz_centroid) == 0)
    {
        plane_parameters.setConstant(std::numeric_limits<float>::quiet_NaN());
        curvature = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    // Get the plane normal and surface curvature
    solvePlaneParameters(covariance_matrix, xyz_centroid, plane_parameters, curvature);
}
}

void Feature::getLamda(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
      EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
      Eigen::Vector4f normal;
      float curvature;
      pcl::computePointNormal (cloud, covariance_matrix, normal, curvature);

      //std::cout<<covariance_matrix<<std::endl;

      Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);

      Eigen::Matrix3f eigenValue = es.pseudoEigenvalueMatrix();
      Eigen::Matrix3f eigenVector = es.pseudoEigenvectors();

      //std::cout << "The pseudo-eigenvalue matrix eigenvalue is:" << std::endl << eigenValue << std::endl;
      //std::cout << "The pseudo-eigenvector matrix eigenvector is:" << std::endl << eigenVector << std::endl;
      //std::cout << "Finally, V * D * V^(-1) = " << std::endl << eigenVector * eigenValue * eigenVector.inverse() << std::endl;


      lamda_<<eigenValue.coeff(0),eigenValue.coeff(4),eigenValue.coeff(8);

      std::sort(lamda_.data(),lamda_.data()+lamda_.size());

     // std::cout<<lamda_<<std::endl;

     // std::cout<<lamda_[0]<<lamda_[1]<<lamda_[2]<<std::endl;
}


