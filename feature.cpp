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
    pcl::computePointNormal(cloud, covariance_matrix, normal, curvature);

    //std::cout<<covariance_matrix<<std::endl;

    Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);

    Eigen::Matrix3f eigenValue = es.pseudoEigenvalueMatrix();
    Eigen::Matrix3f eigenVector = es.pseudoEigenvectors();

    //std::cout << "The pseudo-eigenvalue matrix eigenvalue is:" << std::endl << eigenValue << std::endl;
    //std::cout << "The pseudo-eigenvector matrix eigenvector is:" << std::endl << eigenVector << std::endl;
    //std::cout << "Finally, V * D * V^(-1) = " << std::endl << eigenVector * eigenValue * eigenVector.inverse() << std::endl;

    lamda_ << eigenValue.coeff(0), eigenValue.coeff(4), eigenValue.coeff(8);

    //将特征值按非降序排列
    std::sort(lamda_.data(), lamda_.data() + lamda_.size());

    // std::cout<<lamda_<<std::endl;

    // std::cout<<lamda_[0]<<lamda_[1]<<lamda_[2]<<std::endl;
}
/*
 * 计算超体素点云中每个点的法向量与水平面的夹角
 * 由于opencv与pcl坐标表示方式不同，我用高翔代码生成的点云，其z轴的法向量是[0, -1, 0]
 * 这里的超体素点云的法向量是提取过程中已经计算好的并且是归一化的
 * 因此，这里计算其与z轴的夹角arcos( dot(mormal, [0, -1, 0]) ),计算结果其实就是法向量y的值的反余弦，值的范围是0-pi
 * 但这里为了计算方便，不进行反余弦操作
*/
void Feature::getAngel(const pcl::PointCloud<pcl::Normal> &normals)
{
    typename pcl::PointCloud<pcl::Normal>::const_iterator normal_itr = normals.begin();
    std::vector<float> angels;
    angels.resize(normals.size());
    for (; normal_itr != normals.end(); normal_itr++)
    {
        angels.push_back(-normal_itr->normal_y);
        //测试一下法向量是否已经归一化，即其模是否为1，验证后，已经归一化了
        // std::cout<<normal_itr->normal_x*normal_itr->normal_x+normal_itr->normal_y*normal_itr->normal_y+normal_itr->normal_z*normal_itr->normal_z<<std::endl;
    }

    //计算均值与方差
    double sum = std::accumulate(std::begin(angels), std::end(angels), 0.0);
    double mean = sum / angels.size(); //均值

    double accum = 0.0;
    std::for_each(std::begin(angels), std::end(angels), [&](const double d) {
        accum += (d - mean) * (d - mean);
    });

    double stdev = sqrt(accum / (angels.size() - 1)); //方差

    angel_ << mean, stdev;

    //std::cout << angel_ << std::endl;
}
void Feature::getHeight(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
    std::vector<float> heights;
    heights.resize(cloud.size());
    typename pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator cloud_itr = cloud.begin();
    for (; cloud_itr != cloud.end(); cloud_itr++){
        heights.push_back(-cloud_itr->y);
    }
    std::sort(heights.begin(), heights.end());
    height_<<heights[0],heights[(heights.size()-1)/2],heights[heights.size()-1];

    /*测试计算结果
    std::cout<<height_<<std::endl;
    std::cout<<heights.size()<<std::endl;
    std::cout<<"输出高度序列"<<std::endl;
    for(int i=0;i<heights.size();i++){
            std::cout<<heights[i]<<std::endl;
    }
    */


}
