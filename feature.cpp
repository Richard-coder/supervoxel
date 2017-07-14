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
    for (size_t i=0; normal_itr != normals.end(); normal_itr++,i++)
    {
        angels[i]=-normal_itr->normal_y;
        //测试一下法向量是否已经归一化，即其模是否为1，验证后，已经归一化了
        // std::cout<<normal_itr->normal_x*normal_itr->normal_x+normal_itr->normal_y*normal_itr->normal_y+normal_itr->normal_z*normal_itr->normal_z<<std::endl;
    }

    //计算均值与方差
    angel_ = compute(angels);
std::cout << "统计计算点的个数"<<":\t"<<angels.size() << std::endl;
    std::cout << "角度的均值和方差"<<":\t"<<angel_ << std::endl;
        std::cout<<"输出角度序列"<<std::endl;
    for(int i=0;i<angels.size();i++){
            std::cout<<angels[i]<<std::endl;
    }
}
void Feature::getHeight(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
    std::vector<float> heights;
    heights.resize(cloud.size());
    typename pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator cloud_itr = cloud.begin();
    for (size_t i=0; cloud_itr != cloud.end(); cloud_itr++,i++)
    {
        heights[i]=-cloud_itr->y;
    }
    std::sort(heights.begin(), heights.end());
    height_ << heights[0], heights[(heights.size() - 1) / 2], heights[heights.size() - 1];

    //测试计算结果
    std::cout<<"高度的结果：\t"<<height_<<std::endl;
    std::cout<<"计算点的个数：\t"<<heights.size()<<std::endl;
    std::cout<<"输出高度序列"<<std::endl;
    for(int i=0;i<heights.size();i++){
            std::cout<<heights[i]<<std::endl;
    }
    
}

void Feature::getLab(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
    std::map<char, std::vector<float>> lab_map;
    lab_map['l'].resize(cloud.size());
    lab_map['a'].resize(cloud.size());
    lab_map['b'].resize(cloud.size());
    Eigen::Vector3f singel_lab;
    typename pcl::PointCloud<pcl::PointXYZRGBA>::const_iterator cloud_itr = cloud.begin();
    for (size_t i=0; cloud_itr != cloud.end(); cloud_itr++,i++)
    {
        singel_lab = RGB2Lab(cloud_itr->getRGBVector3i());
        lab_map['l'][i]=singel_lab[0];
       lab_map['a'][i]=singel_lab[1];
        lab_map['b'][i]=singel_lab[2];

        std::cout <<i<<"\t" <<singel_lab[0] << std::endl;
    }
    Eigen::Vector2f L = compute(lab_map['l']);
    Eigen::Vector2f A = compute(lab_map['a']);
    Eigen::Vector2f B = compute(lab_map['b']);

    lab_ << L[0], A[0], B[0], L[1], A[1], B[1];
    std::cout <<"输出计算点的个数：\t"<< lab_map['l'].size()<< std::endl;
    std::cout  <<"计算的颜色均值和方差：\t"<< lab_ << std::endl;
}

Eigen::Vector3f
Feature::RGB2Lab(const Eigen::Vector3i &colorRGB)
{
    // for sRGB   -> CIEXYZ see http://www.easyrgb.com/index.php?X=MATH&H=02#text2
    // for CIEXYZ -> CIELAB see http://www.easyrgb.com/index.php?X=MATH&H=07#text7

    double R, G, B, X, Y, Z;

    // map sRGB values to [0, 1]
    R = colorRGB[0] / 255.0;
    G = colorRGB[1] / 255.0;
    B = colorRGB[2] / 255.0;

    // linearize sRGB values
    if (R > 0.04045)
        R = pow((R + 0.055) / 1.055, 2.4);
    else
        R = R / 12.92;

    if (G > 0.04045)
        G = pow((G + 0.055) / 1.055, 2.4);
    else
        G = G / 12.92;

    if (B > 0.04045)
        B = pow((B + 0.055) / 1.055, 2.4);
    else
        B = B / 12.92;

    // postponed:
    //    R *= 100.0;
    //    G *= 100.0;
    //    B *= 100.0;

    // linear sRGB -> CIEXYZ
    X = R * 0.4124 + G * 0.3576 + B * 0.1805;
    Y = R * 0.2126 + G * 0.7152 + B * 0.0722;
    Z = R * 0.0193 + G * 0.1192 + B * 0.9505;

    // *= 100.0 including:
    X /= 0.95047; //95.047;
    //    Y /= 1;//100.000;
    Z /= 1.08883; //108.883;

    // CIEXYZ -> CIELAB
    if (X > 0.008856)
        X = pow(X, 1.0 / 3.0);
    else
        X = 7.787 * X + 16.0 / 116.0;

    if (Y > 0.008856)
        Y = pow(Y, 1.0 / 3.0);
    else
        Y = 7.787 * Y + 16.0 / 116.0;

    if (Z > 0.008856)
        Z = pow(Z, 1.0 / 3.0);
    else
        Z = 7.787 * Z + 16.0 / 116.0;

    Eigen::Vector3f colorLab;
    colorLab[0] = static_cast<float>(116.0 * Y - 16.0);
    colorLab[1] = static_cast<float>(500.0 * (X - Y));
    colorLab[2] = static_cast<float>(200.0 * (Y - Z));

    return colorLab;
}

Eigen::Vector2f
Feature::compute(const std::vector<float> &vec_in)
{
    Eigen::Vector2f res;

    double sum = std::accumulate(std::begin(vec_in), std::end(vec_in), 0.0);
    double mean = sum / vec_in.size(); //均值

    double accum = 0.0;
    std::for_each(std::begin(vec_in), std::end(vec_in), [&](const double d) {
        accum += (d - mean) * (d - mean);
    });
    res[0] = static_cast<float>(mean);
    res[1] = static_cast<float>(sqrt(accum / (vec_in.size() - 1))); //方差

    return res;
}


