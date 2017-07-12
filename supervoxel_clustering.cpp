#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <vtkPolyLine.h>
// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
/*
void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                       PointCloudT &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);
*/


  namespace pcl{

inline void
solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix,
                            Eigen::Vector3f::Scalar &eigen_value,
                           float &nx, float &ny, float &nz, float &curvature)
{
  // Avoid getting hung on Eigen's optimizers
//  for (int i = 0; i < 9; ++i)
//    if (!pcl_isfinite (covariance_matrix.coeff (i)))
//    {
//      //PCL_WARN ("[pcl::solvePlaneParameteres] Covariance matrix has NaN/Inf values!\n");
//      nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
//      return;
//    }
  // Extract the smallest eigenvalue and its eigenvector

  //EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);

  nx = eigen_vector [0];
  ny = eigen_vector [1];
  nz = eigen_vector [2];

  // Compute the curvature surface change
  float eig_sum = covariance_matrix.coeff (0) + covariance_matrix.coeff (4) + covariance_matrix.coeff (8);
/*
  std::cout<<covariance_matrix.coeff (0)<<"\t"<<covariance_matrix.coeff (4)<<"\t"<<covariance_matrix.coeff (8)<<"\t"<<std::endl<<std::endl;
  std::cout<<covariance_matrix<<std::endl<<std::endl;
  */
  if (eig_sum != 0)
    curvature = fabsf (eigen_value / eig_sum);
  else
    curvature = 0;
}

inline void
solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix,
                            Eigen::Vector3f::Scalar &eigen_value,
                           const Eigen::Vector4f &point,
                           Eigen::Vector4f &plane_parameters, float &curvature)
{
  solvePlaneParameters (covariance_matrix, eigen_value, plane_parameters [0], plane_parameters [1], plane_parameters [2], curvature);

  plane_parameters[3] = 0;
  // Hessian form (D = nc . p_plane (centroid here) + p)
  plane_parameters[3] = -1 * plane_parameters.dot (point);
}


  template <typename PointT> inline void
  computePointNormal (const pcl::PointCloud<PointT> &cloud,
                      Eigen::Matrix3f &covariance_matrix,
                      Eigen::Vector3f::Scalar &eigen_value,
                      Eigen::Vector4f &plane_parameters, float &curvature)
  {
    // Placeholder for the 3x3 covariance matrix at each surface patch
    //EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;

    if (cloud.size () < 3 ||
        computeMeanAndCovarianceMatrix (cloud, covariance_matrix, xyz_centroid) == 0)
    {
      plane_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
      curvature = std::numeric_limits<float>::quiet_NaN ();
      return;
    }

    // Get the plane normal and surface curvature
    solvePlaneParameters (covariance_matrix, eigen_value, xyz_centroid, plane_parameters, curvature);
  }
}

int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    pcl::console::print_error ("Syntax is: %s <pcd-file> \n "
                                "--NT Dsables the single cloud transform \n"
                                "-v <voxel resolution>\n-s <seed resolution>\n"
                                "-c <color weight> \n-z <spatial weight> \n"
                                "-n <normal_weight>\n", argv[0]);
    return (1);
  }


  PointCloudT::Ptr cloud = boost::make_shared <PointCloudT> ();
  pcl::console::print_highlight ("Loading point cloud...\n");
  if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud))
  {
    pcl::console::print_error ("Error loading cloud file!\n");
    return (1);
  }

//与点云是否为结构点云有关
  bool use_transform = ! pcl::console::find_switch (argc, argv, "--NT");
//体素的大小,单位米
  float voxel_resolution = 0.008f;
  bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
  if (voxel_res_specified)
    pcl::console::parse (argc, argv, "-v", voxel_resolution);
//超像素的大小,单位米
  float seed_resolution = 0.1f;
  bool seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
  if (seed_res_specified)
    pcl::console::parse (argc, argv, "-s", seed_resolution);
//颜色对于超体聚类的影响
  float color_importance = 0.2f;
  if (pcl::console::find_switch (argc, argv, "-c"))
    pcl::console::parse (argc, argv, "-c", color_importance);
//设置空间形状的影响,其值越大,超体素形状越规则,其值越小,超体素形状越不规则,受法向量和颜色的影响越大.
  float spatial_importance = 0.4f;
  if (pcl::console::find_switch (argc, argv, "-z"))
    pcl::console::parse (argc, argv, "-z", spatial_importance);
//设置法向量的影响
  float normal_importance = 1.0f;
  if (pcl::console::find_switch (argc, argv, "-n"))
    pcl::console::parse (argc, argv, "-n", normal_importance);

  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution, use_transform);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
                                                                                                                                                                                                                                                                  
  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
/*
//显示原始的点云
  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");
/*
//为每一块超体素用不同的颜色区分，注释掉后会显示原始的点云
 /* PointCloudT::Ptr colored_voxel_cloud = super.getColoredVoxelCloud ();
  viewer->addPointCloud (colored_voxel_cloud, "colored voxels");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "colored voxels");
*/
  PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  //显示超体素中心点的法向量
  //viewer->addPointCloudNormals<PointNT> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr >::iterator sv_itr=supervoxel_clusters.begin();
  for(;sv_itr!=supervoxel_clusters.end();sv_itr++){
    uint32_t label=sv_itr->first;
    if((*(supervoxel_clusters[label]->voxels_)).size()>20){
      EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
      EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
      Eigen::Vector4f normal_;
      float curvature_;
      pcl::computePointNormal (*(supervoxel_clusters[label]->voxels_), covariance_matrix, eigen_value, normal_, curvature_);
      
      std::cout<<covariance_matrix<<std::endl;
      /*
      std::cout<<normal_<<std::endl;
      std::cout<<curvature_<<std::endl;
      */
      std::cout<<eigen_value<<std::endl;

    Eigen::EigenSolver<Eigen::Matrix3f> es(covariance_matrix);

    Eigen::Matrix3f D = es.pseudoEigenvalueMatrix();
    Eigen::Matrix3f V = es.pseudoEigenvectors();
    std::cout << "The pseudo-eigenvalue matrix D is:" << std::endl << D << std::endl;
    std::cout << "The pseudo-eigenvector matrix V is:" << std::endl << V << std::endl;
    std::cout << "Finally, V * D * V^(-1) = " << std::endl << V * D * V.inverse() << std::endl;


      break;
    }
    //显示点云
    std::stringstream ss;
    ss << label;
    PointCloudT::Ptr voxel_cloud = supervoxel_clusters[label]->voxels_;
    if((*(supervoxel_clusters[label]->voxels_)).size()>3){
      viewer->addPointCloud(voxel_cloud, ss.str());
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, ss.str());
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, ss.str());
    }
    ///std::cout<<(*(supervoxel_clusters[label]->voxels_)).size()<<std::endl;
    //viewer->spinOnce (50);
  }
/*
  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
  std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
  for ( ; label_itr != supervoxel_adjacency.end (); )
  {
    //First get the label
    uint32_t supervoxel_label = label_itr->first;
    //Now get the supervoxel corresponding to the label
    pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

    //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
    PointCloudT adjacent_supervoxel_centers;
    std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
    for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
    {
      pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
      adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
    }
    //Now we make a name for this polygon
    std::stringstream ss;
    ss << "supervoxel_" << supervoxel_label;
    //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
    addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
    //Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
  }
*/
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
  }
  return (0);
}


/*
void
addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

  //Iterate through all adjacent points, and add a center point to adjacent point pair
  PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
  for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
  {
    points->InsertNextPoint (supervoxel_center.data);
    points->InsertNextPoint (adjacent_itr->data);
  }
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
  // Add the points to the dataset
  polyData->SetPoints (points);
  polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
  for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    polyLine->GetPointIds ()->SetId (i,i);
  cells->InsertNextCell (polyLine);
  // Add the lines to the dataset
  polyData->SetLines (cells);
  viewer->addModelFromPolyData (polyData,supervoxel_name);
}
*/


