#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/boundary.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/concave_hull.h>

int main (int argc, char** argv){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;

  reader.read ("/home/avell/Desktop/point_cloud/patchs/cloud_cluster_0.pcd", *cloud);

  // Create a set of planar coefficients with X=Y=0,Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 1;
  coefficients->values[2] = 0;
  coefficients->values[3] = 0;

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);


//  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
//  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
//  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
//  normal_estimator.setSearchMethod (tree);
//  normal_estimator.setInputCloud (cloud_projected);
//  normal_estimator.setKSearch (50);
//  normal_estimator.compute (*normals);
//
//  pcl::PointCloud<pcl::Boundary> boundaries;
//  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
//  est.setInputCloud (cloud);
//  est.setInputNormals (normals);
//  est.setRadiusSearch (0.3);   // 2cm radius
//  est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
//  est.setAngleThreshold(M_PI/4);
//  est.compute (boundaries);
//
//  //get points which on the boundary form point cloud;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
//  for(int i=0;i<cloud->points.size();i++)  {
//          if(boundaries[i].boundary_point==1)     {
//             boundaryCloud->points.push_back(cloud->points[i]);
//     }
//  }
//  boundaryCloud->width=boundaryCloud->points.size();
//  boundaryCloud->height=1;
//  boundaryCloud->is_dense=true;

  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (.2);
  chull.reconstruct (*cloud_hull);

//  pcl::PCDWriter writer;
//  writer.write<pcl::PointXYZ> ("/home/avell/Desktop/boundary.pcd", *boundaryCloud, false);


  pcl::visualization::CloudViewer viewer ("Visualizer");
  viewer.showCloud(cloud_hull);
  while (!viewer.wasStopped ()){}
  return (0);
}
