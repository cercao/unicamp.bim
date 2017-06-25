#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace pcl;
using namespace std;

void showLimits(PointCloud<PointXYZ>::Ptr cloud){
  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);
  for (size_t i = 0; i < cloud->points.size (); ++i){
     Array4fMap pt = cloud->points[i].getArray4fMap();
     min_p = min_p.min (pt);
     max_p = max_p.max (pt);
  }

  cout << "Máximos: " << max_p << endl;
  cout << "Mínimos: " << min_p << endl;
}


PointCloud<PointXYZ>::Ptr cropCloud(PointCloud<PointXYZ>::Ptr cloud, const Eigen::Vector3f& min,
                                                                           const Eigen::Vector3f& max){
    ConditionAnd<PointXYZ>::Ptr range_cond(new ConditionAnd<PointXYZ> ());
    range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr
                             (new FieldComparison<PointXYZ>("x", ComparisonOps::GT, min[0])));
    range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr
                             (new FieldComparison<PointXYZ>("x", ComparisonOps::LT, max[0])));
    range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr
                             (new FieldComparison<PointXYZ>("y", ComparisonOps::GT, min[1])));
    range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr
                             (new FieldComparison<PointXYZ>("y", ComparisonOps::LT, max[1])));
    range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr
                             (new FieldComparison<PointXYZ>("z", ComparisonOps::GT, min[2])));
    range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr
                             (new FieldComparison<PointXYZ>("z", ComparisonOps::LT, max[2])));

    PointCloud<PointXYZ>::Ptr croppedCloud (new PointCloud<PointXYZ>);
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.filter (*croppedCloud);
    return croppedCloud;
}
PointCloud<PointXYZ>::Ptr noiseRemove(PointCloud<PointXYZ>::Ptr cloud, float mean, float std){
    PointCloud<PointXYZ>::Ptr cloud_filtered (new PointCloud<PointXYZ>);

    StatisticalOutlierRemoval<PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean);
    sor.setStddevMulThresh(std);
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}
void extractRegions(search::Search<PointXYZ>::Ptr tree,PointCloud<PointXYZ>::Ptr cloud,std::vector <PointIndices> clusters){
   PCDWriter writer;

   PointCloud<PointXYZ>::Ptr cloud_plane (new PointCloud<PointXYZ>);
   pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
   ec.setClusterTolerance (0.01); // 1cm
   ec.setMinClusterSize (100);
   ec.setMaxClusterSize (25000);
   ec.setSearchMethod (tree);
   ec.setInputCloud (cloud);
   ec.extract (clusters);

   int j = 0;
   for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
   {
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
       cloud_cluster->points.push_back (cloud->points[*pit]); //*
     cloud_cluster->width = cloud_cluster->points.size ();
     cloud_cluster->height = 1;
     cloud_cluster->is_dense = true;

     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
     std::stringstream ss;
     ss << "cloud_cluster_" << j << ".pcd";
     writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
     j++;
   }
}

int main (int argc, char** argv){

  PointCloud<PointXYZRGB>::Ptr cloud_rgb (new PointCloud<PointXYZRGB>);
  PCDReader reader;
  reader.read("/home/avell/Desktop/point_cloud/bmrt_small.pcd",*cloud_rgb);

  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);

  copyPointCloud(*cloud_rgb, *cloud);

  cloud_rgb->~PointCloud();



  cout << "Número de pontos " << cloud->width*cloud->height << '\n';

//  showLimits(cloud);

  Eigen::Vector3f min(-25.f,-25.f,-20.f); Eigen::Vector3f max(-1.f,-1.f,20.f);

  PointCloud<PointXYZ>::Ptr croppedCloud = cropCloud(cloud,min,max);
  cloud->~PointCloud();

  cout << "Número de pontos " << croppedCloud->width*croppedCloud->height << '\n';

  PointCloud<PointXYZ>::Ptr cloud_filtered = noiseRemove(croppedCloud,50,1);

  croppedCloud->~PointCloud();


  search::Search<PointXYZ>::Ptr tree = boost::shared_ptr<search::Search<PointXYZ> > (new search::KdTree<PointXYZ>);
  PointCloud <Normal>::Ptr normals (new PointCloud <Normal>);
  NormalEstimationOMP<PointXYZ, Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_filtered);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  IndicesPtr indices (new std::vector <int>);
  PassThrough<PointXYZ> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);


  RegionGrowing<PointXYZ, Normal> reg;
  reg.setMinClusterSize (1000);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (50);
  reg.setInputCloud (cloud_filtered);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (DEG2RAD(3.f));
  reg.setCurvatureThreshold (1);

  std::vector <PointIndices> clusters;
  reg.extract (clusters);


  PointCloud <PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//
  io::savePCDFile("bmrtsmall_filtred_segmented.pcd", *colored_cloud);
//
  // Visualização
  visualization::CloudViewer viewer ("Visualizer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ()){}
  return (0);
}
