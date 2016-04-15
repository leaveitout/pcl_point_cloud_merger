#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/boost.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <CameraExtrinsicsIO.hpp>
#include <pcl/surface/convex_hull.h>
#include "Logger.hpp"

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointXYZRGBNormal PointNormalType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;


static const auto NUM_PCD_FILES = 2;
static const auto NUM_YML_FILES = 1;

Eigen::Matrix4f final_transform;

std::string output_params_file;


/// \brief Prints the help information
///
void printHelp(int, char **argv) {
  pcl::console::print_error("Syntax is: %s source.pcd target.pcd "
                                "output_params.yml\n", argv[0]);
}


// Get the indices of the points in the cloud that belong to the dominant plane
pcl::PointIndicesPtr extractPlaneIndices(Cloud::Ptr cloud,
                                         double plane_thold = 0.02) {
  // Object for storing the plane model coefficients.
  auto coefficients = boost::make_shared<pcl::ModelCoefficients>();
  // Create the segmentation object.
  auto segmentation = pcl::SACSegmentation<PointType>{};
  segmentation.setInputCloud(cloud);
  // Configure the object to look for a plane.
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  // Use RANSAC method.
  segmentation.setMethodType(pcl::SAC_RANSAC);
  // Set the maximum allowed distance to the model.
  segmentation.setDistanceThreshold(plane_thold);
  // Enable model coefficient refinement (optional).
  segmentation.setOptimizeCoefficients(true);

  auto inlierIndices = boost::make_shared<pcl::PointIndices>();
  segmentation.segment(*inlierIndices, *coefficients);

  return inlierIndices;
}


boost::detail::sp_if_not_array<pcl::PointCloud<pcl::PointXYZRGBA>>::type extractIndices(Cloud::Ptr cloud,
                                                                                        pcl::PointIndicesPtr indices,
                                                                                        bool keep_organised = false) {
  // Object for extracting points from a list of indices.
  auto extract = std::make_unique<pcl::ExtractIndices<pcl::PointXYZRGBA>>();
  extract->setInputCloud(cloud);
  extract->setIndices(indices);
  extract->setKeepOrganized(keep_organised);
  extract->setNegative(false);      // Extract indexed points

  auto extractedCloud = boost::make_shared<Cloud>();
  extract->filter(*extractedCloud);

  return extractedCloud;
}


// Euclidean Clustering
pcl::PointIndicesPtr euclideanClustering(Cloud::Ptr cloud,
                                         double cluster_tolerance = 0.005,
                                         int min_size = 100,
                                         int max_size = 307200) {
  // kd-tree object for searches.
  auto kd_tree = boost::make_shared<pcl::search::KdTree<PointType>>();
  kd_tree->setInputCloud(cloud);

  // Euclidean clustering object.
  pcl::EuclideanClusterExtraction<PointType> clustering;
  // Set cluster tolerance to 1cm (small values may cause objects to be divided
  // in several clusters, whereas big values may join objects in a same cluster).
  clustering.setClusterTolerance(cluster_tolerance);
  // Set the minimum and maximum number of points that a cluster can have.
  clustering.setMinClusterSize(min_size);
  clustering.setMaxClusterSize(max_size);
  clustering.setSearchMethod(kd_tree);
  clustering.setInputCloud(cloud);
  std::vector<pcl::PointIndices> clusters;
  clustering.extract(clusters);

  // Find largest cluster and return indices
  size_t largest_cluster_index = 0;
  size_t current_cluster_index = 0;
  size_t max_points = 0;
  for (const auto &cluster: clusters) {
    if (cluster.indices.size() > max_points) {
      max_points = cluster.indices.size();
      largest_cluster_index = current_cluster_index;
    }
    current_cluster_index++;
  }

  pcl::PointIndicesPtr indicesPtr = boost::make_shared<pcl::PointIndices>(clusters.at(largest_cluster_index));
  return indicesPtr;
}


CloudPtr extractLargestCluster(CloudPtr input,
                               double cluster_tolerance = 0.005,
                               int min_size = 100,
                               int max_size = 307200) {
  auto largestClusterIndices = euclideanClustering(input, cluster_tolerance, min_size, max_size);
  return extractIndices(input, largestClusterIndices, false);
}


// Region Growing
pcl::PointIndicesPtr regionGrowing(Cloud::Ptr cloud,
                                   double smoothness_degrees = 7.0,
                                   double curvature_thold = 1.0,
                                   double normal_radius_search = 0.01,
                                   size_t num_neighbours = 30,
                                   size_t min_size = 100,
                                   size_t max_size = 307200) {
  // kd-tree object for searches.
  pcl::search::KdTree<PointType>::Ptr kdTree(new pcl::search::KdTree<PointType>);
  kdTree->setInputCloud(cloud);

  // Estimate the normals.
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<PointType, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud);
  normalEstimation.setRadiusSearch(normal_radius_search);
  normalEstimation.setSearchMethod(kdTree);
  normalEstimation.compute(*normals);

  // Region growing clustering object.
  pcl::RegionGrowing<PointType, pcl::Normal> clustering;
  clustering.setMinClusterSize((int) min_size);
  clustering.setMaxClusterSize((int) max_size);
  clustering.setSearchMethod(kdTree);
  clustering.setNumberOfNeighbours((int) num_neighbours);
  clustering.setInputCloud(cloud);
  clustering.setInputNormals(normals);
  // Set the angle in radians that will be the smoothness threshold
  // (the maximum allowable deviation of the normals).
  clustering.setSmoothnessThreshold((float) (smoothness_degrees / 180.0 * M_PI)); // degrees.
  // Set the curvature threshold. The disparity between curvatures will be
  // tested after the normal deviation check has passed.
  clustering.setCurvatureThreshold((float) curvature_thold);

  std::vector<pcl::PointIndices> clusters;
  clustering.extract(clusters);

  // Find largest cluster and return indices
  size_t largest_cluster_index = 0;
  size_t current_cluster_index = 0;
  size_t max_points = 0;
  for (const auto &cluster: clusters) {
    if (cluster.indices.size() > max_points) {
      max_points = cluster.indices.size();
      largest_cluster_index = current_cluster_index;
    }
    current_cluster_index++;
  }

  pcl::PointIndicesPtr indicesPtr = boost::make_shared<pcl::PointIndices>(clusters.at(largest_cluster_index));
  return indicesPtr;
}


Cloud::Ptr getTable(Cloud::Ptr cloud, bool keep_organized = false) {
  pcl::PointIndicesPtr pointIndices = extractPlaneIndices(cloud);

  if (pointIndices->indices.size() == 0) {
    pcl::console::print_highlight("No plane to be found in input pcd.\n");
    return nullptr;
  }

  Cloud::Ptr extractedPlane = extractIndices(cloud, pointIndices, keep_organized);

  pcl::PointIndicesPtr largestClusterIndices = euclideanClustering(extractedPlane);

  Cloud::Ptr clusteredPlane = extractIndices(extractedPlane, largestClusterIndices, keep_organized);

  pcl::PointIndicesPtr regionGrowIndices = regionGrowing(clusteredPlane);

  return extractIndices(clusteredPlane, regionGrowIndices, keep_organized);
}


CloudPtr getPointsOnTable(Cloud::Ptr cloud,
                          double min_height = -0.05,
                          double max_height = 0.7,
                          bool keep_organized = false) {
  auto table = getTable(cloud);

  auto hull = pcl::ConvexHull<PointType>{};
  hull.setInputCloud(table);

  auto convexHullPoints = boost::make_shared<Cloud>();
  hull.reconstruct(*convexHullPoints);

  if (hull.getDimension() == 2) {
    auto prism = pcl::ExtractPolygonalPrismData<PointType>{};
    prism.setInputCloud(cloud);
    prism.setInputPlanarHull(convexHullPoints);
    prism.setHeightLimits(min_height, max_height);
    auto indicesPtr = boost::make_shared<pcl::PointIndices>();
    prism.segment(*indicesPtr);
    return extractIndices(cloud, indicesPtr);
  }
  else {
    pcl::console::print_error("The input cloud does not represent a planar surface for hull.\n");
    return nullptr;
  }
}


pcl::PointCloud<PointNormalType>::Ptr addNormals(pcl::PointCloud<PointType>::Ptr cloud) {
  auto normal_cloud = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  auto cloud_xyz = boost::make_shared< pcl::PointCloud<pcl::PointXYZ>>();
  pcl::copyPointCloud(*cloud, *cloud_xyz);

  auto search_tree = boost::make_shared<pcl::search::KdTree<PointType>>();
  search_tree->setInputCloud( cloud );

  auto normal_estimator = std::make_unique<pcl::NormalEstimation<PointType, pcl::Normal>>();
  normal_estimator->setInputCloud ( cloud );
  normal_estimator->setSearchMethod ( search_tree );
  normal_estimator->setKSearch( 30 );
  normal_estimator->compute( *normal_cloud );

  auto result_cloud = boost::make_shared<pcl::PointCloud<PointNormalType>>();
  pcl::concatenateFields( *cloud_xyz, *normal_cloud, *result_cloud );
  return result_cloud;
}


void pairAlign (const CloudPtr cloud_src,
                const CloudPtr cloud_tgt,
                CloudPtr output,
                Eigen::Matrix4f &final_transform,
                bool downsample = false) {
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  auto src = boost::make_shared<Cloud>();
  auto tgt = boost::make_shared<Cloud>();
  auto grid = pcl::VoxelGrid<PointType>{};
  if (downsample) {
    grid.setLeafSize (0.01, 0.01, 0.01);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  std::cout << src->size() << std::endl;
  std::cout << tgt->size() << std::endl;


  // Compute surface normals and curvature
  auto points_with_normals_src = addNormals(src);
  auto points_with_normals_tgt = addNormals(tgt);

  // Align
  pcl::IterativeClosestPointWithNormals<PointNormalType, PointNormalType> reg;
  reg.setTransformationEpsilon(1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(0.10);

  reg.setInputSource(points_with_normals_src);
  reg.setInputTarget(points_with_normals_tgt);

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f prev, Ti;
  Ti = Eigen::Matrix4f::Identity ();
//  auto prev = Eigen::Matrix4f::Identity ();
//  auto targetToSource = Eigen::Matrix4f::Identity ();
  auto reg_result = points_with_normals_src;
  reg.setMaximumIterations(2);
  for (int i = 0; i < 40; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource(points_with_normals_src);
    reg.align(*reg_result);

    //accumulate transformation between each Iteration
    Ti = static_cast<Eigen::Matrix4f>(reg.getFinalTransformation()) * Ti;

    //if the difference between this transformation and the previous one
    //is smaller than the threshold, refine the process by reducing
    //the maximal correspondence distance
    auto incremental_diff = fabs ((reg.getLastIncrementalTransformation () - prev).sum ());
    if ( incremental_diff < reg.getTransformationEpsilon ()) {
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () * 0.95);
    }

    prev = reg.getLastIncrementalTransformation ();
  }

  //add the source to the transformed target
  *output += *cloud_src;

  final_transform = Ti;
}


void saveTransformation() {
  Logger::log(Logger::INFO, "Saving extrinsics....");

  CameraExtrinsicsIO::saveExtrinsics(output_params_file, final_transform);
}


void keyboardEventHandler(const pcl::visualization::KeyboardEvent &event, void* ) {
  if (event.getKeySym() == "s" && event.keyDown()) {
    saveTransformation();
  }
}


int main(int argc, char **argv) {
  pcl::console::print_info("Register two point clouds using ICP, usage:\n%s -h\n", argv[0]);

  if (argc != NUM_PCD_FILES + NUM_YML_FILES + 1) {
    printHelp(argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  auto pcd_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  if (pcd_file_indices.size() != NUM_PCD_FILES) {
    pcl::console::print_error("Incorrect number of .pcd file parameters.\n");
    printHelp(argc, argv);
    return (-1);
  }

  auto yml_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".yml");
  if (yml_file_indices.size() != NUM_YML_FILES) {
    pcl::console::print_error("Incorrect number of .yml file parameters.\n");
    printHelp(argc, argv);
    return (-1);
  }

  auto source_pcd = std::string{argv[pcd_file_indices[0]]};
  auto target_pcd = std::string{argv[pcd_file_indices[1]]};
  output_params_file = std::string{argv[yml_file_indices[0]]};

  auto target = boost::make_shared<pcl::PointCloud<PointType>>();
  auto source = boost::make_shared<pcl::PointCloud<PointType>>();

  if (!pcl::io::loadPCDFile(target_pcd, *target) < 0) {
    pcl::console::print_error("Could not load pcd file: %s", target_pcd.c_str());
    return -1;
  }
  if (!pcl::io::loadPCDFile(source_pcd, *source) < 0) {
    pcl::console::print_error("Could not load pcd file: %s", source_pcd.c_str());
    return -1;
  }

  // Clean clouds of NaNs
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*target, *target, mapping);
  pcl::removeNaNFromPointCloud(*source, *source, mapping);

  auto target_table = getPointsOnTable(target, -0.02);
  auto source_table = getPointsOnTable(source, -0.02);

  final_transform = Eigen::Matrix4f::Identity();
  auto final_cloud = boost::make_shared<Cloud>();
  pairAlign(source_table, target_table, final_cloud, final_transform);
  pcl::transformPointCloud(*source_table, *source_table, final_transform);
  pcl::transformPointCloud(*source, *source, final_transform);

  std::cout << "Final transformation: RT = " << std::endl << final_transform << std::endl;

  // Visualize both the original and the result.
  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

  // Change colors
  pcl::visualization::PointCloudColorHandlerCustom<PointType>
      colorHandlerTarget(target_table, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<PointType>
      colorHandler(source_table, 255, 200, 200);
  viewer.addPointCloud(target, "original");
  viewer.addPointCloud(source, "source");

  viewer.initCameraParameters();
  pcl::visualization::Camera camera;
  viewer.getCameraParameters(camera);
  camera.view[1] *= -1;
  viewer.setCameraParameters(camera);
  viewer.registerKeyboardCallback(keyboardEventHandler, (void*)&viewer);

  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }
}
