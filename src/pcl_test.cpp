#include <iostream>
#include <string>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

//using namespace pcl;
using namespace std;

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;



class ComputeProcessingTime
{
  public:
    double tElapsedTime_;
  #ifdef WIN32
    // Windows Time 측정
    LARGE_INTEGER tFreq_, tStart_, tEnd_;
  #else
    struct timeval start_time_, end_time_;
  #endif
    ComputeProcessingTime()
    {
      
    }
    ~ComputeProcessingTime()
    {
    }
    void start_time()
    {
    #ifdef WIN32
      QueryPerformanceFrequency(&tFreq_);        // 주파수 측정
      QueryPerformanceCounter(&tStart_);            // 카운트 시작
    #else
      gettimeofday(&start_time_, NULL);
    #endif
    }
    float end_time()
    {
     #ifdef WIN32
      QueryPerformanceCounter(&tEnd_);            // 카운트 종료
      // 측정 시간 저장
      tElapsedTime_ = ((tEnd_.QuadPart - tStart_.QuadPart)/(double)tFreq_.QuadPart);
     #else
      gettimeofday(&end_time_, NULL);
      tElapsedTime_ = (end_time_.tv_sec - start_time_.tv_sec) + (double)(end_time_.tv_usec - start_time_.tv_usec)/1000000.0;
    #endif
      return tElapsedTime_;
    }

};

void range_image_creation(const pcl::PointCloud<pcl::PointXYZI>::Ptr &points)
{
  float angularResolution = (float) ( 4.95f * (M_PI/180.0f));
  float maxAngleWidth      = (float) ( 360.0f * (M_PI/180.0f));
  float maxAnglHeight      = (float) ( 180.f * (M_PI/180.0f));

  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordnate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel = 0.0f;
  float minRange = 0.0f;
  int borderSize = 0;

  cout << "range image creation" << endl;
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(*points, 4.95,0.4, maxAngleWidth, maxAnglHeight, 
      sensorPose, coordnate_frame, noiseLevel, minRange, borderSize);

  std::cout << rangeImage << std::endl;
 
}
void visualize_pointcloud(const Cloud::Ptr& points)
{
  // Add clouds to vizualizer
  pcl::visualization::PCLVisualizer viewer;
  viewer.addPointCloud(points, "points");
  viewer.spin();
}

void down_sample(float leaf_size,const pcl::PointCloud<pcl::PointXYZI>::Ptr &points, const pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_points)
{
  pcl::VoxelGrid<pcl::PointXYZI> vox;
  vox.setInputCloud(points);
  vox.setLeafSize(leaf_size, leaf_size, leaf_size);
  vox.filter(*filtered_points);
  cout << "Voxel Grid: " <<points->width * points->height << " --> " 
    << filtered_points->width * filtered_points->height << endl;
}

// neighbor_method 0: Radius search, 1:KNN search
void compute_surface_normals(bool neighbor_method, float neighbor_param, const pcl::PointCloud<pcl::PointXYZI>::Ptr &points, const pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  // Surface Normal Computation 
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(points);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  ne.setSearchMethod(tree);
  if(!neighbor_method)
    ne.setRadiusSearch(neighbor_param);
  else
    ne.setKSearch(neighbor_param);
	ne.setViewPoint(0,0,1);
	ne.compute(*normals);
}

void pointcloud_demo()
{
  std::string filename;
  std::cout << "Input filename: ";
  std::cin >> filename;


  // Create new point clouds to hold data
  Cloud::Ptr points (new Cloud);

  // Load point cloud
  pcl::io::loadPCDFile(filename, *points);

  // Visualize point cloud
  visualize_pointcloud(points);
}


void kitti_open(string filename, pcl::PointCloud<pcl::PointXYZI>::Ptr &points)
{
	cout<<"kitti_file_open()"<<endl;
	// load point cloud
	fstream input(filename.c_str(), ios::in | ios::binary);
	if(!input.good()){
		cerr << "Could not read file: " << filename << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
	
	int i;
	for (i=0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
	cout << "Read KTTI point cloud with " << i << " points" << endl;
}

void range_image_demo()
{
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	std::string filename;
	filename = "0.bin";


  pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);


  kitti_open(filename, points);


  points->width = 10000;
  points->points.resize(points->width * points->height);
  pcl::PointCloud<pcl::PointXYZI>& point_cloud = *points;
  // Generate the data
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      pointCloud.points.push_back(point);
    }
  }
  pointCloud.width = (uint32_t) pointCloud.points.size();
  pointCloud.height = 1;
  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  //pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;

  float noiseLevel=0.05;
  float minRange = 0.0f;
  int borderSize = 1;
  
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(point_cloud,pcl::deg2rad(0.49462f), pcl::deg2rad(0.41875f), maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  //rangeImage.createFromPointCloud(*filtered_points,pcl::deg2rad(4.9462f), pcl::deg2rad(0.41875f), maxAngleWidth, maxAngleHeight,
  //                                sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  std::cout << rangeImage << "\n";
  
  // --------------------------
  // -----Show range image-----
  // --------------------------
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  range_image_widget.showRangeImage (rangeImage);

  range_image_widget.spin ();
}
void kitti_demo()
{
  cout<<"kitti_demo()"<<endl;
  std::string filename;
  std::cout << "Input filename: ";
//  std::cin >> filename;
  filename = "0.bin";
  cout << filename << endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_points (new pcl::PointCloud<pcl::PointXYZI>);

  ///////////////////////////////
  // Claculating processing time
  ///////////////////////////////
  ComputeProcessingTime CPTime;
  ///////////////////////////////

//  kitti_open(filename, points);
  kitti_open(filename, filtered_points);

  //filtered_points->width = 114304;
  //filtered_points->points.resize(filtered_points->width * filtered_points->height);
  
  CPTime.start_time();
  down_sample(0.2f, filtered_points, points);
//  down_sample(0.5f, points, filtered_points);

  float CP_time = CPTime.end_time();
	cout << "ElaspedTime for Voxel Filter:"<< CP_time<<"sec, " <<  endl;

  cout<<"calculate Normals"<<endl;

  CPTime.start_time();
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  //0:Radisu Search, 1: KNN Search
  compute_surface_normals(1, 4, points, cloud_normals);

  CP_time = CPTime.end_time();
	cout << "ElaspedTime:"<< CP_time<<"sec " <<  endl;

  CPTime.start_time();
 // 시간 측정을 위한 소스 코드
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);

  compute_surface_normals(1, 8, points, cloud_normals1);
  CP_time = CPTime.end_time();
  cout << "ElaspedTime1:"<< CP_time<<"sec " <<  endl;

  CPTime.start_time();
  // estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);

  compute_surface_normals(0, 0.2, points, cloud_normals2);

  CP_time = CPTime.end_time();

  cout << "ElaspedTime2:"<< CP_time<<"sec " <<  endl;

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices) ;
  pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients); 
  plane_coefficients->values.resize(4);
  plane_coefficients->values[0] = plane_coefficients->values[1] = 0;
  plane_coefficients->values[2] = 1.0;
  plane_coefficients->values[3] = 0;

  seg.setDistanceThreshold(0.05);
  seg.setMaxIterations(1000);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setInputCloud(points);
  seg.segment(*plane_inliers,*plane_coefficients);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);  
  pcl::ProjectInliers<pcl::PointXYZI> proj;
  // Extract the inliers indices as a separate point cloud
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(points);

  proj.setIndices(plane_inliers);
  proj.setModelCoefficients(plane_coefficients);
  proj.filter(*cloud_plane);
  cout<<"plane points: " << cloud_plane->width * cloud_plane->height <<endl;
  cout<<"visualize()"<<endl;
  pcl::visualization::PCLVisualizer viewer1;

  viewer1.addPointCloud<pcl::PointXYZI>(cloud_plane,"points");
  pcl::visualization::PCLVisualizer viewer;
  viewer.initCameraParameters();
  int v1(0);
  viewer.createViewPort(0.0, 0.0,0.325,1.0,v1);
  viewer.setBackgroundColor(0,0,0,v1);
  viewer.addText("cloud normals with KNN search 4",10,10,"v1 text",v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color1(points, 0, 0, 255);
  //viewer.addPointCloud<pcl::PointXYZI>(points, "points",v1);
  viewer.addPointCloudNormals<pcl::PointXYZI, pcl::Normal>(points, cloud_normals, 1, 0.3, "normals", v1);


  int v2(0);
  viewer.createViewPort(0.3375, 0.0,1.0,1.0,v2);
  viewer.setBackgroundColor(0.2,0.2,0.2,v2);
  viewer.addText("cloud normals with KNN search 8",10,10,"v2 text",v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(points, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZI>(points,single_color, "points1",v2);
  viewer.addPointCloudNormals<pcl::PointXYZI, pcl::Normal>(points, cloud_normals1, 1, 0.3, "normals1", v2);
  int v3(0);
  viewer.createViewPort(0.675, 0.0,1.0,1.0,v3);
  viewer.setBackgroundColor(0.3,0.3,0.3,v3);
  viewer.addText("cloud normals with radius search 0.2",10,10,"v3 text",v3);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color2(points, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZI>(points,single_color2, "points2",v3);
  viewer.addPointCloudNormals<pcl::PointXYZI, pcl::Normal>(points, cloud_normals2, 1, 0.3, "normals2", v3);

  viewer.addCoordinateSystem(1.0);
  viewer.spin();

  pcl::PCDWriter writer;
  std::string output_filename = "output.pcd";
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr p_n_cloud_c(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::concatenateFields (*points, *cloud_normals, *p_n_cloud_c);
  // Save DoN features
  writer.write<pcl::PointXYZINormal> (output_filename, *p_n_cloud_c, false);
}
void kitti_pcd()
{
  std::string input_filename;
  std::cout << "input filename: ";
  std::cin >> input_filename;
  std::string output_filename;
  std::cout << "output filename: ";
  std::cin >> output_filename;

  pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
  // load point cloud
  kitti_open(input_filename, points);

  pcl::PCDWriter writer;

  // Save DoN features
  writer.write<pcl::PointXYZI> (output_filename, *points, false);

}

int main (int argc, char** argv)
{
  //PointCloud cloud;


  // // Fill in the cloud data
  // cloud.width    = 5;
  // cloud.height   = 1;
  // cloud.is_dense = false;
  // cloud.points.resize (cloud.width * cloud.height);

  // for (size_t i = 0; i < cloud.points.size (); ++i)
  // {
  //   cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
  //   cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
  //   cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  // }

  // pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  // std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  // for (size_t i = 0; i < cloud.points.size (); ++i)
  //   std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
  //pointcloud_demo();
  //kitti_pcd();
  kitti_demo();
  //range_image_demo();
  return (0);
}

