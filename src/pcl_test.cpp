#include <iostream>
#include <string>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
//using namespace pcl;
using namespace std;

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

void visualize_pointcloud(const PointCloud::Ptr& points)
{
  // Add clouds to vizualizer
  pcl::visualization::PCLVisualizer viewer;
  viewer.addPointCloud(points, "points");
  viewer.spin();
}

void pointcloud_demo()
{
  std::string filename;
  std::cout << "Input filename: ";
  std::cin >> filename;


  // Create new point clouds to hold data
  PointCloud::Ptr points (new pcl::PointCloud<Point>);

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
void kitti_demo()
{
  cout<<"kitti_demo()"<<endl;
  std::string filename;
  std::cout << "Input filename: ";
  std::cin >> filename;

  pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

  kitti_open(filename, points);

  cout<<"calculate Normals"<<endl;

  double tElapsedTime;
#ifdef WIN32
  // Windows Time 측정
  LARGE_INTEGER        tFreq, tStart, tEnd;

  QueryPerformanceFrequency(&tFreq);        // 주파수 측정
  QueryPerformanceCounter(&tStart);            // 카운트 시작
#else
  struct timeval start_time, end_time;
  gettimeofday(&start_time, NULL);
#endif

  // 시간 측정을 위한 소스 코드
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(points);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  //ne.setRadiusSearch(0.2);
  ne.setKSearch(4);
	ne.setViewPoint(1,0,1);
	ne.compute(*cloud_normals);

#ifdef WIN32
	QueryPerformanceCounter(&tEnd);            // 카운트 종료
	// 측정 시간 저장
	tElapsedTime = ((tEnd.QuadPart - tStart.QuadPart)/(double)tFreq.QuadPart)*1000;
#else
  gettimeofday(&end_time, NULL);
  tElapsedTime = (end_time.tv_sec - start_time.tv_sec) + (double)(end_time.tv_usec - start_time.tv_usec)/1000000.0;
#endif
	cout << "ElaspedTime:"<< tElapsedTime<<"sec" << endl;

#ifdef WIN32
  QueryPerformanceFrequency(&tFreq);        // 주파수 측정
  QueryPerformanceCounter(&tStart);            // 카운트 시작
#else
  gettimeofday(&start_time, NULL);
#endif
 // 시간 측정을 위한 소스 코드
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne1;
  ne1.setInputCloud(points);
  ne1.setSearchMethod(tree);
  ne1.setRadiusSearch(0.02);
  //ne1.setKSearch(10);
  ne1.setViewPoint(0,0,1);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
  ne1.compute(*cloud_normals1);

#ifdef WIN32
	QueryPerformanceCounter(&tEnd);            // 카운트 종료
	// 측정 시간 저장
	tElapsedTime = ((tEnd.QuadPart - tStart.QuadPart)/(double)tFreq.QuadPart)*1000;
#else
  gettimeofday(&end_time, NULL);
  tElapsedTime = (end_time.tv_sec - start_time.tv_sec) + (double)(end_time.tv_usec - start_time.tv_usec)/1000000.0;
#endif
  cout << "ElaspedTime1:"<< tElapsedTime<<"sec" << endl;

#ifdef WIN32
  QueryPerformanceFrequency(&tFreq);        // 주파수 측정
  QueryPerformanceCounter(&tStart);            // 카운트 시작
#else
  gettimeofday(&start_time, NULL);
#endif
  // estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);

  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne2;
  ne2.setInputCloud(points);
  ne2.setSearchMethod(tree);
  ne2.setRadiusSearch(0.2);
  //ne1.setKSearch(10);
  ne2.setViewPoint(0,0,1);
  ne2.compute(*cloud_normals2);
#ifdef WIN32
	QueryPerformanceCounter(&tEnd);            // 카운트 종료
	// 측정 시간 저장
	tElapsedTime = ((tEnd.QuadPart - tStart.QuadPart)/(double)tFreq.QuadPart)*1000;
#else
  gettimeofday(&end_time, NULL);
  tElapsedTime = (end_time.tv_sec - start_time.tv_sec) + (double)(end_time.tv_usec - start_time.tv_usec)/1000000.0;
#endif
  cout << "ElaspedTime2:"<< tElapsedTime<<"sec" << endl;

  cout<<"visualize()"<<endl;
  pcl::visualization::PCLVisualizer viewer;
  //viewer.initCameraParameters();
  int v1(0);
  viewer.createViewPort(0.0, 0.0,0.325,1.0,v1);
  viewer.setBackgroundColor(0,0,0,v1);
  viewer.addText("cloud normals with radius search 0.02",10,10,"v1 text",v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color1(points, 0, 0, 255);
  viewer.addPointCloud<pcl::PointXYZI>(points,single_color1, "points",v1);
  //viewer.addPointCloud<pcl::PointXYZI>(points, "points",v1);
  viewer.addPointCloudNormals<pcl::PointXYZI, pcl::Normal>(points, cloud_normals1, 10, 0.5, "normals1", v1);


  int v2(0);
  viewer.createViewPort(0.325, 0.0,1.0,1.0,v2);
  viewer.setBackgroundColor(0.2,0.2,0.2,v2);
  viewer.addText("cloud normals with knn search 4",10,10,"v2 text",v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(points, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZI>(points,single_color, "points1",v2);
  viewer.addPointCloudNormals<pcl::PointXYZI, pcl::Normal>(points, cloud_normals, 10, 0.5, "normals", v2);
  int v3(0);
  viewer.createViewPort(0.675, 0.0,1.0,1.0,v3);
  viewer.setBackgroundColor(0.3,0.3,0.3,v3);
  viewer.addText("cloud normals with radius search 0.2",10,10,"v3 text",v3);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color2(points, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZI>(points,single_color2, "points2",v3);
  viewer.addPointCloudNormals<pcl::PointXYZI, pcl::Normal>(points, cloud_normals2, 10, 0.5, "normals2", v3);

  viewer.addCoordinateSystem(1.0);
  viewer.spin();
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
  return (0);
}

