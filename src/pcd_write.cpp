#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

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
	cout<<"kitti_demo()"<<endl;
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

	//// load point cloud
	//fstream input(filename.c_str(), ios::in | ios::binary);
	//if(!input.good()){
	//	cerr << "Could not read file: " << filename << endl;
	//	exit(EXIT_FAILURE);
	//}
	//input.seekg(0, ios::beg);
	pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

	//int i;
	//for (i=0; input.good() && !input.eof(); i++) {
	//	pcl::PointXYZI point;
	//	input.read((char *) &point.x, 3*sizeof(float));
	//	input.read((char *) &point.intensity, sizeof(float));
	//	points->push_back(point);
	//}
	//input.close();
	kitti_open(filename, points);

	pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud<pcl::PointXYZI>(points, "points");
    viewer.spin();

	

    // Save DoN features
    /*writer.write<pcl::PointXYZI> (outfile_name, *points, false);*/
}
void kitti_pcd()
{
	std::string infile_name;
	std::cout << "input filename: ";
	std::cin >> infile_name;
	std::string outfile_name;
	std::cout << "output filename: ";
	std::cin >> outfile_name;

	// load point cloud
	fstream input(infile_name.c_str(), ios::in | ios::binary);
	if(!input.good()){
		cerr << "Could not read file: " << infile_name << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
	pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

	int i;
	for (i=0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();

	cout << "Read KTTI point cloud with " << i << " points, writing to " << outfile_name << endl;

    pcl::PCDWriter writer;

    // Save DoN features
    writer.write<pcl::PointXYZI> (outfile_name, *points, false);

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

