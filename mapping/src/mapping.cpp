// Code to generate a map and detect obstacles from velodyne measurements.
// 16-05-2018
// José Antonio Cobano

//#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/point_cloud_conversion.h>

#include <stdio.h>
#include <time.h>
//#include <sys/time.h>
//#include <ctime>
#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <boost/concept_check.hpp>

// Representation (RVIZ)
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Pose2D.h>
#include "pcl_ros/point_cloud.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "mapping/vectorInt.h"
#include "mapping/vectorVector.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include "mapping/timeNode.h"

typedef std::vector<float> VI;
typedef std::vector<VI> VVI;
typedef std::vector<VVI> VVVI;

void exploration(const sensor_msgs::PointCloud2 &nube3d_uav);
void receiveSensor(const sensor_msgs::PointCloud2 &cloud);
int is_in_map(int i, int j);
int readV(const VVVI &V, int i, int j);
void save_matrix(char *filename, const VVVI &M);
void save_matrix3d(char *fileName, const VVVI &m, bool vel);
void phase_cb(const std_msgs::Int8 phaseMode);

// Publisher
ros::Publisher pub_time, pub_filtered_map2, pub_matrix;

//-------Variables globales------------------
bool patron_vacio = true;

int tamano;

// Variables related to the Map
//int range_sea=100;
int range_dock = 10;
int range_sea = 100;
float cell_div /* = 2 */; //número de celdas por 1 metro

int rang = range_dock * cell_div;
int rows = 2 * rang + 1;
int columns = 2 * rang + 1;
double cloud_thres_distance = 0.5; //Meters

// Atraque: 1 | Puerto: 2 | Mar: 3
int phase = 1;

mapping::vectorInt columns_mat;
mapping::vectorVector rows_mat;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ICP3D");
	ros::NodeHandle n;

	ros::Subscriber sub_phase = n.subscribe("/phase", 1, phase_cb);
	ros::Subscriber sub = n.subscribe("/velodyne_points", 1, receiveSensor);

	pub_time = n.advertise<mapping::timeNode>("/time_mapp", 1);
	pub_filtered_map2 = n.advertise<sensor_msgs::PointCloud2>("/map_filtered2", 1);
	pub_matrix = n.advertise<mapping::vectorVector>("/v_map", 1);

	ros::spin();

	return 0;
}

void phase_cb(const std_msgs::Int8 phaseMode)
{
	phase = phaseMode.data;
	switch (phase)
	{
	// Atraque
	case 1:
		range_dock = 10;
		cell_div = 2;
		break;
	// Puerto
	case 2:
		range_dock = 10;
		cell_div = 2;
		break;
	// Mar abierto
	case 3:
		range_dock = 100;
		cell_div = 1; // Revisarlo, estaba puesto a 2 (lento)
		break;
	}
	rang = range_dock * cell_div;
	rows = 2 * rang + 1;
	columns = 2 * rang + 1;
}

void receiveSensor(const sensor_msgs::PointCloud2 &cloud)
{
	ros::Time begin = ros::Time::now();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr data_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
	pcl::fromROSMsg(cloud, *cloud_XYZ);

	tamano = (int)cloud_XYZ->points.size();

	if (patron_vacio == true)
	{
		switch (phase)
		{
		// Atraque
		case 1:
			for (int i = 0; i < tamano - 1; i++)
			{
				float x = cloud_XYZ->points[i].x;
				float y = cloud_XYZ->points[i].y;
				float z = cloud_XYZ->points[i].z;
				// Lateral
				if (x > -1 && x < 1 && abs(y) < range_dock && abs(y) > 1)
				{
					data_cloud->points.push_back(cloud_XYZ->points[i]);
				}
				// Front
				if (x > 1 && x < range_dock && abs(y) < range_dock)
				{
					data_cloud->points.push_back(cloud_XYZ->points[i]);
				}
				// Back
				if (x > -range_dock && x < -1 && abs(y) < range_dock)
				{
					data_cloud->points.push_back(cloud_XYZ->points[i]);
				}
			}
			break;
		// Puerto
		case 2:
			for (int i = 0; i < tamano - 1; i++)
			{
				float x = cloud_XYZ->points[i].x;
				float y = cloud_XYZ->points[i].y;
				float z = cloud_XYZ->points[i].z;
				// Lateral
				if (x > -3.6 && x < 2.4 && abs(y) < range_dock && abs(y) > 1.2)
				{
					data_cloud->points.push_back(cloud_XYZ->points[i]);
				}
				// Front
				if (x > 2.4 && x < range_dock && abs(y) < range_dock)
				{
					data_cloud->points.push_back(cloud_XYZ->points[i]);
				}
				// Back
				if (x > -range_dock && x < -3.6 && abs(y) < range_dock)
				{
					data_cloud->points.push_back(cloud_XYZ->points[i]);
				}
			}
			break;
		// Mar
		case 3:
			for (int i = 0; i < tamano - 1; i++)
			{
				float x = cloud_XYZ->points[i].x;
				float y = cloud_XYZ->points[i].y;
				float z = cloud_XYZ->points[i].z;
				//if (abs(y) < range_sea && abs(x) < range_sea )
				//if (abs(y) < range_open && abs(x) < range_open )
				//if ((y < 100 || y > -100) && (x < 100 || x > -100))
				if ((y > 50) && (x < 100 || x > -100))
				{
					data_cloud->points.push_back(cloud_XYZ->points[i]);
				}
			}
			break;
		}
		//Convert the pointcloud to be used in ROS, usin variable sensor_msgs::PointCloud2 output;
		sensor_msgs::PointCloud2 cloud_filtered;
		pcl::toROSMsg(*data_cloud, cloud_filtered);
		cloud_filtered.header.frame_id = cloud.header.frame_id;
		// BUILD THE MAP (PointCloud2D)
		exploration(cloud_filtered);
	}
	std::cout << "[ MAPP] Time: " << ros::Time::now() - begin << std::endl;

	mapping::timeNode time_mapp;
	time_mapp.time_node = ros::Time::now() - begin;
	pub_time.publish(time_mapp);
}

//BUILD THE MAP
void exploration(const sensor_msgs::PointCloud2 &nube3d_uav)
{
	//V[i][j][k] is:
	//0 if empty or unexplored
	//1 if there is an obstacle
	//2 virtual obstacle
	//4 if it is the robot's cell
	//5 goal
	//>=10 if the cell surrounds an obstacle. 11 and 12 are safer than 10.

	sensor_msgs::PointCloud2 filtered_cloud_3D;

	VVVI V(rows, VVI(columns, VI(2)));
	VVVI V_map(rows, VVI(columns, VI(2)));
	VVVI cont_V(rows, VVI(columns, VI(2)));
	// Initialize the matrix
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			for (int k = 0; k < 2; k++)
			{
				if (k == 0)
				{
					V[i][j][k] = 0;
					cont_V[i][j][k] = 0;
					V_map[i][j][k] = 0;
				}
			}
		}
	}
	// Indicate the cell where the vessel is.
	V[rang][rang][0] = 4; //Position of the sensor (it is always the same because the map is local)

	int tam = nube3d_uav.width * nube3d_uav.height;

	pcl::PointCloud<pcl::PointXYZ>::Ptr fil_cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);  //Data to use in the map.
	pcl::PointCloud<pcl::PointXYZ>::Ptr fil_data_cloud(new pcl::PointCloud<pcl::PointXYZ>); //Filtered point cloud to publish
	// Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
	pcl::fromROSMsg(nube3d_uav, *fil_cloud_XYZ);
	// Build the matrix
	int max = 0;
	for (int i = 0; i < tam; i++) // nlements --> tamano
	{
		float cloud_dist = hypot(fil_cloud_XYZ->points[i].x, fil_cloud_XYZ->points[i].y);
		if (cloud_dist >= cloud_thres_distance)
		{
			int iM = round(rang - cell_div * fil_cloud_XYZ->points[i].x);
			int jM = round(rang - cell_div * fil_cloud_XYZ->points[i].y);
			if (cloud_dist < rang && is_in_map(iM, jM) == 1 && readV(V, iM, jM) != 4)
			{
				if (V[iM][jM][0] != 1)
				{
					V[iM][jM][0] = 1; //JAC
					cont_V[iM][jM][0] = cont_V[iM][jM][0] + 1;
				}
				if (V[iM][jM][0] == 1 && cont_V[iM][jM][0] >= 1)
				{
					cont_V[iM][jM][0] = cont_V[iM][jM][0] + 1;
				}
				if (cont_V[iM][jM][0] > max)
				{
					max = cont_V[iM][jM][0];
				}
			}
		}
	}
	// Filtered map to send to the collision avoidance module --> V_map
	V_map = V;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			for (int k = 0; k < 2; k++)
			{
				if (cont_V[i][j][0] < 1 && V[i][j][0] == 1) //5
				{
					V_map[i][j][0] = 0;
				}
			}
		}
	}
	// Publish V
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			columns_mat.columns.push_back(V[i][j][0]);
		}
		rows_mat.rows.push_back(columns_mat);
		columns_mat.columns.clear();
	}
	pub_matrix.publish(rows_mat);
	rows_mat.rows.clear();
	// Esto es para confirmar con Rviz que se van filtrando las olas. Se publica un topic con los datos de la nube filtrados.
	// map1_
	for (int i = 0; i < tam; i++) // nlements --> tamano
	{
		float cloud_dist = hypot(fil_cloud_XYZ->points[i].x, fil_cloud_XYZ->points[i].y);
		if (cloud_dist >= cloud_thres_distance)
		{
			int iM = round(rang - cell_div * fil_cloud_XYZ->points[i].x);
			int jM = round(rang - cell_div * fil_cloud_XYZ->points[i].y);
			if (cloud_dist < rang && is_in_map(iM, jM) == 1 && readV(V, iM, jM) != 4)
			{
				if (V[iM][jM][0] == 1 && V_map[iM][jM][0] == 0)
				{
				}
				else
				{
					fil_data_cloud->points.push_back(fil_cloud_XYZ->points[i]);
				}
			}
		}
	}
	//Convert the pointcloud to be used in ROS, usin variable sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*fil_data_cloud, filtered_cloud_3D);
	filtered_cloud_3D.header.frame_id = "velodyne";
	filtered_cloud_3D.header.stamp = ros::Time();
	pub_filtered_map2.publish(filtered_cloud_3D);

	// GENERATE FILES TO CHECK THE OUTPUTS AND THE GENERATION OF THE MATRIX.
	save_matrix3d("/home/hector/Matlab_ws/matrix.txt", V, false);
	save_matrix3d("/home/hector/Matlab_ws/contador.txt", cont_V, false);
	save_matrix3d("/home/hector/Matlab_ws/mapa.txt", V_map, false);
}

int is_in_map(int i, int j)
{
	if (i < 0 || i > rows - 1)
		return 0;
	if (j < 0 || j > columns - 1)
		return 0;
	return 1;
}

int readV(const VVVI &V, int i, int j)
{
	if (i < rows && j < columns && i >= 0 && j >= 0)
		return V[i][j][0];
	else
		return -1;
}

// FILE TO SAVE A MATRIX
void save_matrix3d(char *fileName, const VVVI &m, bool vel)
{
	std::ofstream file;
	file.open(fileName);

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			if (vel == false)
				file << m[i][j][0] << " ";
			if (vel == true)
				file << m[i][j][1] << " ";
		}
		file << std::endl;
	}

	file.close();
}