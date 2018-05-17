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

using namespace std; //Prueba

typedef vector<int> VI;
typedef vector<VI> VVI;
typedef vector<VVI> VVVI;

void exploration(const sensor_msgs::PointCloud2 &nube3d_uav);
int is_in_map(int i, int j);
int readV(const VVI &V, int i, int j);
void save_matrix(char *filename, const VVI &M);

// Publisher
ros::Publisher pub_filtered_map, pub_filtered_map2;

//-------Variables globales------------------
bool patron_vacio = true;
FILE *log_pos;
FILE *log_icp;
FILE *log_times;

float time_patron;
float time_icp;
float time_result;
int contador;
int tamano, tamano1;


// Variables related to the Map
int range_sea=100;
int range_dock=10;
float cell_div = 2; //número de celdas por 1 metro

const int rang = range_dock * cell_div;
const int rows = 2 * rang + 1;
const int columns = 2 * rang + 1;
double cloud_thres_distance = 0.5; //Meters

int phase = 1;
//phase=0 --> mar abierto
//phase=1 --> atraque

// Obstacles
int rows_obs = 20;
int columns_obs = 2;

VVI V(rows, VI(columns));
VVI V_map(rows, VI(columns));
VVI cont_V(rows, VI(columns));

//--------------------------------------------
//sensor_msgs::PointCloud cloud_2D;
sensor_msgs::PointCloud2 filtered_cloud_3D2;
const std_msgs::Header msg_header;

void receiveSensor(const sensor_msgs::PointCloud2 &cloud)
{
	//ROS_INFO("Recibido \n");
	//sensor_msgs::PointCloud2 data_cloud = cloud;  // Se puede prescindir de data_cloud cuando funcione bien con pcl.
	
	float runningTime;
	
	// Variables to detect obstacle
	VVI obstacles_left(rows_obs, VI(columns_obs));
	VVI obstacles_right(rows_obs, VI(columns_obs));
	VVI obstacles_front(rows_obs, VI(columns_obs));
	VVI obstacles_back(rows_obs, VI(columns_obs));
	float min_lat_left, min_lat_right, min_front, min_back;
	float pos_min_lat_left[3], pos_min_lat_right[3], pos_min_front[3], pos_min_back[3];
	int first_lat_left = 1;
	int first_lat_right = 1;
	int first_front = 1;
	int first_back = 1;

	float x, y, z;
	unsigned long a;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr data_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
	pcl::fromROSMsg(cloud, *cloud_XYZ);

	//tamano = cloud.width * cloud.height;
	
	tamano = (int)cloud_XYZ->points.size();  
	//ROS_INFO("tam= %d \n", tamano);
	
	//tamano1 = (int)cloud_cluster->points.size();
	//ROS_INFO("tam1= %d \n", tamano1);

	if (patron_vacio == true)
	{

		int i = 0;

		timespec t0_1;
		clock_gettime(CLOCK_MONOTONIC, &t0_1);

		//JAC: Mar abierto.
		if (phase == 0)
		{
			for (i = 0; i < tamano - 1; i++)
			{
				//fprintf(log_pos, "%lf %lf %lf\n", x, y, z);
				x=cloud_XYZ->points[i].x;
				y=cloud_XYZ->points[i].y;
				z=cloud_XYZ->points[i].z;
								
				if (abs(x) < range_sea && abs(y) < range_sea )
				{
					//data_cloud->points.push_back(cloud_cluster->points[i]);
					data_cloud->points.push_back(cloud_XYZ->points[i]);
										
					//fprintf(log_pos, "%lf %lf %lf\n", x, y, z);
				}
			}
		}
		//JAC: Atraque.

		else
		{
			for (i = 0; i < tamano - 1; i++)
			{
				x=cloud_XYZ->points[i].x;
				y=cloud_XYZ->points[i].y;
				z=cloud_XYZ->points[i].z;

				// Lateral
				if (x > -3.6 && x < 2.4 && abs(y) < range_dock && abs(y) > 1.2)
				{
					data_cloud->points.push_back(cloud_XYZ->points[i]);
					
					// Left side
					if (y > 0)
					{
						if (first_lat_left == 1)
						{
							min_lat_left = data_cloud->points[i].y;
							pos_min_lat_left[0] = data_cloud->points[i].x;
							pos_min_lat_left[1] = data_cloud->points[i].y;
							pos_min_lat_left[2] = data_cloud->points[i].z;
							first_lat_left = 0;
						}
						else
						{
							if (y < min_lat_left)
							{
								min_lat_left = data_cloud->points[i].y;
								pos_min_lat_left[0] = data_cloud->points[i].x;
								pos_min_lat_left[1] = data_cloud->points[i].y;
								pos_min_lat_left[2] = data_cloud->points[i].z;
							}
						}
					}
					// Right side
					if (y < 0)
					{
						if (first_lat_right == 1)
						{
							min_lat_right = data_cloud->points[i].y;
							pos_min_lat_right[0] = data_cloud->points[i].x;
							pos_min_lat_right[1] = data_cloud->points[i].y;
							pos_min_lat_right[2] = data_cloud->points[i].z;
							first_lat_right = 0;
						}
						else
						{
							if (y < min_lat_right)
							{
								min_lat_right = data_cloud->points[i].y;
								pos_min_lat_right[0] = data_cloud->points[i].x;
								pos_min_lat_right[1] = data_cloud->points[i].y;
								pos_min_lat_right[2] = data_cloud->points[i].z;
							}
						}
					}
				}

				// Front
				if (x > 2.4 && x < range_dock && abs(y) < range_dock)
				{
					data_cloud->points.push_back(cloud_XYZ->points[i]);
					
					// Front
					if (x > 0)
					{
						if (first_front == 1)
						{
							min_front = data_cloud->points[i].x;
							pos_min_front[0] = data_cloud->points[i].x;
							pos_min_front[1] = data_cloud->points[i].y;
							pos_min_front[2] = data_cloud->points[i].z;
							first_front = 0;
						}
						else
						{
							if (x < min_front)
							{
								min_front = data_cloud->points[i].x;
								pos_min_front[0] = data_cloud->points[i].x;
								pos_min_front[1] = data_cloud->points[i].y;
								pos_min_front[2] = data_cloud->points[i].z;
							}
						}
					}
				}

				// Back
				if (x > -range_dock && x < -3.6 && abs(y) < range_dock)
				{
					data_cloud->points.push_back(cloud_XYZ->points[i]);
					
					// Front
					if (x < 0)
					{
						if (first_back == 1)
						{
							min_back = data_cloud->points[i].x;
							pos_min_back[0] = data_cloud->points[i].x;
							pos_min_back[1] = data_cloud->points[i].y;
							pos_min_back[2] = data_cloud->points[i].z;
							first_back = 0;
						}
						else
						{
							if (x > min_back)
							{
								min_back = data_cloud->points[i].x;
								pos_min_back[0] = data_cloud->points[i].x;
								pos_min_back[1] = data_cloud->points[i].y;
								pos_min_back[2] = data_cloud->points[i].z;
							}
						} //x<0
					}
				}
			}
		}
	
		//Convert the pointcloud to be used in ROS, usin variable sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
		//sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
		//pcl::toROSMsg(*data_cloud, *output);
		//output->header.frame_id = cloud.header.frame_id;
		
		//Convert the pointcloud to be used in ROS, usin variable sensor_msgs::PointCloud2 output;
		sensor_msgs::PointCloud2 cloud_filtered;
		pcl::toROSMsg(*data_cloud, cloud_filtered);
		cloud_filtered.header.frame_id = cloud.header.frame_id;
		
		//tamano1 = (int)data_cloud->points.size();
		//ROS_INFO("data_cloud= %d \n", tamano1);
		//tamano1 = cloud_filtered.width * cloud_filtered.height;
		//ROS_INFO("cloud_filtered= %d \n", tamano1);
		
		// BUILD THE MAP (PointCloud2D)
		exploration(cloud_filtered);
		
		timespec t0_2;
		clock_gettime(CLOCK_MONOTONIC, &t0_2);
		time_patron = ((float)t0_2.tv_sec + t0_2.tv_nsec / 1000000000.0) - ((float)t0_1.tv_sec + t0_1.tv_nsec / 1000000000.0);
		//printf ( "\nTiempo en transformar: %f\n", ( ( float ) t0_2.tv_sec+t0_2.tv_nsec/1000000000.0 )- ( ( float ) t0_1.tv_sec+t0_1.tv_nsec/1000000000.0 ) );
		printf("\nTiempo en transformar: %f\n", time_patron);

	} // if patron_vacio
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ICP3D");
	ros::NodeHandle n;

	log_pos = fopen("log_pos.txt", "w");
	log_icp = fopen("log_icp.txt", "w");
	log_times = fopen("log_times.txt", "w");

	ros::Subscriber sub = n.subscribe("/velodyne_points", 1, receiveSensor);
	//pub_filtered_map = n.advertise<sensor_msgs::PointCloud>("/map_filtered", 1);
	pub_filtered_map2 = n.advertise<sensor_msgs::PointCloud2>("/map_filtered2", 1);
	ros::spin();

	return 0;
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

	//timespec t_ini_exp;
	//clock_gettime ( CLOCK_MONOTONIC, &t_ini_exp ); // I have to add this function

	int i, j, k_i, k_j;
	int max=0;
	
	sensor_msgs::PointCloud2 filtered_cloud_3D;
	
	// Initialize the matrix
	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < columns; j++)
		{
			V[i][j] = 0;
			cont_V[i][j] = 0;
		}
	}

	// Indicate the cell where the vessel is.
	V[rang][rang] = 4; //Position of the sensor (it is always the same because the map is local)

	float cloud_dist, cloud_range = rang;
	int iM, jM, kM, tam, tam_filter;
	int iM_min, iM_max, jM_min, jM_max;
	int count_close = 0;
	geometry_msgs::Point puntonube;

	//filtered_cloud_3D = nube3d_uav;
	
	//tam = (int) nube3d_uav.size();
	tam = nube3d_uav.width * nube3d_uav.height;
		
	//tam_filter = filtered_cloud_3D.points.size();
	//tam_filter = filtered_cloud_3D.width * filtered_cloud_3D.height;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr fil_cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);  //Data to use in the map.
	pcl::PointCloud<pcl::PointXYZ>::Ptr fil_data_cloud(new pcl::PointCloud<pcl::PointXYZ>);	//Filtered point cloud to publish
	// Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
	pcl::fromROSMsg(nube3d_uav, *fil_cloud_XYZ);
	
	tam_filter = (int)fil_cloud_XYZ->points.size();
	
	//ROS_INFO("tam= %d \n", tam);
	ROS_INFO("tam_filter= %d \n", tam_filter);
	
	
	

	// Build the matrix
	for (i = 0; i < tam; i++) // nlements --> tamano
	{
		//fprintf(log_icp, "%lf %lf %lf\n", nube3d_uav.points[i].x, nube3d_uav.points[i].y, nube3d_uav.points[i].z);  //JAC
		//cloud_dist = hypot(nube3d_uav.points[i].x, nube3d_uav.points[i].y);
		cloud_dist = hypot(fil_cloud_XYZ->points[i].x, fil_cloud_XYZ->points[i].y);
		
		if (cloud_dist >= cloud_thres_distance)
		{

			iM = round(rang - cell_div * fil_cloud_XYZ->points[i].x);
			jM = round(rang - cell_div * fil_cloud_XYZ->points[i].y);

			if (cloud_dist < cloud_range && is_in_map(iM, jM) == 1 && readV(V, iM, jM) != 4)
			{

				if (V[iM][jM] != 1)
				{

					V[iM][jM] = 1; //JAC
					cont_V[iM][jM] = cont_V[iM][jM] + 1;
				}
				if (V[iM][jM] == 1 && cont_V[iM][jM] >= 1)
				{
					cont_V[iM][jM] = cont_V[iM][jM] + 1;
				}
				if (cont_V[iM][jM]>max){
					max=cont_V[iM][jM];
				}
			}

		} //if

	} //for

	//ROS_INFO("Max=%d\n", max);
	
	// Filtered map to send to the collision avoidance module --> V_map

	V_map = V;
	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < columns; j++)
		{
			if (cont_V[i][j] < 5 && V[i][j] == 1)
			{
				V_map[i][j] = 0;
			}
		}
	}


	// Esto es para confirmar con Rviz que se van filtrando las olas. Se publica un topic con los datos de la nube filtrados.
	// map1_

	for (i = 0; i < tam; i++) // nlements --> tamano
	{
		cloud_dist = hypot(fil_cloud_XYZ->points[i].x, fil_cloud_XYZ->points[i].y);

		if (cloud_dist >= cloud_thres_distance)
		{
			iM = round(rang - cell_div * fil_cloud_XYZ->points[i].x);
			jM = round(rang - cell_div * fil_cloud_XYZ->points[i].y);

			if (cloud_dist < cloud_range && is_in_map(iM, jM) == 1 && readV(V, iM, jM) != 4)
			{

				if (V[iM][jM] == 1 && V_map[iM][jM] == 0)
				{
					//fil_cloud_XYZ->points[i].x=0;
					//fil_cloud_XYZ->points[i].y=0;
					//fil_cloud_XYZ->points[i].z=0;
					//fil_cloud_XYZ->erase(fil_cloud_XYZ->points[i]);
				}
				else{
				  fil_data_cloud->points.push_back(fil_cloud_XYZ->points[i]);   
				}
			}

		} //if

	} //for

	
	tam_filter = (int)fil_data_cloud->points.size();
	
	//ROS_INFO("tam= %d \n", tam);
	ROS_INFO("tam_fil_data_cloud= %d \n", tam_filter);
	
	//sensor_msgs::convertPointCloudToPointCloud2(filtered_cloud_3D, filtered_cloud_3D2);

	
	//Convert the pointcloud to be used in ROS, usin variable sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*fil_data_cloud, filtered_cloud_3D);
	filtered_cloud_3D.header.frame_id = "velodyne";
	filtered_cloud_3D.header.stamp = ros::Time();
		
	//pub_filtered_map.publish(filtered_cloud_3D);
	//pub_filtered_map2.publish(filtered_cloud_3D2);
	pub_filtered_map2.publish(filtered_cloud_3D);
	

	// GENERATE FILES TO CHECK THE OUTPUTS AND THE GENERATION OF THE MATRIX.
	save_matrix("matrix.txt", V);
	save_matrix("contador.txt", cont_V);
	save_matrix("mapa.txt", V_map);

  
}

int is_in_map(int i, int j)
{
	if (i < 0 || i > rows - 1)
		return 0;
	if (j < 0 || j > columns - 1)
		return 0;
	return 1;
}

int readV(const VVI &V, int i, int j)
{
	if (i < rows && j < columns && i >= 0 && j >= 0)
		return V[i][j];
	else
		return -1;
}

// FILE TO SAVE A MATRIX
void save_matrix(char *fileName, const VVI &M)
{
	FILE *fp = fopen(fileName, "w");
	if (fp == NULL)
	{
		exit(EXIT_FAILURE);
	}
	char linea[100001];
	for (int i = 0; i < rows; i++)
	{
		linea[0] = '\0';
		for (int j = 0; j < columns; j++)
		{
			char buffer[10];
			sprintf(buffer, "%d \t ", M[i][j]);
			strcat(linea, buffer);
		}
		int len = strlen(linea);
		linea[len - 1] = '\n';
		fputs(linea, fp);
	}

	fclose(fp);
}
