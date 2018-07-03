#include <ros/ros.h>
#include <Eigen/Core>
#include <cfloat>
#include <vector>
#include <iostream>

void save_matrix(char *fileName, const std::vector<std::vector<int>> &M);
std::vector<Eigen::Vector3i> voxel_traversal(Eigen::Vector3d ray_start, Eigen::Vector3d ray_end);
int _bin_size = 1;
int rang, param, rr, cc;

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "voxtraversal_node");

    param = 1;
    if (param == 1)
    {
        rang = 20;
    }
    else
    {
        rang = 200;
    }

    rr = rang * 2 + 1;
    cc = rang * 2 + 1;

    std::vector<std::vector<int>> matrix(rr, std::vector<int>(cc));
    std::vector<std::vector<int>> thinMatrix(rr, std::vector<int>(cc));

    for (int i = 0; i < rr; i++)
    {
        for (int j = 0; j < cc; j++)
        {
            if ((j % 2 == 0 && i % 2 == 0) && (i < (rr - rr * 0.8) || i > (rr - rr * 0.2) || j < (rr - rr * 0.8) || j > (rr - rr * 0.2)))
            {
                matrix[i][j] = 1;
            }
            else
            {
                matrix[i][j] = 0;
            }
        }
    }
    save_matrix("/home/hector/Matlab_ws/testM1.txt", matrix);

    for (int i = 0; i < rr; i++)
    {
        for (int j = 0; j < cc; j++)
        {
            if (param == 1)
            {
                if ((i == 0) || ((j == 39 || j == 2) && i < 29) || ((i == 39) && 1 < j))
                {
                    Eigen::Vector3d ray_start(rang, rang, 0);
                    Eigen::Vector3d ray_end(i, j, 0);
                    // ROS_WARN("[] i: %i | j: %i", i, j);
                    std::vector<Eigen::Vector3i> ids = voxel_traversal(ray_start, ray_end);
                    for (int k = 0; k < ids.size(); k++)
                    {
                        Eigen::Vector3i pointer = ids[k];
                        if (matrix[pointer[0]][pointer[1]] == 1)
                        {
                            // ROS_WARN("Pointer0: %i Pointer1: %i | i: %i | j: %i ", pointer[0], pointer[1], i, j);
                            thinMatrix[pointer[0]][pointer[1]] = 1;
                            break;
                        }
                    }
                }
            }
            else
            {
                if ((i < 242 && j == 0) || (i == 0 && j < 242) || (i == 390 && 199 < j) || (199 < i && j == 390) || (252 < i && 252 < j))
                {
                    Eigen::Vector3d ray_start(rang, rang, 0);
                    Eigen::Vector3d ray_end(i, j, 0);
                    // ROS_WARN("[] i: %i | j: %i", i, j);
                    std::vector<Eigen::Vector3i> ids = voxel_traversal(ray_start, ray_end);
                    for (int k = 0; k < ids.size(); k++)
                    {
                        Eigen::Vector3i pointer = ids[k];
                        if (matrix[pointer[0]][pointer[1]] == 1)
                        {
                            // ROS_WARN("Pointer0: %i Pointer1: %i | i: %i | j: %i ", pointer[0], pointer[1], i, j);
                            thinMatrix[pointer[0]][pointer[1]] = 1;
                            break;
                        }
                    }
                }
            }
        }
    }

    matrix[rang][rang] = 1;
    thinMatrix[rang][rang] = 1;

    save_matrix("/home/hector/Matlab_ws/testM2.txt", thinMatrix);
    std::cout << "Matrix saved!" << std::endl;

    return 0;
}

void save_matrix(char *fileName, const std::vector<std::vector<int>> &M)
{
    FILE *fp = fopen(fileName, "w");
    if (fp == NULL)
    {
        exit(EXIT_FAILURE);
    }
    char linea[100001];
    for (int i = 0; i < rr; i++)
    {
        linea[0] = '\0';
        for (int j = 0; j < cc; j++)
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

std::vector<Eigen::Vector3i> voxel_traversal(Eigen::Vector3d ray_start, Eigen::Vector3d ray_end)
{
    // Reference  -- >  https://github.com/francisengelmann/fast_voxel_traversal

    std::vector<Eigen::Vector3i> visited_voxels;

    // This id of the first/current voxel hit by the ray.
    // Using floor (round down) is actually very important,
    // the implicit int-casting will round up for negative numbers.
    Eigen::Vector3i current_voxel(std::floor(ray_start[0] / _bin_size),
                                  std::floor(ray_start[1] / _bin_size),
                                  std::floor(ray_start[2] / _bin_size));

    // The id of the last voxel hit by the ray.
    // TODO: what happens if the end point is on a border?
    Eigen::Vector3i last_voxel(std::floor(ray_end[0] / _bin_size),
                               std::floor(ray_end[1] / _bin_size),
                               std::floor(ray_end[2] / _bin_size));

    // Compute normalized ray direction.
    Eigen::Vector3d ray = ray_end - ray_start;
    //ray.normalize();

    // In which direction the voxel ids are incremented.
    double stepX = (ray[0] >= 0) ? 1 : -1; // correct
    double stepY = (ray[1] >= 0) ? 1 : -1; // correct
    double stepZ = (ray[2] >= 0) ? 1 : -1; // correct

    // Distance along the ray to the next voxel border from the current position (tMaxX, tMaxY, tMaxZ).
    double next_voxel_boundary_x = (current_voxel[0] + stepX) * _bin_size; // correct
    double next_voxel_boundary_y = (current_voxel[1] + stepY) * _bin_size; // correct
    double next_voxel_boundary_z = (current_voxel[2] + stepZ) * _bin_size; // correct

    // tMaxX, tMaxY, tMaxZ -- distance until next intersection with voxel-border
    // the value of t at which the ray crosses the first vertical voxel boundary
    double tMaxX = (ray[0] != 0) ? (next_voxel_boundary_x - ray_start[0]) / ray[0] : DBL_MAX; //
    double tMaxY = (ray[1] != 0) ? (next_voxel_boundary_y - ray_start[1]) / ray[1] : DBL_MAX; //
    double tMaxZ = (ray[2] != 0) ? (next_voxel_boundary_z - ray_start[2]) / ray[2] : DBL_MAX; //

    // tDeltaX, tDeltaY, tDeltaZ --
    // how far along the ray we must move for the horizontal component to equal the width of a voxel
    // the direction in which we traverse the grid
    // can only be FLT_MAX if we never go in that direction
    double tDeltaX = (ray[0] != 0) ? _bin_size / ray[0] * stepX : DBL_MAX;
    double tDeltaY = (ray[1] != 0) ? _bin_size / ray[1] * stepY : DBL_MAX;
    double tDeltaZ = (ray[2] != 0) ? _bin_size / ray[2] * stepZ : DBL_MAX;

    Eigen::Vector3i diff(0, 0, 0);
    bool neg_ray = false;
    if (current_voxel[0] != last_voxel[0] && ray[0] < 0)
    {
        diff[0]--;
        neg_ray = true;
    }
    if (current_voxel[1] != last_voxel[1] && ray[1] < 0)
    {
        diff[1]--;
        neg_ray = true;
    }
    if (current_voxel[2] != last_voxel[2] && ray[2] < 0)
    {
        diff[2]--;
        neg_ray = true;
    }
    visited_voxels.push_back(current_voxel);
    if (neg_ray)
    {
        current_voxel += diff;
        visited_voxels.push_back(current_voxel);
    }

    // ROS_WARN("DEBUG2");
    while (last_voxel != current_voxel)
    {
        // std::cout << last_voxel << " != " << current_voxel << std::endl;
        if (tMaxX < tMaxY)
        {
            if (tMaxX < tMaxZ)
            {
                current_voxel[0] += stepX;
                tMaxX += tDeltaX;
            }
            else
            {
                current_voxel[2] += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
        else
        {
            if (tMaxY < tMaxZ)
            {
                current_voxel[1] += stepY;
                tMaxY += tDeltaY;
            }
            else
            {
                current_voxel[2] += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
        visited_voxels.push_back(current_voxel);
    }
    // ROS_WARN("DEBUG3");
    return visited_voxels;
}