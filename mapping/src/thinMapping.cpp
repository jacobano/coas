#include <mapping/thinMapping.h>

#include <vector>
#include <fstream>
#include <sstream>

ThinMapping::ThinMapping()
{
    n = ros::NodeHandle();
    params();
    // // Subscriptions
    matrix_sub = n.subscribe("/v_map", 1, &ThinMapping::matrix_cb, this);

    // // Publishers
    // pub_trackBoundingBoxArray = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("/trackBoundingBoxes", 1);

    loop();
}

ThinMapping::~ThinMapping()
{
}

void ThinMapping::params()
{
    ros::NodeHandle nparam("~");
    if (nparam.getParam("phase", phase))
    {
        ROS_WARN("Got ThinMapping param phase: %i", phase);
        if (phase == 0)
        {
            rang = 100 * 2;
        }
        if (phase == 1)
        {
            rang = 10 * 2;
        }
        rr = 2 * rang + 1;
        cc = 2 * rang + 1;
    }
    else
    {
        phase = 0;
        rang = 100 * 2;
        rr = 2 * rang + 1;
        cc = 2 * rang + 1;
        ROS_WARN("Failed to get ThinMapping param phase: %i", phase);
    }
}

void ThinMapping::matrix_cb(const mapping::vectorVector input)
{
    ros::Time begin = ros::Time::now();
    mapping::vectorVector rows = input;
    std::vector<std::vector<int>> matrix(rr, std::vector<int>(cc));
    std::vector<std::vector<int>> thinMatrix(rr, std::vector<int>(cc));

    for (int i = 0; i < rows.rows.size(); i++)
    {
        mapping::vectorInt col = rows.rows[i];
        for (int j = 0; j < col.columns.size(); j++)
        {
            matrix[i][j] = col.columns[j];
        }
    }

    for (int i = 0; i < rows.rows.size(); i++)
    {
        mapping::vectorInt col = rows.rows[i];
        for (int j = 0; j < col.columns.size(); j++)
        {
            if (i == 0 || j == 0 || i == rows.rows.size() - 1 || j == col.columns.size() - 1)
            {
                Eigen::Vector3d ray_start(rang, rang, 0);
                Eigen::Vector3d ray_end(i, j, 0);

                std::vector<Eigen::Vector3i> ids = voxel_traversal(ray_start, ray_end);
                // std::cout << "Voxel size: " << _bin_size << std::endl;
                // std::cout << "Starting position: " << ray_start.transpose() << std::endl;
                // std::cout << "Ending position: " << ray_end.transpose() << std::endl;
                // std::cout << "Voxel ID's from start to end:" << std::endl;

                for (int k = 0; k < ids.size(); k++)
                {
                    Eigen::Vector3i pointer = ids[k];
                    if (matrix[pointer[0]][pointer[1]] == 1)
                    {
                        thinMatrix[pointer[0]][pointer[1]] = 1;
                        break;
                    }
                }

                // for (auto &i : ids)
                // {
                //     std::cout << "> " << i.transpose() << std::endl;
                // }
            }
        }
    }
    save_matrix("thinMatrix.txt", thinMatrix);

    std::cout << "[ THMP] Time: " << ros::Time::now() - begin << std::endl;
}

void ThinMapping::save_matrix(char *fileName, const std::vector<std::vector<int>> &M)
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

float ThinMapping::dist2Points(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float distance;
    distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) + ((z2 - z1) * (z2 - z1)));
    return distance;
}

std::vector<Eigen::Vector3i> ThinMapping::voxel_traversal(Eigen::Vector3d ray_start, Eigen::Vector3d ray_end)
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

    while (last_voxel != current_voxel)
    {
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
    return visited_voxels;
}

void ThinMapping::loop()
{
    while (ros::ok())
    {
        sleep(0.1);
        ros::spinOnce();
    }
}