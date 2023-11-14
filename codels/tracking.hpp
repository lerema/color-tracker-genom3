/*
 * Copyright (c) 2022-2022 LAAS/CNRS
 *
 * Author: Felix Ingrand - LAAS/CNRS
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#ifndef TRACKING_HPP
#define TRACKING_HPP
#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>
#include <eigen3/Eigen/Geometry>

#include "ColorTracker_c_types.h"
#include "acColorTracker.h"

using namespace std;

namespace Tracking
{
    bool detectObject(cv::Mat &image, cv::Mat &mask, const or_ColorTrack_ColorInfo *color, double &x, double &y, cv::Rect &bounding_box, const bool debug = false, const bool show_frames = false)
    {

        // Create the mask &initialize it to white (no color detected)
        // Create the thresholded image
        cv::Mat bgr = image.clone();
        int b = color->r;
        int g = color->g;
        int r = color->b;
        int tolerance = color->threshold;
        bool object_found = false;

        // We create the mask
        cv::inRange(bgr, cv::Scalar(b - tolerance, g - tolerance, r - tolerance), cv::Scalar(b + tolerance, g + tolerance, r + tolerance), mask);

        // Kernel for the morphological operations
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(1, 1));

        // Morphological opening (inverse because we have white pixels on black background)
        cv::dilate(mask, mask, kernel); // 1, 1
        cv::erode(mask, mask, kernel);  // 1, 1

        // Get Image Moments
        cv::Moments m = cv::moments(mask, true);
        double m10 = m.m10;
        double m01 = m.m01;
        double mA = m.m00;

        // Find contours and create a bounding box around the detected object
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        int largest_area = 0;
        int largest_contour_index = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            double a = cv::contourArea(contours[i], false);
            if (a > largest_area)
            {
                largest_area = a;
                largest_contour_index = i;
                bounding_box = cv::boundingRect(contours[i]);
            }
        }

        // Draw the largest contour using the bounding rectangle
        if (show_frames)
        {
            cv::Scalar color(255, 0, 0); // color of the contour in the
            cv::drawContours(image, contours, largest_contour_index, color, 2, 8, hierarchy);
            cv::rectangle(image, bounding_box, cv::Scalar(0, 255, 0), 1, 8, 0);
        }

        // Calculate the center of the object
        if (mA > 1000)
        {
            x = m10 / mA;
            y = m01 / mA;
            cv::circle(image, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);
            object_found = true;
        }
        else
        {
            x = -1;
            y = -1;
            object_found = false;
        }

        // Debug
        if (show_frames)
        {
            cv::imshow("Image Mask", mask);
            cv::imshow("Camera Image", image);
            cv::waitKey(1);
        }

        return object_found;
    }

    void imageToWorldCoordinates(const int &image_x, const int &image_y,
                                 const ColorTracker_CameraInfo *camera_info,
                                 const or_pose_estimator_state *drone_state,
                                 double &xw, double &yw, double &zw)
    {
        // Get the camera parameters
        double focal_length = camera_info->focal_length;
        double field_of_view = camera_info->field_of_view;
        double pixel_size = camera_info->pixel_size;
        double image_width = camera_info->image_size.width;
        double image_height = camera_info->image_size.height;

        double drone_x = drone_state->pos._value.x;
        double drone_y = drone_state->pos._value.y;
        double drone_z = drone_state->pos._value.z;
        double drone_qx = drone_state->att._value.qx;
        double drone_qy = drone_state->att._value.qy;
        double drone_qz = drone_state->att._value.qz;
        double drone_qw = drone_state->att._value.qw;

        // Calculate the distance to target with the pinhole camera model
        Eigen::Vector3d A(-(image_y - image_height / 2) * pixel_size,
                          -(image_x - image_width / 2) * pixel_size,
                          -focal_length);

        // Build Quaternion vector
        Eigen::Quaternion<double> Q(drone_qw, drone_qx, drone_qy, drone_qz);

        // Vector to Blob in world frame
        Eigen::Vector3d Vw = Q * A;

        // Build Drone Position vector
        Eigen::Vector3d P(drone_x, drone_y, drone_z);

        // Calculate Intersection
        double alpha = P(2) / Vw(2);

        // Calculate Blob Position in world frame
        Eigen::Vector3d Pw = P - alpha * Vw;

        xw = Pw(0);
        yw = Pw(1);
        zw = Pw(2);
    }

    void getRotationMatrix(const cv::Mat quaternion, cv::Mat R)
    {
        // Convert the quaternion to euler angles
        double roll, pitch, yaw;
        roll = atan2(2 * (quaternion.at<double>(0) * quaternion.at<double>(1) + quaternion.at<double>(2) * quaternion.at<double>(3)), 1 - 2 * (quaternion.at<double>(1) * quaternion.at<double>(1) + quaternion.at<double>(2) * quaternion.at<double>(2)));
        pitch = asin(2 * (quaternion.at<double>(0) * quaternion.at<double>(2) - quaternion.at<double>(3) * quaternion.at<double>(1)));
        yaw = atan2(2 * (quaternion.at<double>(0) * quaternion.at<double>(3) + quaternion.at<double>(1) * quaternion.at<double>(2)), 1 - 2 * (quaternion.at<double>(2) * quaternion.at<double>(2) + quaternion.at<double>(3) * quaternion.at<double>(3)));

        // Create the rotation matrix
        cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                       0, cos(roll), -sin(roll),
                       0, sin(roll), cos(roll));
        cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch),
                       0, 1, 0,
                       -sin(pitch), 0, cos(pitch));
        cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0,
                       sin(yaw), cos(yaw), 0,
                       0, 0, 1);

        // Combined rotation matrix
        R = R_z * R_y * R_x;
    }

    void getTranslationMatrix(const optional_or_t3d_pos drone_position, cv::Mat T)
    {
        // Create the translation matrix
        if (drone_position._present)
            T = (cv::Mat_<double>(3, 1) << drone_position._value.x, drone_position._value.y, drone_position._value.z);
        else
            T = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    }

    // Function to calculate the distance between two coordinates
    double distance(const or_ColorTrack_PlateInfo &c1, const or_ColorTrack_PlateInfo &c2)
    {
        double dx = c2.coord.x - c1.coord.x;
        double dy = c2.coord.y - c1.coord.y;
        // double dz = c2.coord.z - c1.coord.z;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Function to group coordinates based on a threshold
    std::vector<std::vector<or_ColorTrack_PlateInfo>> groupCoordinates(const std::vector<or_ColorTrack_PlateInfo> &coordinates, float &threshold)
    {
        std::vector<std::vector<or_ColorTrack_PlateInfo>> groups;
        std::vector<or_ColorTrack_PlateInfo> currentGroup;

        for (size_t i = 0; i < coordinates.size(); ++i)
        {
            if (currentGroup.empty())
            {
                currentGroup.push_back(coordinates[i]);
            }
            else
            {
                double dist = distance(coordinates[i], currentGroup.front());
                if (dist <= threshold)
                {
                    currentGroup.push_back(coordinates[i]);
                }
                else
                {
                    // Add the current group to the result
                    groups.push_back(currentGroup);
                    currentGroup.clear();
                    currentGroup.push_back(coordinates[i]);
                }
            }
        }

        // Add the last group to the result
        if (!currentGroup.empty())
        {
            groups.push_back(currentGroup);
        }

        return groups;
    }

    or_ColorTrack_PlateInfo findTargetPoint(const std::vector<or_ColorTrack_PlateInfo> &points)
    {
        // Apply K means to find the target point
        or_ColorTrack_PlateInfo target = points[0];
        double minAverageDistance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < points.size(); ++i)
        {
            double averageDistance = 0;
            for (size_t j = 0; j < points.size(); ++j)
            {
                averageDistance += distance(points[i], points[j]);
            }
            averageDistance /= points.size();

            if (averageDistance < minAverageDistance)
            {
                minAverageDistance = averageDistance;
                target = points[i];
            }
        }

        return target;
    }

    void nearestNeighbours(or_ColorTrack_PlateSequence *all_detections, or_ColorTrack_PlateSequence *plates, float threshold)
    {
        // Convert the sequence to a vector
        std::vector<or_ColorTrack_PlateInfo> plates_vector;
        for (size_t i = 0; i < all_detections->seq._length; ++i)
        {
            plates_vector.push_back(all_detections->seq._buffer[i]);
        }

        // Group the coordinates
        std::vector<std::vector<or_ColorTrack_PlateInfo>> groups =
            groupCoordinates(plates_vector, threshold);

        // Compute the target point for each group
        or_ColorTrack_PlateSequence result;
        result.seq._length = groups.size();
        result.seq._buffer = new or_ColorTrack_PlateInfo[result.seq._length];
        for (size_t i = 0; i < groups.size(); ++i)
        {
            or_ColorTrack_PlateInfo target = findTargetPoint(groups[i]);
            target.index = i;
            target.num_blobs = groups[i].size();
            target.state = 1;
            result.seq._buffer[i] = target;
        }

        // Filter the results which by condering only top 20% of the results
        std::sort(result.seq._buffer, result.seq._buffer + result.seq._length, [](const or_ColorTrack_PlateInfo &a, const or_ColorTrack_PlateInfo &b)
                  { return a.num_blobs > b.num_blobs; });
        result.seq._length = std::ceil(result.seq._length * 0.2);

        // Copy the result to the output
        *plates = result;
    };

} // namespace Tracking

#endif
