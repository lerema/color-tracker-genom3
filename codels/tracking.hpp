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

#include "ColorTracker_c_types.h"

using namespace std;

namespace Tracking
{
    bool detectObject(cv::Mat &image, int b, int g, int r, int tolerance, double &x, double &y, bool debug = false, bool show_frames = false)
    {

        // Create the mask &initialize it to white (no color detected)
        cv::Mat mask = cv::Mat(image.size(), image.type());
        // Create the thresholded image
        cv::Mat bgr = image.clone();

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

        // delete *kernel;
        // delete *mask;
        // delete bgr;

        if (debug)
        {
            cv::imshow("Image Mask", mask);
            cv::imshow("Camera Image", image);
            cv::waitKey(1);
        }
        if (mA > 1000)
        {
            x = m10 / mA;
            y = m01 / mA;
            cv::circle(image, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
        }
        else
        {
            x = -1;
            y = -1;
            return false;
        }

        if (debug || show_frames)
        {
            cv::imshow("Image Mask", mask);
            cv::imshow("Camera Image", image);
            cv::waitKey(1);
        }

        return true;
    }

    void imageToWorld(double x, double y, double &xw, double &yw,
                      double &zw, double fx, double fy, double cx,
                      double cy, double z)
    {
        // TODO: Currently the conversion behaves more like a blob detected and returns the x, y and z of the reference directly
        // xw = (x - cx) * z / fx;
        // yw = (y - cy) * z / fy;
        // zw = z;

        // Convert to world coordinates
    }

    // Function to calculate the distance between two coordinates
    double distance(const or_ColorTrack_PlateInfo &c1, const or_ColorTrack_PlateInfo &c2)
    {
        double dx = c2.coord.x - c1.coord.x;
        double dy = c2.coord.y - c1.coord.y;
        double dz = c2.coord.z - c1.coord.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
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
        // Choose an arbitrary starting point as the initial target point
        or_ColorTrack_PlateInfo target = points[0];
        double minAverageDistance = std::numeric_limits<double>::max();

        for (const auto &p : points)
        {
            double totalDistance = 0.0;
            for (const auto &other : points)
            {
                totalDistance += distance(p, other);
            }

            double averageDistance = totalDistance / points.size();
            if (averageDistance < minAverageDistance)
            {
                target = p;
                minAverageDistance = averageDistance;
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

        // Compute the average of each group
        or_ColorTrack_PlateSequence result;
        result.seq._length = groups.size();
        result.seq._buffer = new or_ColorTrack_PlateInfo[result.seq._length];
        for (size_t i = 0; i < groups.size(); ++i)
        {
            or_ColorTrack_PlateInfo average;
            average.coord.x = 0;
            average.coord.y = 0;
            average.coord.z = 0;
            average.index = i;
            for (size_t j = 0; j < groups[i].size(); ++j)
            {
                average.coord.x += groups[i][j].coord.x;
                average.coord.y += groups[i][j].coord.y;
                average.coord.z += groups[i][j].coord.z;
            }
            average.coord.x /= groups[i].size();
            average.coord.y /= groups[i].size();
            average.coord.z /= groups[i].size();
            average.num_blobs += groups[i].size(); // The number of blobs is the number of points in the group
            average.state = 1;                     // Make it as interesting
            result.seq._buffer[i] = average;
        }

        // Copy the result to the output
        *plates = result;
    };

} // namespace Tracking

#endif