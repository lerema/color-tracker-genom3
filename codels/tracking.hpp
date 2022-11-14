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

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/types_c.h>
// #include "opencv2/core/types_c.h"


using namespace std;
bool binarisation(IplImage* image, int b, int g, int r, int tolerance, CvPoint *coord, int *nbPixels) {
        int x, y;
        IplImage *mask, *bgr;
        IplConvKernel *kernel;
        int sommeX = 0, sommeY = 0;

        *nbPixels = 0;

        coord->x=0;
        coord->y=0;
        // Create the mask &initialize it to white (no color detected)
        mask = cvCreateImage(cvGetSize(image), image->depth, 1);
        // Create the bgr image
        bgr = cvCloneImage(image);
        // We create the mask
        cvInRangeS(bgr, cvScalar(b-tolerance, g-tolerance,r-tolerance), cvScalar(b+tolerance, g+tolerance,r+tolerance), mask);
        // Create kernels for the morphological operation
        kernel = cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_ELLIPSE);
        // Morphological opening (inverse because we have white pixels on black background)
        cvDilate(mask, mask, kernel, 1);
        cvErode(mask, mask, kernel, 1);
        // We go through the mask to look for the tracked object and get its gravity center
        for(x = 0; x < mask->width; x++) {
                for(y = 0; y < mask->height; y++) {
                        // If its a tracked pixel, count it to the center of gravity's calcul
                        if(((uchar *)(mask->imageData + y*mask->widthStep))[x] == 255) {
                                sommeX += x;
                                sommeY += y;
                                (*nbPixels)++;
                        }
                }
        }
        // Show the result of the mask image
    cvShowImage("DetectionCnam_Codels Mask", mask);
    //Image camera
    cvShowImage("image_camera",image);
    cvWaitKey(3);
        // We release the memory of kernels
        cvReleaseStructuringElement(&kernel);
        // We release the memory of the mask
        cvReleaseImage(&mask);
        // We release the memory of the hsv image
    cvReleaseImage(&bgr);
        //return nbPixels;
    if(*nbPixels > 0) {
      // gravcenter = ((int)(sommeX / (nbPixels)), (int)(sommeY / (*nbPixels)));
      coord->x = (int)(sommeX / (*nbPixels));
      coord->y = (int)(sommeY / (*nbPixels));
      cvCircle(image,*coord,10,cvScalar(0,0,255,0),2);
      cvShowImage("image_camera",image);
      cvWaitKey(3);
      return true;
    } else {
      coord->x = -1;
      coord->y = -1;
      return false;
    }
}



#endif