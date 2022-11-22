
#include "acColorTracker.h"
#include "fg3utils/trace_f.h"
#include "fg3utils/macros.h"
#include "tracking.hpp"

#include "ColorTracker_c_types.h"
#include <math.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>


/* --- Calibration ------------------------------------------------------ */
struct ColorTracker_calib {
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();              // intrinsic calibration matrix
    cv::Mat K_cv = cv::Mat::zeros(cv::Size(3,3), CV_32F);   // opencv representation of K
    cv::Mat D = cv::Mat::zeros(cv::Size(1,5), CV_32F);      // camera distortion coefs
    Eigen::Vector3d B_p_C = Eigen::Vector3d::Zero();          // translation from camera to body
    Eigen::Matrix3d B_R_C = Eigen::Matrix3d::Identity();      // rotation from body to camera
};

/* --- Helper func ------------------------------------------------------ */
void UpdateCalibrationParameters(const ColorTracker_Intrinsics *intrinsics,
                  const ColorTracker_Extrinsics *extrinsics,
                  ColorTracker_calib **calib, const genom_context self)
{
    // Init intr
    or_sensor_calibration* c = &(intrinsics->data(self)->calib);
    (*calib)->K_cv = (cv::Mat_<float>(3,3) <<
        c->fx, c->gamma, c->cx,
            0,    c->fy, c->cy,
            0,        0,     1
    );
    cv::cv2eigen((*calib)->K_cv, (*calib)->K);
    (*calib)->D = (cv::Mat_<float>(5,1) <<
        intrinsics->data(self)->disto.k1,
        intrinsics->data(self)->disto.k2,
        intrinsics->data(self)->disto.k3,
        intrinsics->data(self)->disto.p1,
        intrinsics->data(self)->disto.p2
    );

    // Init extr
    (*calib)->B_p_C <<
        extrinsics->data(self)->trans.tx,
        extrinsics->data(self)->trans.ty,
        extrinsics->data(self)->trans.tz;
    float r = extrinsics->data(self)->rot.roll;
    float p = extrinsics->data(self)->rot.pitch;
    float y = extrinsics->data(self)->rot.yaw;
    (*calib)->B_R_C <<
        cos(p)*cos(y), sin(r)*sin(p)*cos(y) - cos(r)*sin(y), cos(r)*sin(p)*cos(y) + sin(r)*sin(y),
        cos(p)*sin(y), sin(r)*sin(p)*sin(y) + cos(r)*cos(y), cos(r)*sin(p)*sin(y) - sin(r)*cos(y),
              -sin(p),                        sin(r)*cos(p),                        cos(r)*cos(p);
}

/* --- Task track ------------------------------------------------------- */


/** Codel FetchPorts of task track.
 *
 * Triggered by ColorTracker_start.
 * Yields to ColorTracker_pause_start, ColorTracker_ready.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT.
 */
genom_event
FetchPorts(const ColorTracker_Frame *Frame,
           const ColorTracker_Intrinsics *Intrinsics,
           const ColorTracker_Extrinsics *Extrinsics,
           const ColorTracker_Pose *Pose, const genom_context self)
{
  // Check if all ports are connected and available
  if (Frame->read(self) != genom_ok && Frame->data(self))
  {
    CODEL_LOG_WARNING("Failed to read raw image frame");
    return ColorTracker_pause_start;
  }
  if (Intrinsics->read(self) != genom_ok && Intrinsics->data(self))
  {
    CODEL_LOG_WARNING("Failed to read camera intrinsics");
    return ColorTracker_pause_start;
  }
  if (Extrinsics->read(self) != genom_ok && Extrinsics->data(self))
  {  CODEL_LOG_WARNING("Failed to read camera extrinsics");
    return ColorTracker_pause_start;
  }
  if (Pose->read(self) != genom_ok && Pose->data(self))
  {
    CODEL_LOG_WARNING("Failed to read robot pose");
    return ColorTracker_pause_start;
  }
  return ColorTracker_ready;
}


/** Codel InitIDS of task track.
 *
 * Triggered by ColorTracker_ready.
 * Yields to ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT.
 */
genom_event
InitIDS(const ColorTracker_Frame *Frame,
        ColorTracker_calib **CameraCalibration,
        const ColorTracker_Intrinsics *Intrinsics,
        const ColorTracker_Extrinsics *Extrinsics,
        or_sensor_frame *image_frame, or_sensor_intrinsics *intrinsics,
        or_sensor_extrinsics *extrinsics,
        or_rigid_body_state *tracked_pose,
        ColorTracker_BlobMap *blob_map, const genom_context self)
{
  // Read ports
  *image_frame = *Frame->data(self);
  *intrinsics = *Intrinsics->data(self);
  *extrinsics = *Extrinsics->data(self);

  UpdateCalibrationParameters(Intrinsics, Extrinsics, CameraCalibration, self);

  return ColorTracker_ether;
}

/* --- Activity track_object -------------------------------------------- */

/** Codel TrackObject of activity track_object.
 *
 * Triggered by ColorTracker_start.
 * Yields to ColorTracker_pause_start, ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT,
 *        ColorTracker_e_BAD_POSE_PORT, ColorTracker_e_BAD_OG_PORT,
 *        ColorTracker_e_BAD_TARGET_PORT, ColorTracker_e_OPENCV_ERROR.
 */
genom_event
TrackObject(const or_sensor_frame *image_frame,
            const ColorTracker_calib *CameraCalibration,
            const or_ColorTrack_ColorInfo *color,
            or_rigid_body_state *frame_pose,
            ColorTracker_BlobMap *blob_map, bool *new_findings,
            const ColorTracker_Frame *Frame,
            const ColorTracker_OccupancyGrid *OccupancyGrid,
            const ColorTracker_TrackedPose *TrackedPose, bool debug,
            const genom_context self)
{
  bool is_object_found = false;
  double image_x=0.0, image_y=0.0;
  // Convert frame to cv::Mat
    cv::Mat image;
    if (image_frame->compressed)
    {
        std::vector<uint8_t> buf;
        buf.assign(image_frame->pixels._buffer, image_frame->pixels._buffer + image_frame->pixels._length);
        imdecode(buf, cv::IMREAD_GRAYSCALE, &image);
    }
    else
    {
        int type;
        if      (image_frame->bpp == 1) type = CV_8UC1;
        else if (image_frame->bpp == 2) type = CV_16UC1;
        else if (image_frame->bpp == 3) type = CV_8UC3;
        else if (image_frame->bpp == 4) type = CV_8UC4;
        else return ColorTracker_e_BAD_IMAGE_PORT(self);

        image = cv::Mat(
            cv::Size(image_frame->width, image_frame->height),
            type,
            image_frame->pixels._buffer,
            cv::Mat::AUTO_STEP);
    }
  is_object_found = Tracking::detectObject(image, color->b, color->g, color->r, color->threshold, image_x, image_y);

  if (is_object_found) {
    // Convert image coordinates to world coordinates
    double world_x, world_y, world_z;

    auto z = 3.0; // TODO: get from camera info
    // Tracking::imageToWorld(image_x, image_y, world_x, world_y, world_z, fx, fy, cx, cy, z);

    // Tracked Pose
    // TrackedPose->pos.x = world_x;
    // TrackedPose->data->y = world_y;
    // TrackedPose->data->z = world_z;
    *new_findings = true;
  }
  else {
    *new_findings = false;
  }

  image.release();

  return ColorTracker_pause_start;
}


/* --- Activity publish_occupancy_grid ---------------------------------- */

/** Codel PublishOG of activity publish_occupancy_grid.
 *
 * Triggered by ColorTracker_start.
 * Yields to ColorTracker_pause_start, ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT,
 *        ColorTracker_e_BAD_OG_PORT, ColorTracker_e_OPENCV_ERROR.
 */
genom_event
PublishOG(const ColorTracker_BlobMap *blob_map,
          const ColorTracker_OccupancyGrid *OccupancyGrid,
          const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return ColorTracker_pause_start;
}
