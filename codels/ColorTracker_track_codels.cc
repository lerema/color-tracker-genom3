
#include "acColorTracker.h"
#include "fg3utils/trace_f.h"
#include "fg3utils/macros.h"
#include "tracking.hpp"

#include "ColorTracker_c_types.h"


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
            const or_sensor_intrinsics *intrinsics,
            const or_sensor_extrinsics *extrinsics,
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
    auto fx = intrinsics->fx;
    auto fy = intrinsics->fy;
    auto cx = intrinsics->cx;
    auto cy = intrinsics->cy;
    auto z = 3.0; // TODO: get from camera info
    Tracking::imageToWorld(image_x, image_y, world_x, world_y, world_z, fx, fy, cx, cy, z);

    // Tracked Pose
    TrackedPose->x = world_x;
    TrackedPose->data->y = world_y;
    TrackedPose->data->z = world_z;
    TrackedPose->data->roll = 0.0;
    TrackedPose->data->pitch = 0.0;
    TrackedPose->data->yaw = 0.0;
    *new_findings = true;
  }
  else {
    *new_findings = false;
  }
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
