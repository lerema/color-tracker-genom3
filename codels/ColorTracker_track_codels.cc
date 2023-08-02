
#include "acColorTracker.h"
#include "fg3utils/trace_f.h"
#include "fg3utils/macros.h"
#include "tracking.hpp"
#include <cstdio>

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
           const ColorTracker_DronePose *DronePose,
           const genom_context self)
{
  // Check if all ports are connected and available
  if (check_port_in_p(Frame))
  {
    CODEL_LOG_WARNING("Image port not connected");
    return ColorTracker_pause_start;
  }
  if (check_port_in_p(Intrinsics))
  {
    CODEL_LOG_WARNING("Intrinsics port not connected");
    return ColorTracker_pause_start;
  }
  if (check_port_in_p(Extrinsics))
  {
    CODEL_LOG_WARNING("Extrinsics port not connected");
    return ColorTracker_pause_start;
  }
  if (check_port_in_p(DronePose))
  {
    CODEL_LOG_WARNING("Drone Pose port not connected");
    return ColorTracker_pause_start;
  }
  return ColorTracker_ready;
}

/** Codel InitIDS of task track.
 *
 * Triggered by ColorTracker_ready.
 * Yields to ColorTracker_pause_ready, ColorTracker_main,
 *           ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT.
 */
genom_event
InitIDS(const ColorTracker_Frame *Frame,
        const ColorTracker_Intrinsics *Intrinsics,
        const ColorTracker_Extrinsics *Extrinsics,
        const ColorTracker_DronePose *DronePose,
        or_sensor_frame *image_frame, or_sensor_intrinsics *intrinsics,
        or_sensor_extrinsics *extrinsics,
        or_pose_estimator_state *frame_pose,
        or_ColorTrack_PlateSequence *plates,
        ColorTracker_BlobMap *blob_map, const genom_context self)
{
  or_sensor_frame *FrameData;
  or_sensor_intrinsics *IntrinsicsData;
  or_sensor_extrinsics *ExtrinsicsData;

  // Read ports
  if (Frame->read(self) == genom_ok && Frame->data(self))
    FrameData = Frame->data(self);
  else
  {
    ColorTracker_e_BAD_IMAGE_PORT_detail msg;
    snprintf(msg.message, sizeof(msg.message), "%s", "Failed to read image port. waiting");
    // return ColorTracker_e_BAD_IMAGE_PORT(&msg, self);
    //  warnx("%s", msg.message);
    return ColorTracker_pause_ready;
  }
  if (Intrinsics->read(self) == genom_ok && Intrinsics->data(self))
    IntrinsicsData = Intrinsics->data(self);
  else
  {
    ColorTracker_e_BAD_IMAGE_PORT_detail msg;
    snprintf(msg.message, sizeof(msg.message), "%s", "Failed to read intrinsics port");
    // return ColorTracker_e_BAD_IMAGE_PORT(&msg, self);
    // warnx("%s", msg.message);
    return ColorTracker_pause_ready;
  }
  if (Extrinsics->read(self) == genom_ok && Extrinsics->data(self))
    ExtrinsicsData = Extrinsics->data(self);
  else
  {
    ColorTracker_e_BAD_IMAGE_PORT_detail msg;
    snprintf(msg.message, sizeof(msg.message), "%s", "Failed to read extrinsics port");
    // warnx("%s", msg.message);
    // return ColorTracker_e_BAD_IMAGE_PORT(&msg, self);
    return ColorTracker_pause_ready;
  }
  if (DronePose->read(self) == genom_ok && DronePose->data(self))
    frame_pose = DronePose->data(self);
  else
  {
    ColorTracker_e_BAD_IMAGE_PORT_detail msg;
    snprintf(msg.message, sizeof(msg.message), "%s", "Failed to read pose port");
    // warnx("%s", msg.message);
    // return ColorTracker_e_BAD_IMAGE_PORT(&msg, self);
    return ColorTracker_pause_ready;
  }

  // Copy data
  *image_frame = *FrameData;
  *intrinsics = *IntrinsicsData;
  *extrinsics = *ExtrinsicsData;

  // Initialize plates info
  if (genom_sequence_reserve(&(plates->seq), 1) != -1)
  {
    ColorTracker_e_OUT_OF_MEM_detail msg;
    snprintf(msg.message, sizeof(msg.message), "%s", "Failed to reserve memory for plates");
    // warnx("%s", msg.message);
    return ColorTracker_e_OUT_OF_MEM(&msg, self);
  }

  // Initialize blob map
  blob_map->is_blobbed = false;
  blob_map->grid_map.origin_x = 0.0;
  blob_map->grid_map.origin_y = 0.0;
  blob_map->grid_map.width = 10.0;
  blob_map->grid_map.height = 10.0;
  blob_map->grid_map.resolution = 0.1;
  // for (int i = 0; i < blob_map->grid_map.width / blob_map->grid_map.resolution; i++)
  // {
  //   for (int j = 0; j < blob_map->grid_map.height / blob_map->grid_map.resolution; j++)
  //   {
  //     blob_map->grid_map.data[i][j] = 0;
  //   }
  // }
  blob_map->index = 0;

  return ColorTracker_main;
}

/** Codel TrackObject of task track.
 *
 * Triggered by ColorTracker_main.
 * Yields to ColorTracker_pause_main, ColorTracker_publish,
 *           ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT.
 */
genom_event
TrackObject(const or_sensor_frame *image_frame,
            const or_sensor_intrinsics *intrinsics,
            const or_sensor_extrinsics *extrinsics,
            const or_ColorTrack_ColorInfo *color,
            const or_pose_estimator_state *frame_pose,
            or_ColorTrack_PlateSequence *plates,
            ColorTracker_BlobMap *blob_map, bool *new_findings,
            const ColorTracker_OccupancyGrid *OccupancyGrid,
            const ColorTracker_PlatesInfo *PlatesInfo, bool debug,
            const genom_context self)
{
  bool is_object_found = false;
  double image_x = 0.0, image_y = 0.0;
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
    if (image_frame->bpp == 1)
      type = CV_8UC1;
    else if (image_frame->bpp == 2)
      type = CV_16UC1;
    else if (image_frame->bpp == 3)
      type = CV_8UC3;
    else if (image_frame->bpp == 4)
      type = CV_8UC4;
    else
    {
      ColorTracker_e_BAD_IMAGE_PORT_detail *msg;
      snprintf(msg->message, sizeof(msg->message), "%s", "Invalid image bpp");
      return ColorTracker_e_BAD_IMAGE_PORT(msg, self);
    }

    image = cv::Mat(
        cv::Size(image_frame->width, image_frame->height),
        type,
        image_frame->pixels._buffer,
        cv::Mat::AUTO_STEP);
  }
  is_object_found = Tracking::detectObject(image, color->b, color->g, color->r, color->threshold, image_x, image_y);

  if (is_object_found)
  {
    // Convert image coordinates to world coordinates
    double world_x, world_y, world_z;
    auto fx = intrinsics->calib.fx;
    auto fy = intrinsics->calib.fy;
    auto cx = intrinsics->calib.cx;
    auto cy = intrinsics->calib.cy;
    auto z = 3.0; // TODO: get from camera info
    Tracking::imageToWorld(image_x, image_y, world_x, world_y, world_z, fx, fy, cx, cy, z);

    // Convert relative world coordinates to absolute world coordinates
    double abs_world_x, abs_world_y, abs_world_z;
    abs_world_x = world_x + frame_pose->pos._value.x;
    abs_world_y = world_y + frame_pose->pos._value.y;
    abs_world_z = world_z + frame_pose->pos._value.z;

    // Add nearest neighbors to avoid duplicates
    // Tracking::nearestNeighbours(abs_world_x, abs_world_y, abs_world_z, plates);

    // Plates info
    or_ColorTrack_PlateSequence *PlatesInfoData = plates;
    write_port_p(PlatesInfo);
  }
  else
  {
    *new_findings = false;
    return ColorTracker_pause_main;
  }
  return ColorTracker_publish;
}

/** Codel PublishOG of task track.
 *
 * Triggered by ColorTracker_publish.
 * Yields to ColorTracker_main, ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT.
 */
genom_event
PublishOG(const ColorTracker_BlobMap *blob_map,
          const ColorTracker_OccupancyGrid *OccupancyGrid,
          const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return ColorTracker_main;
}
