
#include "acColorTracker.h"

#include "ColorTracker_c_types.h"


/* --- Task track ------------------------------------------------------- */


/** Codel InitIDS of task track.
 *
 * Triggered by ColorTracker_start.
 * Yields to ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM.
 */
genom_event
InitIDS(or_rigid_body_state *tracked_pose,
        ColorTracker_BlobMap *blob_map,
        const ColorTracker_OccupancyGrid *OccupancyGrid,
        const ColorTracker_TrackedPose *TrackedPose,
        const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return ColorTracker_ether;
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
TrackObject(const or_sensor_intrinsics *intrinsics,
            const or_sensor_extrinsics *extrinsics,
            const or_sensor_frame *image_frame,
            const or_ColorTrack_PlateSequence *plates,
            const or_ColorTrack_ColorInfo *color, int32_t *obj_x,
            int32_t *obj_y, ColorTracker_BlobMap *blob_map,
            bool *new_findings,
            const ColorTracker_OccupancyGrid *OccupancyGrid,
            const ColorTracker_TrackedPose *TrackedPose,
            const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return ColorTracker_pause_start;
}


/* --- Activity publish_occupancy_grid ---------------------------------- */

/** Codel PublishOG of activity publish_occupancy_grid.
 *
 * Triggered by ColorTracker_start.
 * Yields to ColorTracker_pause_start, ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_OG_PORT,
 *        ColorTracker_e_OPENCV_ERROR.
 */
genom_event
PublishOG(const ColorTracker_BlobMap *blob_map,
          const ColorTracker_OccupancyGrid *OccupancyGrid,
          const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return ColorTracker_pause_start;
}
