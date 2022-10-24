
#include "acCT_drone.h"

#include "CT_drone_c_types.h"


/* --- Task track ------------------------------------------------------- */


/** Codel InitIDS of task track.
 *
 * Triggered by CT_drone_start.
 * Yields to CT_drone_ether.
 * Throws CT_drone_e_OUT_OF_MEM.
 */
genom_event
InitIDS(or_rigid_body_state *tracked_pose, CT_drone_BlobMap *blob_map,
        const CT_drone_OccupancyGrid *OccupancyGrid,
        const CT_drone_TrackedPose *TrackedPose,
        const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return CT_drone_ether;
}


/* --- Activity track_object -------------------------------------------- */

/** Codel TrackObject of activity track_object.
 *
 * Triggered by CT_drone_start.
 * Yields to CT_drone_pause_start, CT_drone_ether.
 * Throws CT_drone_e_OUT_OF_MEM, CT_drone_e_BAD_IMAGE_PORT,
 *        CT_drone_e_BAD_POSE_PORT, CT_drone_e_BAD_OG_PORT,
 *        CT_drone_e_BAD_TARGET_PORT, CT_drone_e_OPENCV_ERROR.
 */
genom_event
TrackObject(const or_sensor_intrinsics *intrinsics,
            const or_sensor_extrinsics *extrinsics,
            const or_sensor_frame *image_frame,
            const or_ColorTrack_PlateSequence *plates,
            const or_ColorTrack_ColorInfo *color, int32_t *obj_x,
            int32_t *obj_y, CT_drone_BlobMap *blob_map,
            bool *new_findings,
            const CT_drone_OccupancyGrid *OccupancyGrid,
            const CT_drone_TrackedPose *TrackedPose,
            const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return CT_drone_pause_start;
}


/* --- Activity publish_occupancy_grid ---------------------------------- */

/** Codel PublishOG of activity publish_occupancy_grid.
 *
 * Triggered by CT_drone_start.
 * Yields to CT_drone_pause_start, CT_drone_ether.
 * Throws CT_drone_e_OUT_OF_MEM, CT_drone_e_BAD_OG_PORT,
 *        CT_drone_e_OPENCV_ERROR.
 */
genom_event
PublishOG(const CT_drone_BlobMap *blob_map,
          const CT_drone_OccupancyGrid *OccupancyGrid,
          const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return CT_drone_pause_start;
}
