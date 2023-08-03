
#include "acColorTracker.h"
#include "fg3utils/trace_f.h"
#include "fg3utils/macros.h"
#include <cstdio>

#include "ColorTracker_c_types.h"

/* --- Function set_color ----------------------------------------------- */

/** Codel SetColor of function set_color.
 *
 * Returns genom_ok.
 * Throws ColorTracker_e_OPENCV_ERROR.
 */
genom_event
SetColor(const or_ColorTrack_ColorInfo *color_info,
         or_ColorTrack_ColorInfo *color, const genom_context self)
{
  *color = *color_info;
  return genom_ok;
}

/* --- Function set_map_size -------------------------------------------- */

/** Codel SetMapSize of function set_map_size.
 *
 * Returns genom_ok.
 * Throws ColorTracker_e_BAD_OG_PORT.
 */
genom_event
SetMapSize(uint8_t map_width, uint8_t map_height,
           ColorTracker_BlobMap *blob_map, const genom_context self)
{
  if (map_width <= 0 || map_height <= 0)
  {
    ColorTracker_e_BAD_OG_PORT_detail *msg;
    CODEL_LOG_ERROR("Invalid map size: %f x %f", map_width, map_height);
    snprintf(msg->message, sizeof(msg->message), "%s", "Acquired invalid map size");
    return ColorTracker_e_BAD_OG_PORT(msg, self);
  }

  blob_map->grid_map.width = map_width;
  blob_map->grid_map.height = map_height;
  // blob_map->grid_map.data = (uint8_t *)malloc(map_width * map_height);

  return genom_ok;
}

/* --- Function set_debug ----------------------------------------------- */

/** Codel SetDebug of function set_debug.
 *
 * Returns genom_ok.
 */
genom_event
SetDebug(bool is_debug_mode, bool *debug, const genom_context self)
{
  *debug = is_debug_mode;
  return genom_ok;
}

/* --- Function show_image_frames --------------------------------------- */

/** Codel ShowFrames of function show_image_frames.
 *
 * Returns genom_ok.
 */
genom_event
ShowFrames(bool show_cv_frames, bool *show_frames,
           const genom_context self)
{
  *show_frames = show_cv_frames;
  return genom_ok;
}

/* --- Function set_verbose_level --------------------------------------- */

/** Codel SetVerboseLevel of function set_verbose_level.
 *
 * Returns genom_ok.
 */
genom_event
SetVerboseLevel(uint8_t verbose_level, uint8_t *v_level,
                const genom_context self)
{
  *v_level = verbose_level;
  return genom_ok;
}

/* --- Function set_distance_threshold ---------------------------------- */

/** Codel SetDistanceThreshold of function set_distance_threshold.
 *
 * Returns genom_ok.
 */
genom_event
SetDistanceThreshold(float tolerance, float *distance_threshold,
                     const genom_context self)
{
  *distance_threshold = tolerance;
  return genom_ok;
}

/* --- Function start_color_tracking ------------------------------------ */

/** Codel SetStartTracking of function start_color_tracking.
 *
 * Returns genom_ok.
 */
genom_event
SetStartTracking(bool *start_tracking, const genom_context self)
{
  *start_tracking = true;
  return genom_ok;
}

/* --- Function stop_color_tracking ------------------------------------- */

/** Codel SetStopTracking of function stop_color_tracking.
 *
 * Returns genom_ok.
 */
genom_event
SetStopTracking(bool *start_tracking, const genom_context self)
{
  *start_tracking = false;
  return genom_ok;
}
