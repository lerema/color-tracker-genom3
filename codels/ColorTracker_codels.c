
#include "acColorTracker.h"

#include "ColorTracker_c_types.h"


/* --- Function set_color ----------------------------------------------- */

/** Codel SetColor of function set_color.
 *
 * Returns genom_ok.
 * Throws ColorTracker_e_OPENCV_ERROR.
 */
genom_event
SetColor(const or_ColorTrack_ColorInfo *color,
         const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Function get_color ----------------------------------------------- */

/** Codel GetColor of function get_color.
 *
 * Returns genom_ok.
 * Throws ColorTracker_e_OPENCV_ERROR.
 */
genom_event
GetColor(or_ColorTrack_ColorInfo *color, const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Function get_image_frame ----------------------------------------- */

/** Codel GetImageFrame of function get_image_frame.
 *
 * Returns genom_ok.
 * Throws ColorTracker_e_BAD_IMAGE_PORT.
 */
genom_event
GetImageFrame(or_sensor_frame *image_frame, const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}
