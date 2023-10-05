
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

/* --- Function clear_findins ------------------------------------------- */

/** Codel ClearFindings of function clear_findins.
 *
 * Returns genom_ok.
 * Throws ColorTracker_e_OUT_OF_MEM.
 */
genom_event
ClearFindings(or_ColorTrack_PlateSequence *all_detected_plates,
              const genom_context self)
{
    for (int i = 0; i < all_detected_plates->seq._length; i++)
    {
        or_ColorTrack_PlateInfo *plate = &all_detected_plates->seq._buffer[i];
        delete plate;
    }
    all_detected_plates->seq._length = 0;
    all_detected_plates->seq._maximum = 0;
    return genom_ok;
}
