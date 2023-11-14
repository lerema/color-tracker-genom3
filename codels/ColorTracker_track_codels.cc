
#include "acColorTracker.h"
#include "fg3utils/trace_f.h"
#include "fg3utils/macros.h"
#include "tracking.hpp"
#include <cstdio>

#include "ColorTracker_c_types.h"

/* --- Task track ------------------------------------------------------- */

/** Codel InitPort of task track.
 *
 * Triggered by ColorTracker_start.
 * Yields to ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT.
 */
genom_event
InitPort(const ColorTracker_output *output, const genom_context self)
{
    output->open("frame", self);
    output->open("mask", self);

    return ColorTracker_ether;
}

/* --- Activity color_track --------------------------------------------- */

/** Codel FetchPorts of activity color_track.
 *
 * Triggered by ColorTracker_start.
 * Yields to ColorTracker_pause_start, ColorTracker_poll.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT,
 *        ColorTracker_e_BAD_POSE_PORT, ColorTracker_e_BAD_OG_PORT,
 *        ColorTracker_e_BAD_TARGET_PORT, ColorTracker_e_OPENCV_ERROR.
 */
genom_event
FetchPorts(const ColorTracker_Frame *Frame,
           const ColorTracker_Intrinsics *Intrinsics,
           const ColorTracker_Extrinsics *Extrinsics,
           const ColorTracker_DronePose *DronePose,
           or_ColorTrack_PlateSequence *plates,
           or_ColorTrack_PlateSequence *all_detected_plates,
           bool debug, const genom_context self)
{
    // Check if all ports are connected and available
    if (!check_port_in_p(Frame))
    {
        CODEL_LOG_WARNING("Image port not connected");
        return ColorTracker_pause_start;
    }
    if (!check_port_in_p(Intrinsics))
    {
        CODEL_LOG_WARNING("Intrinsics port not connected");
        return ColorTracker_pause_start;
    }
    if (!check_port_in_p(Extrinsics))
    {
        CODEL_LOG_WARNING("Extrinsics port not connected");
        return ColorTracker_pause_start;
    }
    if (!check_port_in_p(DronePose))
    {
        CODEL_LOG_WARNING("Drone Pose port not connected");
        return ColorTracker_pause_start;
    }

    // Initialize plates info
    if (genom_sequence_reserve(&(plates->seq), 15) == -1)
    {
        ColorTracker_e_OUT_OF_MEM_detail msg;
        snprintf(msg.message, sizeof(msg.message), "%s", "Failed to reserve memory for plates");
        // warnx("%s", msg.message);
        return ColorTracker_e_OUT_OF_MEM(&msg, self);
    }

    if (genom_sequence_reserve(&(all_detected_plates->seq), 5000000) == -1)
    {
        ColorTracker_e_OUT_OF_MEM_detail msg;
        snprintf(msg.message, sizeof(msg.message), "%s", "Failed to reserve memory for plates");
        // warnx("%s", msg.message);
        return ColorTracker_e_OUT_OF_MEM(&msg, self);
    }

    if (debug)
    {
        CODEL_LOG_INFO(2, 1, "All ports connected, fetching data");
    }

    return ColorTracker_poll;
}

/** Codel FetchDataFromPorts of activity color_track.
 *
 * Triggered by ColorTracker_poll.
 * Yields to ColorTracker_pause_poll, ColorTracker_main,
 *           ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT,
 *        ColorTracker_e_BAD_POSE_PORT, ColorTracker_e_BAD_OG_PORT,
 *        ColorTracker_e_BAD_TARGET_PORT, ColorTracker_e_OPENCV_ERROR.
 */
genom_event
FetchDataFromPorts(const ColorTracker_Frame *Frame,
                   const ColorTracker_Intrinsics *Intrinsics,
                   const ColorTracker_Extrinsics *Extrinsics,
                   const ColorTracker_DronePose *DronePose,
                   or_sensor_frame *image_frame,
                   or_sensor_intrinsics *intrinsics,
                   or_sensor_extrinsics *extrinsics,
                   ColorTracker_CameraInfo *camera_info,
                   or_pose_estimator_state *frame_pose, bool debug,
                   const genom_context self)
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
        return ColorTracker_pause_poll;
    }
    if (Intrinsics->read(self) == genom_ok && Intrinsics->data(self))
        IntrinsicsData = Intrinsics->data(self);
    else
    {
        ColorTracker_e_BAD_IMAGE_PORT_detail msg;
        snprintf(msg.message, sizeof(msg.message), "%s", "Failed to read intrinsics port");
        // return ColorTracker_e_BAD_IMAGE_PORT(&msg, self);
        // warnx("%s", msg.message);
        return ColorTracker_pause_poll;
    }
    if (Extrinsics->read(self) == genom_ok && Extrinsics->data(self))
        ExtrinsicsData = Extrinsics->data(self);
    else
    {
        ColorTracker_e_BAD_IMAGE_PORT_detail msg;
        snprintf(msg.message, sizeof(msg.message), "%s", "Failed to read extrinsics port");
        // warnx("%s", msg.message);
        // return ColorTracker_e_BAD_IMAGE_PORT(&msg, self);
        return ColorTracker_pause_poll;
    }
    if (DronePose->read(self) == genom_ok && DronePose->data(self))
        frame_pose = DronePose->data(self);
    else
    {
        ColorTracker_e_BAD_IMAGE_PORT_detail msg;
        snprintf(msg.message, sizeof(msg.message), "%s", "Failed to read pose port");
        // warnx("%s", msg.message);
        // return ColorTracker_e_BAD_IMAGE_PORT(&msg, self);
        return ColorTracker_pause_poll;
    }

    // Copy data
    *image_frame = *FrameData;
    *intrinsics = *IntrinsicsData;
    *extrinsics = *ExtrinsicsData;

    // Calculate Focal Distance from intrinsics
    camera_info->focal_length = intrinsics->calib.fx / 1000; //  focal length in mm

    // Calculate the FoV of the camera
    camera_info->field_of_view = 2 * atan(image_frame->width / (2 * camera_info->focal_length * 1000));
    // double fov_y = 2 * atan(intrinsics->height / (2 * focal_length));

    // Pixel size in mm
    camera_info->pixel_size = (2 * camera_info->focal_length * tan(camera_info->field_of_view / 2)) / image_frame->width;
    // double pixel_size_y = (2 * focal_length * tan(fov_y / 2)) / intrinsics->height;

    // Image Size
    camera_info->image_size.width = image_frame->width;
    camera_info->image_size.height = image_frame->height;

    if (debug)
    {
        CODEL_LOG_INFO(2, 1, "Camera Info: Focal Length: %f, Field of View: %f, Pixel Size: %f", camera_info->focal_length, camera_info->field_of_view, camera_info->pixel_size);
    }

    return ColorTracker_main;
}

/** Codel TrackObject of activity color_track.
 *
 * Triggered by ColorTracker_main.
 * Yields to ColorTracker_main, ColorTracker_poll,
 *           ColorTracker_publish, ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT,
 *        ColorTracker_e_BAD_POSE_PORT, ColorTracker_e_BAD_OG_PORT,
 *        ColorTracker_e_BAD_TARGET_PORT, ColorTracker_e_OPENCV_ERROR.
 */
genom_event
TrackObject(bool start_tracking, const or_sensor_frame *image_frame,
            const ColorTracker_CameraInfo *camera_info,
            const or_ColorTrack_ColorInfo *color,
            const ColorTracker_DronePose *DronePose,
            float distance_threshold,
            or_ColorTrack_PlateSequence *plates,
            or_ColorTrack_PlateSequence *all_detected_plates,
            const ColorTracker_Pose *camera_pose,
            const ColorTracker_PlatesInfo *PlatesInfo,
            const ColorTracker_output *output, bool debug,
            bool show_frames, bool publish_og,
            const genom_context self)
{

    if (!start_tracking)
    {
        if (debug)
        {
            CODEL_LOG_WARNING("Color Tracking not running.");
        }
        return ColorTracker_poll;
    }
    bool is_object_found = false;
    double image_x = 0.0, image_y = 0.0;
    // Convert frame to cv::Mat
    cv::Mat image;
    if (image_frame->compressed)
    {
        std::vector<uint8_t> buf;
        buf.assign(image_frame->pixels._buffer, image_frame->pixels._buffer + image_frame->pixels._length);
        imdecode(buf, cv::IMREAD_COLOR, &image);
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
        // cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    }

    cv::Rect bounding_box;
    cv::Mat mask = cv::Mat(image.size(), image.type());
    is_object_found = Tracking::detectObject(image, mask, color, image_x, image_y, bounding_box, debug, show_frames);

    // Publish the detected frame
    output->data("frame", self)->width = image.cols;
    output->data("frame", self)->height = image.rows;
    output->data("frame", self)->bpp = 3;
    output->data("frame", self)->compressed = false;
    output->data("frame", self)->pixels._buffer = image.data;
    output->data("frame", self)->pixels._length = image.cols * image.rows * 3;
    output->data("frame", self)->ts.sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    output->data("frame", self)->ts.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 1000000000;
    output->write("frame", self);

    output->data("mask", self)->width = mask.cols;
    output->data("mask", self)->height = mask.rows;
    output->data("mask", self)->bpp = 1;
    output->data("mask", self)->compressed = false;
    output->data("mask", self)->pixels._buffer = mask.data;
    output->data("mask", self)->pixels._length = mask.cols * mask.rows;
    output->data("mask", self)->ts.sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    output->data("mask", self)->ts.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 1000000000;
    output->write("mask", self);

    if (is_object_found)
    {
        if (debug)
        {
            CODEL_LOG_INFO(2, 1, "Object found at (%f, %f)", image_x, image_y);
        }

        // Check if the drone position is fetched
        or_pose_estimator_state *DronePoseData;
        DronePoseData = DronePose->data(self);
        if (!DronePoseData->pos._present)
        {
            CODEL_LOG_WARNING("Drone position is not fetched yet");
            ColorTracker_e_BAD_POSE_PORT_detail msg;
            snprintf(msg.message, sizeof(msg.message), "%s",
                     "Drone position is not fetched yet");
            return ColorTracker_e_BAD_POSE_PORT(&msg, self);
        }

        double target_x = 0.0, target_y = 0.0, target_z = 0.0;
        Tracking::imageToWorldCoordinates(image_x, image_y, camera_info, DronePoseData, target_x, target_y, target_z);

        if (debug)
        {
            CODEL_LOG_WARNING("World coordinates: (%f, %f, %f)", target_x, target_y, target_z);
            CODEL_LOG_WARNING("Drone coorindates: (%f, %f, %f)", DronePoseData->pos._value.x, DronePoseData->pos._value.y, DronePoseData->pos._value.z);
        }

        // Convert relative world coordinates to absolute world coordinates
        or_ColorTrack_PlateInfo plate;
        plate.coord.x = target_x;
        plate.coord.y = target_y;
        plate.coord.z = target_z;
        plate.index = all_detected_plates->seq._length + 1;

        all_detected_plates->seq._buffer[all_detected_plates->seq._length] = plate;
        all_detected_plates->seq._length++;

        // Gather nearest neighbors to avoid duplicates
        Tracking::nearestNeighbours(all_detected_plates, plates, distance_threshold);

        // Plates info
        PlatesInfo->data(self)->seq._length = plates->seq._length;
        PlatesInfo->data(self)->seq._buffer = plates->seq._buffer;
        PlatesInfo->data(self)->seq._maximum = plates->seq._length;
        PlatesInfo->data(self)->num_interesing_spots = plates->seq._length;
        PlatesInfo->write(self);
    }
    else
    {
        if (debug)
        {
            CODEL_LOG_WARNING("Object not found");
        }
    }
    if (publish_og)
        return ColorTracker_publish;
    else
        return ColorTracker_poll;
}

/** Codel PublishOG of activity color_track.
 *
 * Triggered by ColorTracker_publish.
 * Yields to ColorTracker_poll, ColorTracker_ether.
 * Throws ColorTracker_e_OUT_OF_MEM, ColorTracker_e_BAD_IMAGE_PORT,
 *        ColorTracker_e_BAD_POSE_PORT, ColorTracker_e_BAD_OG_PORT,
 *        ColorTracker_e_BAD_TARGET_PORT, ColorTracker_e_OPENCV_ERROR.
 */
genom_event
PublishOG(const ColorTracker_Size *object_size,
          const ColorTracker_Size *map_size,
          const ColorTracker_PlatesInfo *PlatesInfo,
          const ColorTracker_OccupancyGrid *OccupancyGrid,
          const genom_context self)
{
    // Reserve memory
    or_Environment_OccupancyGrid *grid_map = OccupancyGrid->data(self);
    if (genom_sequence_reserve(&(grid_map->data), map_size->width * map_size->height) == -1)
    {
        ColorTracker_e_OUT_OF_MEM_detail msg;
        snprintf(msg.message, sizeof(msg.message), "%s", "Failed to reserve memory for grid map");
        // warnx("%s", msg.message);
        return ColorTracker_e_OUT_OF_MEM(&msg, self);
    }

    // Inflate the grid cells with the size of the object
    int grid_width = (int)round(object_size->width / grid_map->resolution);
    int grid_height = (int)round(object_size->height / grid_map->resolution);

    grid_map->width = map_size->width;
    grid_map->height = map_size->height;
    grid_map->resolution = or_Environment_MAP_RESOLUTION;
    grid_map->origin_x = 0.0;
    grid_map->origin_y = 0.0;
    grid_map->data._length = map_size->width * map_size->height;
    grid_map->data._maximum = map_size->width * map_size->height;

    // Fill the map with free cells
    std::fill(grid_map->data._buffer, grid_map->data._buffer + grid_map->data._length, static_cast<uint8_t>(or_Environment_FREE_CELL));

    // Fill the map with occupied cells
    for (uint i = 0; i < PlatesInfo->data(self)->seq._length; i++)
    {
        // Get the world coordinates of the plate
        double plate_x = PlatesInfo->data(self)->seq._buffer[i].coord.x;
        double plate_y = PlatesInfo->data(self)->seq._buffer[i].coord.y;

        // Convert world coordinates to grid coordinates
        int grid_x = (int)round((plate_x - grid_map->origin_x) / grid_map->resolution);
        int grid_y = (int)round((plate_y - grid_map->origin_y) / grid_map->resolution);

        // Inflate the grid cells
        grid_map->data._buffer[grid_y * grid_map->width + grid_x] = static_cast<uint8_t>(or_Environment_OCCUPIED_CELL);
    }

    // Publish the occupancy grid
    OccupancyGrid->write(self);

    return ColorTracker_poll;
}
