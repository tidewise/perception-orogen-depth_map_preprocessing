#ifndef depth_map_preprocessing_TYPES_HPP
#define depth_map_preprocessing_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace depth_map_preprocessing
{

/**
 * :NoCompensation
 *    No motion compensation will be applyed.
 * :HorizontalInterpolation and :VerticalInterpolation
 *    Will interpolate the local movement by the first and the last timestamp
 *    and will apply the transformations either horizontal (column-wise) or
 *    vertical (row-wise) in the frame of the newest scan.
 *    Note that |depth_map.timestamps| >= 2.
 * :Horizontal and :Vertical
 *    Will get one transformation for each timestamp in depth_map.timestamps
 *    and will apply them in the frame of the newest scan.
 *    Note that |depth_map.timestamps| == |depth_map.horizontal_size| for the
 *    horizontal case and |depth_map.timestamps| == |depth_map.vertical_size|
 *    for the vertical case.
 */
enum MotionCompensation
{
    NoCompensation = 0,
    HorizontalInterpolation,
    Horizontal,
    VerticalInterpolation,
    Vertical
};

}

#endif

