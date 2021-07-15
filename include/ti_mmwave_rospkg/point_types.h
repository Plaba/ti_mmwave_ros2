/** \file
 *
 *  Customized Point Cloud Library point structures for mmWave1843Boost data.
 *
 *  @author Claude, https://github.com/Claud1234/ti_mmwave_rospkg/
 */

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

/* Customized structure x, y, z, intensity, velocity, range*/
namespace radar_pcl {
    struct PointXYZIVR {
        PCL_ADD_POINT4D;
        float intensity;

        // Velocity in cartesian
        float velocity;

        // range in cartesian
        float range;

        // ensure proper alignment
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(radar_pcl::PointXYZIVR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, velocity, velocity)
                                  (float, range, range))

#endif //TI_MMWAVE_ROSPKG_SRC_TI_MMWAVE_ROSPKG_INCLUDE_POINT_TYPES_H_