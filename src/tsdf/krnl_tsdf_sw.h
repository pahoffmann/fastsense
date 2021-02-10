#pragma once

/**
 * @author Marc Eisoldt
 */

#include <map/local_map_hw.h>
#include <util/point_hw.h>
#include <util/tsdf_hw.h>
#include <hw/kernels/tsdf_kernel.h>

#include <utility>


namespace fastsense::tsdf
{

/**
 * @brief Software equivalent of the kernel TSDF function
 */
void krnl_tsdf_sw(PointHW* scanPoints0,
                  PointHW* scanPoints1,
                  PointHW* scanPoints2,
                  PointHW* scanPoints3,
                  int numPoints,
                  TSDFValueHW* mapData0,
                  TSDFValueHW* mapData1,
                  TSDFValueHW* mapData2,
                  TSDFValueHW* mapData3,
                  int sizeX,   int sizeY,   int sizeZ,
                  int posX,    int posY,    int posZ,
                  int offsetX, int offsetY, int offsetZ,
                  TSDFValueHW* new_entries0,
                  TSDFValueHW* new_entries1,
                  TSDFValueHW* new_entries2,
                  TSDFValueHW* new_entries3,
                  TSDFValueHW::ValueType tau,
                  TSDFValueHW::WeightType max_weight,
                  int up_x, int up_y, int up_z);

} // namespace fastsense::tsdf