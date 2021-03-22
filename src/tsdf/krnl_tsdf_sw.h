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
                  TSDFEntryHW* mapData0,
                  TSDFEntryHW* mapData1,
                  TSDFEntryHW* mapData2,
                  TSDFEntryHW* mapData3,
                  int sizeX,   int sizeY,   int sizeZ,
                  int posX,    int posY,    int posZ,
                  int offsetX, int offsetY, int offsetZ,
                  TSDFEntryHW* new_entries0,
                  TSDFEntryHW* new_entries1,
                  TSDFEntryHW* new_entries2,
                  TSDFEntryHW* new_entries3,
                  TSDFEntryHW::ValueType tau,
                  TSDFEntryHW::WeightType max_weight,
                  int up_x, int up_y, int up_z);

} // namespace fastsense::tsdf