#pragma once

/**
 * @author Marc Eisoldt (meisoldt)
 */

#include <string>
#include <vector>

#include <iostream>
#include <fstream>
#include <map>
#include <memory>

#include <util/types.h>

namespace fastsense::util {

template<typename VEC_T>
class PCDFile
{
public:
    PCDFile(const std::string& file_name);

    void writePoints(const ScanPoints_t<VEC_T>& points, bool binary = false);

    void readPoints(std::vector<std::vector<VEC_T>>& points, unsigned int& number_of_points);

private:

    std::string name_;
};

} //namespace fastsense::util

#include "PCDFile.tcc"