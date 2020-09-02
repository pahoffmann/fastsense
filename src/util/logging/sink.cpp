/**
 * @file sink.cpp
 * @author Marcel Flottmann
 * @date 2020-09-02
 */

#include "sink.h"

#include <iostream>

using namespace FastSense::util::logging::sink;

void CoutSink::write(const std::string& msg)
{
    std::cout << msg;
}

FileSink::FileSink(const std::string& filename) : file(filename, std::ios_base::app)
{
}

void FileSink::write(const std::string& msg)
{
    file << msg;
    file.flush();
}
