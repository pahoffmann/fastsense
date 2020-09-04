/**
 * @file config_manager.h
 * @author Marcel Flottmann
 * @date 2020-09-03
 */

#pragma once

#include "config.h"
#include <list>
#include <boost/property_tree/ptree.hpp>

namespace fastsense::util::config
{

class ConfigManager
{
private:

    Config config;

public:
    ConfigManager();
    void load(std::string filename);
    void update(const boost::property_tree::ptree& tree);

    Config& get()
    {
        return config;
    }
};

}
