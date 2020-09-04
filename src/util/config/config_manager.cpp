/**
 * @file config_manager.cpp
 * @author Marcel Flottmann
 * @date 2020-09-03
 */

#include "config_manager.h"
#include <boost/property_tree/json_parser.hpp>

namespace pt = boost::property_tree;
using namespace fastsense::util::config;

ConfigManager::ConfigManager() : config(std::string(), nullptr)
{
}

void ConfigManager::load(std::string filename)
{
    pt::ptree tree;
    pt::read_json(filename, tree);
    update(tree);
}

void ConfigManager::update(const pt::ptree& tree)
{
    if (config.canSet(tree))
    {
        config.set(tree);
    }
    else
    {
        throw std::runtime_error("Could not set configuration");
    }
}
