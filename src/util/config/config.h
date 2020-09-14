/**
 * @file config.h
 * @author Marcel Flottmann
 * @date 2020-09-03
 */

#pragma once

#include "config_types.h"

namespace fastsense::util::config
{

struct InnerConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(size_t, test1);
    DECLARE_CONFIG_ENTRY(double, test2);
};

struct Config : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_GROUP(InnerConfig, inner);

    DECLARE_CONFIG_ENTRY(float, blub);
    DECLARE_CONFIG_ENTRY(std::string, asdf);
};

}
