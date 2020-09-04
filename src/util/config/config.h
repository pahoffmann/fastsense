/**
 * @file config.h
 * @author Marcel Flottmann
 * @date 2020-09-03
 */

#pragma once

#include "config_types.h"

namespace fastsense::util::config
{

struct InnerConfig : public ConfigContainer
{
    using ConfigContainer::ConfigContainer;

    DECLARE_CONFIG_ENTRY(size_t, test1);
    DECLARE_CONFIG_ENTRY(double, test2);
};

struct Config : public ConfigContainer
{
    using ConfigContainer::ConfigContainer;

    InnerConfig inner{"inner", this};

    DECLARE_CONFIG_ENTRY(float, blub);
    DECLARE_CONFIG_ENTRY(std::string, asdf);
};

}
