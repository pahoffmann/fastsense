/**
 * @file config_types.h
 * @author Marcel Flottmann
 * @date 2020-09-03
 */

#pragma once

#include <unordered_map>
#include <functional>
#include <mutex>
#include <boost/property_tree/ptree.hpp>
#include <util/event_handler_list.h>

namespace fastsense::util::config
{

class ConfigGroup;

class ConfigEntryBase
{
public:
    ConfigEntryBase() = default;
    virtual ~ConfigEntryBase() = default;

    ConfigEntryBase(const ConfigEntryBase&) = delete;
    ConfigEntryBase(ConfigEntryBase&&) = delete;
    ConfigEntryBase& operator=(const ConfigEntryBase&) = delete;
    ConfigEntryBase& operator=(ConfigEntryBase&&) = delete;

    virtual bool isGroup() = 0;
    virtual void set(const boost::property_tree::ptree& val) = 0;
    virtual bool canSet(const boost::property_tree::ptree& val) = 0;
};

template<typename T>
class ConfigEntry : public ConfigEntryBase
{
    T value;
    std::mutex mtx;
    EventHandlerList<void(T)> handlerList;
public:
    ConfigEntry(std::string name, ConfigGroup* parent);

    T operator ()();

    bool isGroup() override;
    void set(const boost::property_tree::ptree& val) override;
    bool canSet(const boost::property_tree::ptree& val) override;

    typename EventHandlerList<void(T)>::handle_t addHandler(const std::function<void(T)>& handler);
    void removeHandler(const typename EventHandlerList<void(T)>::handle_t& handle);
};

class ConfigGroup : public ConfigEntryBase
{
    std::unordered_map<std::string, ConfigEntryBase*> entries;
    EventHandlerList<void()> handlerList;

protected:
    template<typename T>
    friend class ConfigEntry;

    void registerConfigEntry(std::string name, ConfigEntryBase* entry);

public:
    ConfigGroup(std::string name, ConfigGroup* parent);

    std::unordered_map<std::string, ConfigEntryBase*>::iterator begin();

    std::unordered_map<std::string, ConfigEntryBase*>::iterator end();

    ConfigEntryBase* operator[](const std::string& key);

    bool isGroup() override;
    void set(const boost::property_tree::ptree& val) override;
    bool canSet(const boost::property_tree::ptree& val) override;


    EventHandlerList<void()>::handle_t addHandler(const std::function<void()>& handler);
    void removeHandler(const EventHandlerList<void()>::handle_t& handle);
};

}

#define DECLARE_CONFIG_ENTRY(T, name) ConfigEntry<T> name{#name, this}
#define DECLARE_CONFIG_GROUP(T, name) T name{#name, this}

#include "config_types.tcc"
