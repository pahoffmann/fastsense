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

class ConfigContainer;

class ConfigEntryBase
{
public:
    ConfigEntryBase() = default;
    virtual ~ConfigEntryBase() = default;

    ConfigEntryBase(const ConfigEntryBase&) = delete;
    ConfigEntryBase(ConfigEntryBase&&) = delete;
    ConfigEntryBase& operator=(const ConfigEntryBase&) = delete;
    ConfigEntryBase& operator=(ConfigEntryBase&&) = delete;

    virtual bool isContainer() = 0;
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
    ConfigEntry(std::string name, ConfigContainer* parent);

    T operator ()();

    bool isContainer() override;
    void set(const boost::property_tree::ptree& val) override;
    bool canSet(const boost::property_tree::ptree& val) override;

    typename EventHandlerList<void(T)>::handle_t addHandler(const std::function<void(T)>& handler);
    void removeHandler(const typename EventHandlerList<void(T)>::handle_t& handle);
};

#define DECLARE_CONFIG_ENTRY(T, name) ConfigEntry<T> name{#name, this}

class ConfigContainer : public ConfigEntryBase
{
    std::unordered_map<std::string, ConfigEntryBase*> entries;
    EventHandlerList<void()> handlerList;

protected:
    template<typename T>
    friend class ConfigEntry;

    void registerConfigEntry(std::string name, ConfigEntryBase* entry);

public:
    ConfigContainer(std::string name, ConfigContainer* parent);

    std::unordered_map<std::string, ConfigEntryBase*>::iterator begin();

    std::unordered_map<std::string, ConfigEntryBase*>::iterator end();

    ConfigEntryBase* operator[](const std::string& key);

    bool isContainer() override;
    void set(const boost::property_tree::ptree& val) override;
    bool canSet(const boost::property_tree::ptree& val) override;


    EventHandlerList<void()>::handle_t addHandler(const std::function<void()>& handler);
    void removeHandler(const EventHandlerList<void()>::handle_t& handle);
};

}

#include "config_types.tcc"
