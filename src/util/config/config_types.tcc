/**
 * @file config_types.tcc
 * @author Marcel Flottmann
 * @date 2020-09-03
 */

#pragma once

#include <util/logging/logger.h>

namespace fastsense::util::config
{

template<typename T>
ConfigEntry<T>::ConfigEntry(std::string name, ConfigGroup* parent) : value(T())
{
    if (parent)
    {
        parent->registerConfigEntry(name, this);
    }
}

template<typename T>
T ConfigEntry<T>::operator ()()
{
    std::lock_guard lock(mtx);
    T copy(value);
    return copy;
}

template<typename T>
bool ConfigEntry<T>::isGroup()
{
    return false;
}

template<typename T>
void ConfigEntry<T>::set(const boost::property_tree::ptree& val)
{
    {
        std::lock_guard lock(mtx);
        value = val.get_value<T>();
    }
    handlerList.invoke((*this)());
}

template<typename T>
bool ConfigEntry<T>::canSet(const boost::property_tree::ptree& val)
{
    return val.get_value_optional<T>() != boost::none;
}

template<typename T>
typename EventHandlerList<void(T)>::handle_t ConfigEntry<T>::addHandler(const std::function<void(T)>& handler)
{
    return handlerList.add(handler);
}

template<typename T>
void ConfigEntry<T>::removeHandler(const typename EventHandlerList<void(T)>::handle_t& handle)
{
    handlerList.remove(handle);
}

inline void ConfigGroup::registerConfigEntry(std::string name, ConfigEntryBase* entry)
{
    entries.emplace(name, entry);
}

inline ConfigGroup::ConfigGroup(std::string name, ConfigGroup* parent)
{
    if (parent)
    {
        parent->registerConfigEntry(name, this);
    }
}

inline std::unordered_map<std::string, ConfigEntryBase*>::iterator ConfigGroup::begin()
{
    return entries.begin();
}

inline std::unordered_map<std::string, ConfigEntryBase*>::iterator ConfigGroup::end()
{
    return entries.end();
}

inline ConfigEntryBase* ConfigGroup::operator[](const std::string& key)
{
    return entries.at(key);
}

inline bool ConfigGroup::isGroup()
{
    return true;
}

inline void ConfigGroup::set(const boost::property_tree::ptree& val)
{
    for (auto& item : val)
    {
        ConfigEntryBase* entry = entries.at(item.first);
        entry->set(item.second);
    }
    handlerList.invoke();
}

inline bool ConfigGroup::canSet(const boost::property_tree::ptree& val)
{
    for (auto& item : val)
    {
        auto entry_it = entries.find(item.first);
        if (entry_it == entries.end())
        {
            fastsense::util::logging::Logger::error("Config entry \"", item.first, "\" does not exist!");
            return false;
        }
        ConfigEntryBase* entry = entry_it->second;
        if (!entry->canSet(item.second))
        {
            if (entry->isGroup())
            {
                fastsense::util::logging::Logger::error("Cannot set config entry in \"", item.first, "\"!");
            }
            else
            {
                fastsense::util::logging::Logger::error("Cannot set config entry \"", item.first, "\" with value \"", item.second.data(), "\"!");
            }
            return false;
        }
    }
    return true;
}

inline EventHandlerList<void()>::handle_t ConfigGroup::addHandler(const std::function<void()>& handler)
{
    return handlerList.add(handler);
}

inline void ConfigGroup::removeHandler(const EventHandlerList<void()>::handle_t& handle)
{
    handlerList.remove(handle);
}

}
