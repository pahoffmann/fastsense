/**
 * @file config.cpp
 * @author Marcel Flottmann
 * @date 2020-09-11
 */

#include "catch2_config.h"

#include <util/config/config_manager.h>

using namespace fastsense::util::config;

struct TestConfigSubGroup : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_ENTRY(int, myInt);
    DECLARE_CONFIG_ENTRY(std::string, myString);
};

struct TestConfig : public ConfigGroup
{
    using ConfigGroup::ConfigGroup;

    DECLARE_CONFIG_GROUP(TestConfigSubGroup, subGroup);
    DECLARE_CONFIG_ENTRY(float, myFloat);
};

using TestConfigManager = ConfigManagerImpl<TestConfig>;

static const std::string TEST_STRING(
    "{"
    "    \"subGroup\":"
    "    {"
    "        \"myInt\": 4321,"
    "        \"myString\": \"Hello String!\""
    "    },"
    "    \"myFloat\": 1.25"
    "}"
);

static const std::string TEST_STRING_ZERO(
    "{"
    "    \"subGroup\":"
    "    {"
    "        \"myInt\": 0,"
    "        \"myString\": \"\""
    "    },"
    "    \"myFloat\": 0"
    "}"
);

static const std::string TEST_STRING_PARTIAL(
    "{"
    "    \"myFloat\": 1"
    "}"
);

static const std::string TEST_STRING_WRONG_TYPE(
    "{"
    "    \"myFloat\": \"Test\""
    "}"
);

static const std::string TEST_STRING_WRONG_NAME(
    "{"
    "    \"myFloats\": 0"
    "}"
);

TEST_CASE("load configuration", "[ConfigManager]")
{
    SECTION("loading string")
    {
        TestConfigManager::loadString(TEST_STRING);
        REQUIRE(TestConfigManager::config().subGroup.myInt() == 4321);
        REQUIRE(TestConfigManager::config().subGroup.myString() == "Hello String!");
        REQUIRE(TestConfigManager::config().myFloat() == 1.25f);
    }

    SECTION("load file")
    {
        TestConfigManager::loadFile("config.json");
        REQUIRE(TestConfigManager::config().subGroup.myInt() == 1234);
        REQUIRE(TestConfigManager::config().subGroup.myString() == "Hello File!");
        REQUIRE(TestConfigManager::config().myFloat() == 2.5f);
    }

    SECTION("load partial")
    {
        TestConfigManager::loadString(TEST_STRING);
        TestConfigManager::loadString(TEST_STRING_PARTIAL);
        REQUIRE(TestConfigManager::config().subGroup.myInt() == 4321);
        REQUIRE(TestConfigManager::config().subGroup.myString() == "Hello String!");
        REQUIRE(TestConfigManager::config().myFloat() == 1.f);
    }

    SECTION("load wrong data throws")
    {
        REQUIRE_THROWS(TestConfigManager::loadString(TEST_STRING_WRONG_TYPE));
        REQUIRE_THROWS(TestConfigManager::loadString(TEST_STRING_WRONG_NAME));
    }
}

TEST_CASE("export configuration", "[ConfigManager]")
{
    SECTION("create string")
    {
        TestConfigManager::loadString(TEST_STRING);
        std::string str = TestConfigManager::createString();
        TestConfigManager::loadString(TEST_STRING_ZERO);
        TestConfigManager::loadString(str);
        REQUIRE(TestConfigManager::config().subGroup.myInt() == 4321);
        REQUIRE(TestConfigManager::config().subGroup.myString() == "Hello String!");
        REQUIRE(TestConfigManager::config().myFloat() == 1.25f);
    }

    SECTION("create string")
    {
        TestConfigManager::loadString(TEST_STRING);
        TestConfigManager::writeFile("test.json");
        TestConfigManager::loadString(TEST_STRING_ZERO);
        TestConfigManager::loadFile("test.json");
        REQUIRE(TestConfigManager::config().subGroup.myInt() == 4321);
        REQUIRE(TestConfigManager::config().subGroup.myString() == "Hello String!");
        REQUIRE(TestConfigManager::config().myFloat() == 1.25f);
    }
}


TEST_CASE("configuration event handler", "[ConfigManager]")
{
    bool handlerCalled = false;
    auto handle = TestConfigManager::config().myFloat.addHandler([&](float /*unused*/)
    {
        handlerCalled = true;
    });
    TestConfigManager::loadString(TEST_STRING_PARTIAL);
    REQUIRE(handlerCalled);

    handlerCalled = false;
    TestConfigManager::config().myFloat.removeHandler(std::move(handle));
    TestConfigManager::loadString(TEST_STRING_PARTIAL);
    REQUIRE_FALSE(handlerCalled);
}