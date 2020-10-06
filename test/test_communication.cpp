/**
 * @file communication.cpp
 * @author Marcel Flottmann
 * @date 2020-10-6
 */

#include "catch2_config.h"
#include <comm/receiver.h>
#include <comm/sender.h>
#include <msg/point_cloud.h>
#include <msg/tsdf_bridge_msg.h>
#include <iostream>

using namespace fastsense::comm;
using namespace fastsense::msg;

TEST_CASE("Simple Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'Simple Sender Receiver Test'" << std::endl;
    Receiver<int> receiver{1234};
    Sender<int> sender{"127.0.0.1", 1234};

    int value_to_send = 42;
    sender.send(&value_to_send);
    int value_received = receiver.receive();

    REQUIRE(value_to_send == value_received);
}

TEST_CASE("PointCloud Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'PointCloud Sender Receiver Test'" << std::endl;
    Receiver<PointCloud> receiver{1234};
    Sender<PointCloud> sender{"127.0.0.1", 1234};

    PointCloud pc_to_send;
    pc_to_send.rings_ = 2;
    pc_to_send.points_.push_back({1, 2, 3});
    pc_to_send.points_.push_back({2, 3, 4});
    pc_to_send.points_.push_back({3, 4, 5});
    sender.send(pc_to_send);

    PointCloud pc_received;
    receiver.receive(pc_received);

    REQUIRE(pc_to_send.rings_ == pc_received.rings_);
    REQUIRE(pc_to_send.points_ == pc_received.points_);
}

TEST_CASE("TSDFBridgeMessage Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'ZMQConverter Sender Receiver Test'" << std::endl;
    Receiver<TSDFBridgeMessage> receiver{1234};
    Sender<TSDFBridgeMessage> sender{"127.0.0.1", 1234};

    TSDFBridgeMessage tsdf_msg;
    tsdf_msg.tau_ = 2;
    tsdf_msg.map_resolution_ = 1;
    tsdf_msg.size_ = {10, 10, 10};
    tsdf_msg.pos_ = {0, 0, 0};
    tsdf_msg.offset_ = {0, 0, 0};
    tsdf_msg.tsdf_data_.resize(10*10*10);
    for (size_t i = 0; i < 10*10*10; ++i)
    {
       tsdf_msg.tsdf_data_[i].first = 0;
       tsdf_msg.tsdf_data_[i].second = 0;
    }
    
    tsdf_msg.tsdf_data_[4 + 5 * 10 + 5 * 10 * 10].first = 1;
    tsdf_msg.tsdf_data_[4 + 5 * 10 + 5 * 10 * 10].second = 1;

    tsdf_msg.tsdf_data_[3 + 5 * 10 + 5 * 10 * 10].first = 2;
    tsdf_msg.tsdf_data_[3 + 5 * 10 + 5 * 10 * 10].second = 1;

    tsdf_msg.tsdf_data_[6 + 5 * 10 + 5 * 10 * 10].first = -1;
    tsdf_msg.tsdf_data_[6 + 5 * 10 + 5 * 10 * 10].second = 1;

    tsdf_msg.tsdf_data_[7 + 5 * 10 + 5 * 10 * 10].first = -2;
    tsdf_msg.tsdf_data_[7 + 5 * 10 + 5 * 10 * 10].second = 1;
    sender.send(tsdf_msg);

    TSDFBridgeMessage tsdf_received;
    receiver.receive(tsdf_received);

    REQUIRE(tsdf_msg.tau_ == tsdf_received.tau_);
    REQUIRE(tsdf_msg.map_resolution_ == tsdf_received.map_resolution_);
    REQUIRE(tsdf_msg.size_ == tsdf_received.size_);
    REQUIRE(tsdf_msg.pos_ == tsdf_received.pos_);
    REQUIRE(tsdf_msg.offset_ == tsdf_received.offset_);
    REQUIRE(tsdf_msg.tsdf_data_ == tsdf_received.tsdf_data_);
}
