/**
 * @file communication.cpp
 * @author Marcel Flottmann
 * @date 2020-10-6
 */

#include "catch2_config.h"
#include <comm/receiver.h>
#include <comm/sender.h>
#include <comm/buffered_receiver.h>
#include <util/concurrent_ring_buffer.h>
#include <util/point.h>
#include <util/time.h>
#include <util/runner.h>
#include <msg/imu.h>
#include <msg/transform.h>
#include <msg/point_cloud.h>
#include <msg/tsdf_bridge_msg.h>
#include <iostream>
#include <thread>

using namespace fastsense;
using namespace fastsense::comm;
using namespace fastsense::msg;

constexpr size_t iterations = 2;

TEST_CASE("Simple Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'Simple Sender Receiver Test'" << std::endl;
    for (size_t i = 0; i < iterations; ++i)
    {
        int value_received;
        int value_to_send = 42;

        std::thread receive_thread{[&]()
        {
            Receiver<int> receiver{"127.0.0.1", 1234};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            receiver.receive(value_received);
        }};

        std::thread send_thread{[&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            Sender<int> sender{1234};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            sender.send(value_to_send);
        }};

        receive_thread.join();
        send_thread.join();
        REQUIRE(value_to_send == value_received);
    }
}

TEST_CASE("PointCloud Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'PointCloud Sender Receiver Test'" << std::endl;
    for (size_t i = 0; i < iterations; ++i)
    {
        PointCloud pc_to_send;
        pc_to_send.rings_ = 2;
        pc_to_send.points_.push_back({1, 2, 3});
        pc_to_send.points_.push_back({2, 3, 4});
        pc_to_send.points_.push_back({3, 4, 5});
        PointCloud pc_received;

        std::thread receive_thread{[&]()
        {
            Receiver<PointCloud> receiver{"127.0.0.1", 1235};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            receiver.receive(pc_received);
        }};

        std::thread send_thread{[&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            Sender<PointCloud> sender{1235};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            sender.send(pc_to_send);
        }};

        receive_thread.join();
        send_thread.join();
        REQUIRE(pc_to_send.rings_ == pc_received.rings_);
        REQUIRE(pc_to_send.points_ == pc_received.points_);
    }
}

TEST_CASE("PointCloudStamped Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'PointCloudStamped Sender Receiver Test'" << std::endl;
    for (size_t i = 0; i < iterations; ++i)
    {
        PointCloud pc_to_send;
        pc_to_send.rings_ = 2;
        pc_to_send.points_.push_back({1, 2, 3});
        pc_to_send.points_.push_back({2, 3, 4});
        pc_to_send.points_.push_back({3, 4, 5});

        auto tp_to_send = util::HighResTime::now();

        PointCloudStamped pcl_stamped{std::move(pc_to_send), tp_to_send};
        PointCloudStamped pcl_stamped_received;

        std::thread receive_thread{[&]()
        {
            Receiver<PointCloudStamped> receiver{"127.0.0.1", 1236};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            receiver.receive(pcl_stamped_received);
        }};

        std::thread send_thread{[&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            Sender<PointCloudStamped> sender{1236};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            sender.send(pcl_stamped);
        }};

        receive_thread.join();
        send_thread.join();

        REQUIRE(pcl_stamped.data_.rings_ == pcl_stamped_received.data_.rings_);
        REQUIRE(pcl_stamped.data_.points_ == pcl_stamped_received.data_.points_);
        REQUIRE(pcl_stamped.timestamp_ == pcl_stamped_received.timestamp_);
    }
}

TEST_CASE("Stamped<PointCloud> Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'Stamped<PointCloud> Sender Receiver Test'" << std::endl;
    for (size_t i = 0; i < iterations; ++i)
    {
        PointCloud pc_to_send;
        pc_to_send.rings_ = 2;
        pc_to_send.points_.push_back({1, 2, 3});
        pc_to_send.points_.push_back({2, 3, 4});
        pc_to_send.points_.push_back({3, 4, 5});

        auto tp_to_send = util::HighResTime::now();

        Stamped<PointCloud> pcl_stamped{std::move(pc_to_send), tp_to_send};
        Stamped<PointCloud> pcl_stamped_received;

        std::thread receive_thread{[&]()
        {
            Receiver<Stamped<PointCloud>> receiver{"127.0.0.1", 1237};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            receiver.receive(pcl_stamped_received);
        }};

        std::thread send_thread{[&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            Sender<Stamped<PointCloud>> sender{1237};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            sender.send(pcl_stamped);
        }};

        receive_thread.join();
        send_thread.join();

        REQUIRE(pcl_stamped.data_.rings_ == pcl_stamped_received.data_.rings_);
        REQUIRE(pcl_stamped.data_.points_ == pcl_stamped_received.data_.points_);
        REQUIRE(pcl_stamped.timestamp_ == pcl_stamped_received.timestamp_);
    }
}

TEST_CASE("TSDFBridgeMessage Sender Receiver Test", "[communication]")
{   
    std::cout << "Testing 'TSDFBridgeMessage Sender Receiver Test'" << std::endl;

    for (size_t i = 0; i < iterations; ++i)
    {
        TSDFBridgeMessage tsdf_msg;
        tsdf_msg.tau_ = 2;
        tsdf_msg.size_ = {10, 10, 10};
        tsdf_msg.pos_ = {0, 0, 0};
        tsdf_msg.offset_ = {0, 0, 0};
        tsdf_msg.tsdf_data_.resize(10 * 10 * 10);
        for (size_t i = 0; i < 10 * 10 * 10; ++i)
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

        TSDFBridgeMessage tsdf_received;

        std::thread receive_thread{[&]()
        {
            Receiver<TSDFBridgeMessage> receiver{"127.0.0.1", 1238};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            receiver.receive(tsdf_received);
        }};

        std::thread send_thread{[&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            Sender<TSDFBridgeMessage> sender{1238};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            sender.send(tsdf_msg);
        }};

        receive_thread.join();
        send_thread.join();
        REQUIRE(tsdf_msg.tau_ == tsdf_received.tau_);
        REQUIRE(tsdf_msg.size_ == tsdf_received.size_);
        REQUIRE(tsdf_msg.pos_ == tsdf_received.pos_);
        REQUIRE(tsdf_msg.offset_ == tsdf_received.offset_);
        REQUIRE(tsdf_msg.tsdf_data_ == tsdf_received.tsdf_data_);
    }
}

TEST_CASE("ImuStamped Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'ImuStamped Sender Receiver Test'" << std::endl;
    auto tp = util::HighResTimePoint{std::chrono::nanoseconds{1000}};

    LinearAcceleration acc{1, 2, 3};
    AngularVelocity ang{4, 5, 6};
    MagneticField mag{7, 8, 9};
    Imu imu{acc, ang, mag};

    ImuStamped imu_stamped{imu, tp};
    ImuStamped value_received{};

    for (size_t i = 0; i < iterations; ++i)
    {
        auto tp = util::HighResTimePoint{std::chrono::nanoseconds{1000}};

        LinearAcceleration acc{1, 2, 3};
        AngularVelocity ang{4, 5, 6};
        MagneticField mag{7, 8, 9};
        Imu imu{acc, ang, mag};

        ImuStamped imu_stamped{imu, tp};
        ImuStamped value_received{};

        std::thread receive_thread{[&]()
        {
            Receiver<ImuStamped> receiver{"127.0.0.1", 1239};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            value_received = receiver.receive();
        }};

        std::thread send_thread{[&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            Sender<ImuStamped> sender{1239};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            sender.send(imu_stamped);
        }};

        receive_thread.join();
        send_thread.join();

        auto& [ imu_received, tp_received ] = value_received;

        REQUIRE(imu_received.acc.x() == 1);
        REQUIRE(imu_received.acc.y() == 2);
        REQUIRE(imu_received.acc.z() == 3);
        REQUIRE(imu_received.ang.x() == 4);
        REQUIRE(imu_received.ang.y() == 5);
        REQUIRE(imu_received.ang.z() == 6);
        REQUIRE(imu_received.mag.x() == 7);
        REQUIRE(imu_received.mag.y() == 8);
        REQUIRE(imu_received.mag.z() == 9);
        REQUIRE(tp_received == tp);
    }
}

TEST_CASE("BufferedImuStampedReceiver Test", "[communication]")
{
    std::cout << "Testing 'BufferedImuStampedReceiver Test'" << std::endl;

    auto tp = util::HighResTimePoint{std::chrono::nanoseconds{1000}};

    LinearAcceleration acc{1, 2, 3};
    AngularVelocity ang{4, 5, 6};
    MagneticField mag{7, 8, 9};
    Imu imu{acc, ang, mag};

    ImuStamped imu_stamped{imu, tp};
    ImuStamped value_received{};

    auto buffer = std::make_shared<msg::ImuStampedBuffer>(5);

    std::thread receive_thread{[&]()
    {
        BufferedImuStampedReceiver receiver{"127.0.0.1", 1244, buffer};
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        receiver.receive();
    }};

    std::thread send_thread{[&]()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        Sender<ImuStamped> sender{1244};
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        sender.send(imu_stamped);
    }};

    receive_thread.join();
    send_thread.join();

    buffer->pop(&value_received);
    auto& [ imu_received, tp_received ] = value_received;

    REQUIRE(imu_received.acc.x() == 1);
    REQUIRE(imu_received.acc.y() == 2);
    REQUIRE(imu_received.acc.z() == 3);
    REQUIRE(imu_received.ang.x() == 4);
    REQUIRE(imu_received.ang.y() == 5);
    REQUIRE(imu_received.ang.z() == 6);
    REQUIRE(imu_received.mag.x() == 7);
    REQUIRE(imu_received.mag.y() == 8);
    REQUIRE(imu_received.mag.z() == 9);
    REQUIRE(tp_received == tp);
}

TEST_CASE("BufferedPclStampedReceiver Test", "[communication]")
{
    std::cout << "Testing 'BufferedPclStampedReceiver Test'" << std::endl;

    PointCloud pcl;
    pcl.rings_ = 2;
    pcl.points_.push_back({1, 2, 3});
    pcl.points_.push_back({2, 3, 4});
    pcl.points_.push_back({3, 4, 5});

    auto tp_to_send = util::HighResTime::now();

    PointCloudStamped pcl_stamped_to_send{std::move(pcl), tp_to_send};

    auto buffer = std::make_shared<msg::PointCloudPtrStampedBuffer>(5);

    PointCloudPtrStamped data_recv;

    std::thread receive_thread{[&]()
    {
        BufferedPclStampedReceiver receiver{"127.0.0.1", 1254, buffer};
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        receiver.receive();
    }};

    std::thread send_thread{[&]()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        Sender<PointCloudStamped> sender{1254};
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        sender.send(pcl_stamped_to_send);
    }};

    receive_thread.join();
    send_thread.join();

    buffer->pop(&data_recv);
    auto& [ pcl_ptr_stamped_recv, tp_received ] = data_recv;

    REQUIRE(pcl_stamped_to_send.data_.rings_ == pcl_ptr_stamped_recv->rings_);
    REQUIRE(pcl_stamped_to_send.data_.points_ == pcl_ptr_stamped_recv->points_);
    REQUIRE(pcl_stamped_to_send.timestamp_ == tp_received);
}

TEST_CASE("Stamped<int> Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'Stamped<int> Sender Receiver Test'" << std::endl;
    for (size_t i = 0; i < iterations; ++i)
    {
        auto tp = util::HighResTime::now();
        msg::Stamped<int> value_received;
        msg::Stamped<int> value_to_send(42, tp);

        std::thread receive_thread{[&]()
        {
            Receiver<msg::Stamped<int>> receiver{"127.0.0.1", 1264};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            receiver.receive(value_received);
        }};

        std::thread send_thread{[&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            Sender<msg::Stamped<int>> sender{1264};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            sender.send(value_to_send);
        }};

        receive_thread.join();
        send_thread.join();

        REQUIRE(value_to_send.data_ == value_received.data_);
        REQUIRE(value_to_send.timestamp_ == value_received.timestamp_);
    }
}

TEST_CASE("Stamped<msg::Transform> Sender Receiver Test", "[communication]")
{
    std::cout << "Testing 'Stamped<msg::Transform> Sender Receiver Test'" << std::endl;
    for (size_t i = 0; i < iterations; ++i)
    {
        auto tp = util::HighResTime::now();
        msg::Stamped<msg::Transform> value_received;
        msg::Stamped<msg::Transform> value_to_send(std::move(msg::Transform{Quaternionf{1, 2, 3, 4}, Vector3f{5, 6, 7}}), tp);

        std::thread receive_thread{[&]()
        {
            Receiver<msg::Stamped<msg::Transform>> receiver{"127.0.0.1", 1274};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            receiver.receive(value_received);
        }};

        std::thread send_thread{[&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            Sender<msg::Stamped<msg::Transform>> sender{1274};
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            sender.send(value_to_send);
        }};

        receive_thread.join();
        send_thread.join();
        
        REQUIRE(value_to_send.data_.rotation.x() == Approx(value_received.data_.rotation.x()).margin(0.00001));
        REQUIRE(value_to_send.data_.rotation.y() == Approx(value_received.data_.rotation.y()).margin(0.00001));
        REQUIRE(value_to_send.data_.rotation.z() == Approx(value_received.data_.rotation.z()).margin(0.00001));
        REQUIRE(value_to_send.data_.rotation.w() == Approx(value_received.data_.rotation.w()).margin(0.00001));
        REQUIRE(value_to_send.data_.translation == value_received.data_.translation);
        REQUIRE(value_to_send.timestamp_ == value_received.timestamp_);
    }
}