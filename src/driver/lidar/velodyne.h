/**
 * @author Marcel Flottmann
 * @date   2020-08-11
 */

#pragma once

#include <point_cloud.h>
#include <memory>
#include <thread>
#include <concurrent_ring_buffer.h>

namespace fastsense
{
namespace driver
{

constexpr uint8_t POINTS_IN_BLOCK = 32;
constexpr uint8_t BLOCKS_IN_PACKET = 12;

#pragma pack(push, 1)
struct VelodyneDataPoint
{
    uint16_t distance;
    uint8_t intensity;
};

struct VelodyneBlock
{
    uint16_t flag;
    uint16_t azimuth;
    VelodyneDataPoint points[POINTS_IN_BLOCK];
};

struct VelodynePacket
{
    VelodyneBlock blocks[BLOCKS_IN_PACKET];
    uint32_t timestamp;
    uint8_t mode;
    uint8_t produkt_id;
};
#pragma pack(pop)

constexpr uint16_t PACKET_SIZE = sizeof(VelodynePacket);
static_assert(PACKET_SIZE == 1206);

class VelodyneDriver
{
public:
    VelodyneDriver(const std::string& ipaddr, uint16_t port, const std::shared_ptr<ConcurrentRingBuffer<PointCloud::ptr>>& buffer);
    virtual ~VelodyneDriver();
    void start();
    void stop();
    PointCloud::ptr getScan();

protected:
    void receivePacket();
    void decodePacket();

    std::string ipaddr;
    uint16_t port;
    int sockfd;

    std::thread worker;
    bool running;
    VelodynePacket packet;
    float azLast;

    ConcurrentRingBuffer<PointCloud::ptr>::ptr scanBuffer;
    PointCloud::ptr currentScan;
};

}
}
