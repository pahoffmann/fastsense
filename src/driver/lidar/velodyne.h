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
struct velodyne_data_point
{
    uint16_t distance;
    uint8_t intensity;
};

struct velodyne_block
{
    uint16_t flag;
    uint16_t azimuth;
    velodyne_data_point points[POINTS_IN_BLOCK];
};

struct velodyne_packet
{
    velodyne_block blocks[BLOCKS_IN_PACKET];
    uint32_t timestamp;
    uint8_t mode;
    uint8_t produkt_id;
};
#pragma pack(pop)

constexpr uint16_t PACKET_SIZE = sizeof(velodyne_packet);
static_assert(PACKET_SIZE == 1206);

class velodyne
{
public:
    velodyne(const std::string& ipaddr, uint16_t port, const std::shared_ptr<concurrent_ring_buffer<point_cloud::ptr>>& buffer);
    virtual ~velodyne();
    void start();
    void stop();
    point_cloud::ptr get_scan();

protected:
    void receive_packet();
    void decode_packet(uint8_t packet[PACKET_SIZE]);
    
    std::string ipaddr;
    uint16_t port;
    int sockfd;

    std::thread worker;
    bool running;
    float az_last;

    concurrent_ring_buffer<point_cloud::ptr>::ptr scan_buffer;
    point_cloud::ptr current_scan;
};

}
}