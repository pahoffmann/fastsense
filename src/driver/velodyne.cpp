/**
 * @author Marcel Flottmann
 * @date   2020-08-11
 */

#include "velodyne.h"

#include <system_error>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/file.h>
#include <poll.h>

using namespace fastsense::driver;

constexpr uint8_t PROD_ID_VLP16 = 0x22;
constexpr uint8_t MODE_STRONGEST = 0x37;
constexpr uint8_t MODE_LAST = 0x38;
constexpr uint8_t MODE_DUAL = 0x39;
constexpr uint16_t BLOCK_FLAG = 0xEEFF;

constexpr float deg_to_rad(float deg)
{
    return deg * M_PIf32 / 180.f;
}

constexpr float LASER_ID_TO_VERT_ANGLE[16] = {
    deg_to_rad(-15),
    deg_to_rad(1),
    deg_to_rad(-13),
    deg_to_rad(3),
    deg_to_rad(-11),
    deg_to_rad(5),
    deg_to_rad(-9),
    deg_to_rad(7),
    deg_to_rad(-7),
    deg_to_rad(9),
    deg_to_rad(-5),
    deg_to_rad(11),
    deg_to_rad(-3),
    deg_to_rad(13),
    deg_to_rad(-1),
    deg_to_rad(15)
};

constexpr float LASER_ID_TO_OFFSET[16] = {
    11.2f / 1000.f,
    -0.7f / 1000.f,
    9.7f / 1000.f,
    -2.2f / 1000.f,
    8.1f / 1000.f,
    -3.7f / 1000.f,
    6.6f / 1000.f,
    -5.1f / 1000.f,
    5.1f / 1000.f,
    -6.6f / 1000.f,
    3.7f / 1000.f,
    2.2f / 1000.f,
    -9.7f / 1000.f,
    0.7f / 1000.f,
    -11.2f / 1000.f
};

constexpr uint8_t LASER_ID_TO_RING[16] = {
    15,
    13,
    11,
    9,
    7,
    5,
    3,
    1,
    14,
    12,
    10,
    8,
    6,
    4,
    2,
    0
};

velodyne::velodyne(std::string ipaddr, uint16_t port) :
    ipaddr(ipaddr),
    port(port),
    running(false),
    az_last(0.f),
    scan_buffer(16)
{
    // open socket
    sockfd = socket(PF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0)
    {
        throw std::system_error(errno, std::generic_category(),  "failed to open socket");
    }

    // set socket port
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    // bind to port
    if(bind(sockfd, (sockaddr*)&addr, sizeof(addr)) == -1)
    {
        close(sockfd);
        throw std::system_error(errno, std::generic_category(),  "failed to bind");
    }

    // set non blocking mode
    if(fcntl(sockfd, F_SETFL, O_NONBLOCK|FASYNC) < 0)
    {
        close(sockfd);
        throw std::system_error(errno, std::generic_category(),  "failed to set non-blocking mode");
    }
}

velodyne::~velodyne()
{
    stop();
    close(sockfd);
}

void velodyne::start()
{
    if(running == false)
    {
        running = true;
        az_last = 0.f;
        current_scan = std::make_shared<point_cloud>();
        scan_buffer.clear();
        worker = std::thread(&velodyne::receive_packet, this);
    }
}

void velodyne::stop()
{
    running = false;
    worker.join();
}

point_cloud::ptr velodyne::get_scan()
{
    point_cloud::ptr pc;
    scan_buffer.pop(&pc);
    return pc;
}

void velodyne::receive_packet()
{
    constexpr int POLL_TIMEOUT = 1000;
    struct pollfd fds[1];
    fds[0].fd = sockfd;
    fds[0].events = POLLIN;

    sockaddr_in sender;
    socklen_t sender_len = sizeof(sender);

    uint8_t buffer[PACKET_SIZE];

    while(running)
    {
        // wait for packet
        do
        {
            int ret = poll(fds, 1, POLL_TIMEOUT);
            if(ret < 0)
            {
                if(errno != EINTR)
                {
                    throw std::system_error(errno, std::generic_category(),  "poll failed");
                }
            }
            else if (ret == 0)
            {
                //std::cout << "Timeout!" << std::endl;
            }
            else
            {
                if (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL))
                {
                    throw std::runtime_error("poll error");
                }
            }
        } while (!(fds[0].revents & POLLIN));

        // receive packet
        ssize_t size = recvfrom(sockfd, buffer, PACKET_SIZE, 0, (sockaddr*)&sender, &sender_len);

        if(size < 0)
        {
            throw std::system_error(errno, std::generic_category(),  "recvfrom failed");
        }

        // process packet
        if(size == PACKET_SIZE)
        {
            decode_packet(buffer);
        }
    }
}

void velodyne::decode_packet(uint8_t packet[PACKET_SIZE])
{
    velodyne_packet* v_packet = reinterpret_cast<velodyne_packet*>(packet);

    if(v_packet->produkt_id != PROD_ID_VLP16)
    {
        //throw std::runtime_error("wrong sensor");
    }

    if(v_packet->mode != MODE_STRONGEST && v_packet->mode != MODE_LAST)
    {
        throw std::runtime_error("wrong mode");
    }

    for(int b = 0; b < BLOCKS_IN_PACKET; b++)
    {
        if(v_packet->blocks[b].flag == BLOCK_FLAG)
        {
            float az_block = v_packet->blocks[b].azimuth * 0.01f; // 0.01 degree

            // add new scan to queue when azimuth overflows
            if(az_block < az_last)
            {
                scan_buffer.push_nb(current_scan, true);
                current_scan = std::make_shared<point_cloud>();
            }
            az_last = az_block;

            // allocate points for current block
            size_t start_idx = current_scan->points.size();
            current_scan->points.resize(start_idx + POINTS_IN_BLOCK);

            // calculate azimuth difference between current and next block for interpolation
            // last block uses previous difference for simplification
            float az_diff;
            if(b + 1 < BLOCKS_IN_PACKET)
            {
                az_diff = (v_packet->blocks[b + 1].azimuth * 0.01f) - az_block;
                if(v_packet->blocks[b + 1].azimuth < v_packet->blocks[b].azimuth)
                {
                    az_diff += 360.f;
                }
            }

            for (int p = 0; p < POINTS_IN_BLOCK; p++)
            {
                // select point
                point& new_point = current_scan->points[LASER_ID_TO_RING[p%16] + (p < 16 ? 0 : 16)];

                // interpolate azimuth
                float az;
                if(p < 16)
                {
                    az = az_block + (az_diff * 2.304f * p) / 55.296f;
                }
                else
                {
                    az = az_block + (az_diff * 2.304f * (p-16)) / (2*55.296f);
                }

                // check angle overflow
                if(az > 360.f)
                {
                    az -= 360.f;
                }

                az = deg_to_rad(az);

                // calculate XYZ and fill new point
                float r = v_packet->blocks[b].points[p].distance * 0.002f - LASER_ID_TO_OFFSET[p%16]; // 0.002 m
                float cos_vertical = cos(LASER_ID_TO_VERT_ANGLE[p%16]);
                new_point.x = r * cos_vertical * sin(az);
                new_point.y = r * cos_vertical * cos(az);
                new_point.z = r * sin(LASER_ID_TO_VERT_ANGLE[p%16]);
                new_point.ring = LASER_ID_TO_RING[p%16];
                new_point.intensity = v_packet->blocks[b].points[p].intensity / 255.f;
            }
        }
    }
}
