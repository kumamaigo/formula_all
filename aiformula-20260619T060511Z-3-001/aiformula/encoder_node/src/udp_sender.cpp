#include "encoder_node/udp_sender.hpp"
#include <iostream>
#include <asio.hpp>

UdpSender::UdpSender(const std::string &ip, int port)
    : endpoint_(asio::ip::address::from_string(ip), port), socket_(io_context_, asio::ip::udp::v4()) {}

void UdpSender::send(const std::array<int, 2> &ints, const std::array<float, 8> &floats)
{
    struct Packet
    {
        int c1;
        float r1, o1, rps1, rpm1;
        int c2;
        float r2, o2, rps2, rpm2;
    } packet;

    packet.c1 = ints[0];
    packet.r1 = floats[0];
    packet.o1 = floats[1];
    packet.rps1 = floats[2];
    packet.rpm1 = floats[3];
    packet.c2 = ints[1];
    packet.r2 = floats[4];
    packet.o2 = floats[5];
    packet.rps2 = floats[6];
    packet.rpm2 = floats[7];

    try
    {
        socket_.send_to(asio::buffer(&packet, sizeof(packet)), endpoint_);
    }
    catch(std::exception &e)
    {
        std::cerr << "[UDP WARN] Send failed: " << e.what() << std::endl;
    }
}