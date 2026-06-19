#pragma once
#include <array>
#include <string>
#include <asio.hpp>

class UdpSender
{
public:
    UdpSender(const std::string &ip, int port);
    void send(const std::array<int, 2> &ints, const std::array<float, 8> &floats);

private:
    asio::io_context io_context_;
    asio::ip::udp::endpoint endpoint_;
    asio::ip::udp::socket socket_;
};