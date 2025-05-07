// main.cpp
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cstdint>

// 计算 Modbus RTU CRC16
uint16_t calc_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

// 构造读寄存器命令（功能码 0x03）
std::vector<uint8_t> build_read(uint8_t slave, uint16_t reg, uint16_t cnt) {
    std::vector<uint8_t> p = {
        slave,
        0x03,
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>(cnt >> 8),
        static_cast<uint8_t>(cnt & 0xFF)
    };
    uint16_t crc = calc_crc16(p.data(), p.size());
    p.push_back(crc & 0xFF);
    p.push_back(crc >> 8);
    return p;
}

int main() {
    using namespace boost::asio;
    io_context ios;
    serial_port port(ios);

    const std::string port_name = "COM11";  // 根据你的串口改
    try {
        port.open(port_name);
        port.set_option(serial_port_base::baud_rate(115200));
        port.set_option(serial_port_base::character_size(8));
        port.set_option(serial_port_base::parity(serial_port_base::parity::none));
        port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        std::cout << "Opened " << port_name << " at 115200\n";
    } catch (std::exception& e) {
        std::cerr << "Error opening port: " << e.what() << "\n";
        return 1;
    }

    auto frame = build_read(1, 0x3700, 2);
    std::vector<uint8_t> resp(9);

    std::cout << "Polling 0x3700 every 1ms. Ctrl-C to quit.\n";
    while (true) {
        try {
            write(port, buffer(frame));
            read(port, buffer(resp));

            if (resp[0] == 1 && resp[1] == 3) {
                int32_t raw = (int32_t(resp[3]) << 24)
                            | (int32_t(resp[4]) << 16)
                            | (int32_t(resp[5]) <<  8)
                            | (int32_t(resp[6])      );
                uint32_t raw_u = static_cast<uint32_t>(raw);
                std::cout << "raw_signed=" << raw
                          << ", raw_unsigned=" << raw_u << "\r" << std::flush;
            } else {
                std::cout << "Bad resp:";
                for (auto b: resp) std::cout << " " << std::hex << int(b);
                std::cout << std::dec << "\n";
            }
        } catch (std::exception& e) {
            std::cerr << "\nI/O Error: " << e.what() << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
