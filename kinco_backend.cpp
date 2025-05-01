// kinco_backend.cpp
#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <cstdint>
#include <cstring>
#include <chrono>

namespace py = pybind11;

static int fd = -1;

// 打开串口，返回 fd
int open_port(const std::string &device, int baud = B115200) {
  fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) throw std::runtime_error("open() failed");
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) throw std::runtime_error("tcgetattr");
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_iflag = 0;
  tty.c_cc[VTIME] = 0;    // non‐blocking read
  tty.c_cc[VMIN]  = 0;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) throw std::runtime_error("tcsetattr");
  return fd;
}

// 计算 Modbus CRC16
uint16_t crc16(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= buf[i];
    for (int j = 0; j < 8; ++j)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

// 读寄存器 0x3700，返回 signed32
int32_t read_pos() {
  if (fd < 0) throw std::runtime_error("port not opened");
  uint8_t req[8] = {0x01,0x03,0x37,0x00,0x00,0x02,0x00,0x00};
  uint16_t c = crc16(req,6);
  req[6] = c & 0xFF; req[7] = c >> 8;
  ::write(fd, req, 8);
  uint8_t resp[9];
  size_t idx = 0;
  auto start = std::chrono::steady_clock::now();
  while (idx < 9 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start
        ).count() < 2) {
    ssize_t n = ::read(fd, resp+idx, 9-idx);
    if (n > 0) idx += n;
  }
  if (idx!=9 || resp[1]!=0x03) throw std::runtime_error("read_pos fail");
  int32_t v = (resp[3]<<24)|(resp[4]<<16)|(resp[5]<<8)|resp[6];
  return v;
}

PYBIND11_MODULE(kinco_backend, m) {
  m.def("open_port", &open_port, "Open serial port");
  m.def("read_pos", &read_pos, "Read 32-bit position");
}
