//
// Created by abuchegger on 02.07.18.
//
#ifndef VESC_DRIVER_SERIAL_DATA_WRITER_H
#define VESC_DRIVER_SERIAL_DATA_WRITER_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace vesc_driver
{
class SerialDataWriter
{
public:
  typedef std::vector<uint8_t> Buffer;

  explicit SerialDataWriter(Buffer& buffer);
  SerialDataWriter(Buffer& buffer, Buffer::iterator position);

  void writeUnsignedInt8(uint8_t value);
  void writeUnsignedInt16(uint16_t value);
  void writeUnsignedInt32(uint32_t value);

  void writeInt8(int8_t value);
  void writeInt16(int16_t value);
  void writeInt32(int32_t value);

  void writeFloat16(double value);
  void writeFloat32(double value);

  void writeBlock(Buffer::const_iterator begin, Buffer::const_iterator end);

protected:
  Buffer& buffer_;
  Buffer::iterator position_;
};
}

#endif //VESC_DRIVER_SERIAL_DATA_WRITER_H
