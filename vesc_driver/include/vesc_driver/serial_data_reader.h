//
// Created by abuchegger on 02.07.18.
//
#ifndef VESC_DRIVER_SERIAL_DATA_READER_H
#define VESC_DRIVER_SERIAL_DATA_READER_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace vesc_driver
{
class SerialDataReader
{
public:
  typedef std::vector<uint8_t> Buffer;

  SerialDataReader(Buffer::const_iterator begin, Buffer::const_iterator end);

  bool canRead() const;
  void skipBytes(ptrdiff_t n);

  uint8_t readUnsignedInt8();
  uint16_t readUnsignedInt16();
  uint32_t readUnsignedInt32();

  int8_t readInt8();
  int16_t readInt16();
  int32_t readInt32();

  double readFloat16();
  double readFloat32();

protected:
  Buffer::const_iterator begin_;
  Buffer::const_iterator end_;
};
}

#endif //VESC_DRIVER_SERIAL_DATA_READER_H
