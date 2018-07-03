//
// Created by abuchegger on 02.07.18.
//
#include <vesc_driver/serial_data_reader.h>
#include <algorithm>
#include <stdexcept>

namespace vesc_driver
{
SerialDataReader::SerialDataReader(Buffer::const_iterator begin, Buffer::const_iterator end)
  : begin_(begin), end_(end)
{
}

bool SerialDataReader::canRead() const
{
  return begin_ < end_;
}

void SerialDataReader::skipBytes(ptrdiff_t n)
{
  begin_ += std::min(n, end_ - begin_);
}

uint8_t SerialDataReader::readUnsignedInt8()
{
  if (begin_ < end_)
  {
    return *begin_++;
  }
  throw std::out_of_range("tried to read beyond end of buffer");
}

uint16_t SerialDataReader::readUnsignedInt16()
{
  const uint16_t result
    = (static_cast<uint16_t>(readUnsignedInt8()) << 8) +
      (static_cast<uint16_t>(readUnsignedInt8()) << 0);
  return result;
}

uint32_t SerialDataReader::readUnsignedInt32()
{
  const uint32_t result
    = (static_cast<uint32_t>(readUnsignedInt8()) << 24) +
      (static_cast<uint32_t>(readUnsignedInt8()) << 16) +
      (static_cast<uint32_t>(readUnsignedInt8()) << 8) +
      (static_cast<uint32_t>(readUnsignedInt8()) << 0);
  return result;
}

int8_t SerialDataReader::readInt8()
{
  return static_cast<int8_t>(readUnsignedInt8());
}

int16_t SerialDataReader::readInt16()
{
  return static_cast<int16_t>(readUnsignedInt16());
}

int32_t SerialDataReader::readInt32()
{
  return static_cast<int32_t>(readUnsignedInt32());
}

double SerialDataReader::readFloat16()
{
  return static_cast<double>(readInt16());
}

double SerialDataReader::readFloat32()
{
  return static_cast<double>(readInt32());
}
}
