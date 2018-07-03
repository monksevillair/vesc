//
// Created by abuchegger on 02.07.18.
//
#include <vesc_driver/serial_data_writer.h>
#include <algorithm>
#include <stdexcept>

namespace vesc_driver
{
SerialDataWriter::SerialDataWriter(Buffer& buffer)
  : SerialDataWriter(buffer, buffer_.end())
{
}

SerialDataWriter::SerialDataWriter(Buffer& buffer, Buffer::iterator position)
  : buffer_(buffer), position_(position)
{
}

void SerialDataWriter::writeUnsignedInt8(uint8_t value)
{
  position_ = buffer_.insert(position_, value) + 1;
}

void SerialDataWriter::writeUnsignedInt16(uint16_t value)
{
  writeUnsignedInt8(static_cast<uint8_t>(value >> 8));
  writeUnsignedInt8(static_cast<uint8_t>(value >> 0));
}

void SerialDataWriter::writeUnsignedInt32(uint32_t value)
{
  writeUnsignedInt8(static_cast<uint8_t>(value >> 24));
  writeUnsignedInt8(static_cast<uint8_t>(value >> 16));
  writeUnsignedInt8(static_cast<uint8_t>(value >> 8));
  writeUnsignedInt8(static_cast<uint8_t>(value >> 0));
}

void SerialDataWriter::writeInt8(int8_t value)
{
  writeUnsignedInt8(static_cast<uint8_t>(value));
}

void SerialDataWriter::writeInt16(int16_t value)
{
  writeUnsignedInt16(static_cast<uint16_t>(value));
}

void SerialDataWriter::writeInt32(int32_t value)
{
  writeUnsignedInt32(static_cast<uint32_t>(value));
}

void SerialDataWriter::writeFloat16(double value)
{
  if (std::numeric_limits<int16_t>::min() <= value && value <= std::numeric_limits<int16_t>::max())
  {
    writeInt16(static_cast<int16_t>(value));
  }
  throw std::out_of_range("value is outside range for float16");
}

void SerialDataWriter::writeFloat32(double value)
{
  if (std::numeric_limits<int32_t>::min() <= value && value <= std::numeric_limits<int32_t>::max())
  {
    writeInt32(static_cast<int32_t>(value));
  }
  throw std::out_of_range("value is outside range for float32");
}

void SerialDataWriter::writeBlock(Buffer::const_iterator begin, Buffer::const_iterator end)
{
  const ptrdiff_t n = std::distance(begin, end);
  position_ = buffer_.insert(position_, begin, end) + n;
}
}
