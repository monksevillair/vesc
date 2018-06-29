/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/byte_buffer.h>
#include <limits>
#include <ros/console.h>
#include <utility>

namespace vesc_driver
{
ByteBuffer::ByteBuffer(std::vector<uint8_t>&& input_bytes)
  : buffer_(std::forward(input_bytes))
{}

void ByteBuffer::addBytes(const std::vector<uint8_t>& input_bytes)
{
  buffer_.insert(buffer_.end(), input_bytes.begin(), input_bytes.end());

  ROS_DEBUG_STREAM("ByteBuffer::addBytes::1");
  for (const auto byte : input_bytes)
  {
    ROS_DEBUG_STREAM("ByteBuffer::addBytes::1.1 input_bytes: " << static_cast<int>(byte));
  }
}

size_t ByteBuffer::getSize() const
{
  return buffer_.size();
}

void ByteBuffer::clear()
{
  buffer_.clear();
  parsing_index_ = 0;
}

uint8_t ByteBuffer::parseUnsignedInt8()
{
  return buffer_.at(parsing_index_++);
}

uint16_t ByteBuffer::parseUnsignedInt16()
{
  const uint16_t result
    = (static_cast<uint16_t>(buffer_.at(parsing_index_ + 0)) << 8) +
      (static_cast<uint16_t>(buffer_.at(parsing_index_ + 1)) << 0);
  parsing_index_ += 2;
  return result;
}

uint32_t ByteBuffer::parseUnsignedInt32()
{
  const uint32_t result
    = (static_cast<uint32_t>(buffer_.at(parsing_index_ + 0)) << 24) +
      (static_cast<uint32_t>(buffer_.at(parsing_index_ + 1)) << 16) +
      (static_cast<uint32_t>(buffer_.at(parsing_index_ + 2)) << 8) +
      (static_cast<uint32_t>(buffer_.at(parsing_index_ + 3)) << 0);
  parsing_index_ += 4;
  return result;
}

int8_t ByteBuffer::parseInt8()
{
  return static_cast<int8_t>(parseUnsignedInt8());
}

int16_t ByteBuffer::parseInt16()
{
  return static_cast<int16_t>(parseUnsignedInt16());
}

int32_t ByteBuffer::parseInt32()
{
  return static_cast<int32_t>(parseUnsignedInt32());
}

float ByteBuffer::parseFloat16()
{
  return static_cast<float>(parseInt16());
}

float ByteBuffer::parseFloat32()
{
  return static_cast<float>(parseInt32());
}

void ByteBuffer::resetParsing()
{
  parsing_index_ = 0;
}

bool ByteBuffer::canParseFurther() const
{
  ROS_DEBUG_STREAM(
    "ByteBuffer::canParseFurther::1 parsing_index_: " << parsing_index_ << " buffer_.size(): " << buffer_.size());

  return parsing_index_ < buffer_.size();
}

void ByteBuffer::advanceTo(size_t index)
{
  parsing_index_ = index;
}

void ByteBuffer::reverseAdvanceTo(size_t index)
{
  parsing_index_ = buffer_.size() - std::min(index + 1, buffer_.size());
}

void ByteBuffer::advanceBy(size_t index)
{
  parsing_index_ += index;
}

void ByteBuffer::retreatBy(size_t index)
{
  parsing_index_ -= std::min(index, parsing_index_);
}

void ByteBuffer::reduceToParsingPoint()
{
  buffer_.erase(buffer_.begin(), buffer_.begin() + parsing_index_);
  parsing_index_ = 0;
}

ByteBuffer::operator std::vector<uint8_t>() const
{
  return buffer_;
}

void ByteBuffer::addUnsignedInt8(uint8_t value)
{
  buffer_.push_back(value);
}

void ByteBuffer::addUnsignedInt16(uint16_t value)
{
  buffer_.push_back(static_cast<uint8_t>(value >> 8));
  buffer_.push_back(static_cast<uint8_t>(value >> 0));
}

void ByteBuffer::addUnsignedInt32(uint32_t value)
{
  buffer_.push_back(static_cast<uint8_t>(value >> 24));
  buffer_.push_back(static_cast<uint8_t>(value >> 16));
  buffer_.push_back(static_cast<uint8_t>(value >> 8));
  buffer_.push_back(static_cast<uint8_t>(value >> 0));
}

void ByteBuffer::addInt8(int8_t value)
{
  addUnsignedInt8(static_cast<uint8_t>(value));
}

void ByteBuffer::addInt16(int16_t value)
{
  addUnsignedInt16(static_cast<uint16_t>(value));
}

void ByteBuffer::addInt32(int32_t value)
{
  addUnsignedInt32(static_cast<uint32_t>(value));
}

void ByteBuffer::addFloat16(double value)
{
  if (std::numeric_limits<int16_t>::min() <= value && value <= std::numeric_limits<int16_t>::max())
  {
    addInt16(static_cast<int16_t>(value));
  }
  throw std::out_of_range("value is outside range for float16");
}

void ByteBuffer::addFloat32(double value)
{
  if (std::numeric_limits<int32_t>::min() <= value && value <= std::numeric_limits<int32_t>::max())
  {
  addInt32(static_cast<int32_t>(value));
  }
  throw std::out_of_range("value is outside range for float32");
}
}
