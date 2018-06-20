/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/byte_buffer.h>
#include <utility>
#include <ros/ros.h>

namespace vesc_driver
{
  ByteBuffer::ByteBuffer() : parsing_index_(0)
  { }

  ByteBuffer::ByteBuffer(std::vector<uint8_t> &&input_bytes) :
      buffer_(std::forward<std::vector<uint8_t>>(input_bytes)), parsing_index_(0)
  { }

  void ByteBuffer::addBytes(const std::vector<uint8_t> &input_bytes)
  {
    buffer_.insert(buffer_.end(), input_bytes.begin(), input_bytes.end());

    ROS_DEBUG_STREAM("ByteBuffer::addBytes::1");
    for (auto byte : input_bytes)
          ROS_DEBUG_STREAM("ByteBuffer::addBytes::1.1 input_bytes: " << static_cast<int>(byte));    
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

  uint8_t ByteBuffer::parsUnsignedInt8()
  {
    uint8_t result = 0;
    size_t new_parsing_index;

    ROS_DEBUG_STREAM("ByteBuffer::parsUnsignedInt8::1 parsing_index_: " << parsing_index_);

    if ((buffer_.size() - 1) < parsing_index_)
    {
      new_parsing_index = buffer_.size();
    }
    else
    {
      new_parsing_index = parsing_index_ + 1;
      result = buffer_[parsing_index_];
    }

    parsing_index_= new_parsing_index;

    ROS_DEBUG_STREAM("ByteBuffer::parsUnsignedInt8::2 parsing_index_: " << parsing_index_);

    return result;
  }

  uint16_t ByteBuffer::parsUnsignedInt16()
  {
    uint16_t result = 0;
    size_t new_parsing_index;

    if ((buffer_.size() - 1) < (parsing_index_ + 1))
    {
      new_parsing_index = buffer_.size();
    }
    else
    {
      new_parsing_index = parsing_index_ + 2;
      result = (static_cast<uint16_t>(buffer_[parsing_index_]) << 8) +
               static_cast<uint16_t>(buffer_[parsing_index_ + 1]);
    }

    parsing_index_= new_parsing_index;

    return result;
  }

  uint32_t ByteBuffer::parsUnsignedInt32()
  {
    uint32_t result = 0;
    size_t new_parsing_index;

    if ((buffer_.size() - 1) < (parsing_index_ + 3))
    {
      new_parsing_index = buffer_.size();
    }
    else
    {
      new_parsing_index = parsing_index_ + 4;
      result = (static_cast<uint32_t>(buffer_[parsing_index_]) << 24) +
               (static_cast<uint32_t>(buffer_[parsing_index_ + 1]) << 16) +
               (static_cast<uint32_t>(buffer_[parsing_index_ + 2]) << 8) +
               static_cast<uint32_t>(buffer_[parsing_index_ + 3]);
    }

    parsing_index_= new_parsing_index;

    return result;
  }

  int8_t ByteBuffer::parsInt8()
  {
    return static_cast<int8_t>(parsUnsignedInt8());
  }

  int16_t ByteBuffer::parsInt16()
  {
    return static_cast<int16_t>(parsUnsignedInt16());
  }

  int32_t ByteBuffer::parsInt32()
  {
    return static_cast<int32_t>(parsUnsignedInt32());
  }

  float ByteBuffer::parsFloat16()
  {
    return static_cast<float>(parsInt16());
  }

  float ByteBuffer::parsFloat32()
  {
    return static_cast<float>(parsInt32());
  }

  void ByteBuffer::resetParsing()
  {
    parsing_index_ = 0;
  }

  bool ByteBuffer::canFurtherPars()
  {
    ROS_DEBUG_STREAM("ByteBuffer::canFurtherPars::1 parsing_index_: " << parsing_index_ << " buffer_.size(): " << buffer_.size());

    return parsing_index_ < buffer_.size();
  }

  void ByteBuffer::advanceTo(size_t index)
  {
    parsing_index_ = index;
  }

  void ByteBuffer::reverseAdvanceTo(size_t index)
  {
    if (buffer_.empty() || ((buffer_.size() - 1) < index))
    {
      parsing_index_ = 0;
      return;
    }

    parsing_index_ = buffer_.size() - 1 - index;
  }

  void ByteBuffer::step(size_t index)
  {
    parsing_index_+= index;
  }

  void ByteBuffer::reverseStep(size_t index)
  {
    if (parsing_index_ < index)
    {
      parsing_index_ = 0;
      return;
    }

    parsing_index_-=index;
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

  void ByteBuffer::addUnsigedInt8(uint8_t value)
  {
    buffer_.push_back(value);
  }

  void ByteBuffer::addUnsigedInt16(uint16_t value)
  {
    buffer_.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    buffer_.push_back(static_cast<uint8_t>(value & 0xFF));
  }

  void ByteBuffer::addUnsigedInt32(uint32_t value)
  {
    buffer_.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
    buffer_.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    buffer_.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    buffer_.push_back(static_cast<uint8_t>(value & 0xFF));
  }

  void ByteBuffer::addInt8(int8_t value)
  {
    addUnsigedInt8(static_cast<uint8_t>(value));
  }

  void ByteBuffer::addInt16(int16_t value)
  {
    addUnsigedInt16(static_cast<uint16_t>(value));
  }

  void ByteBuffer::addInt32(int32_t value)
  {
    addUnsigedInt32(static_cast<uint32_t>(value));
  }

  void ByteBuffer::addFloat16(float value)
  {
    addInt16(static_cast<int16_t>(value));
  }

  void ByteBuffer::addFloat32(float value)
  {
    addInt32(static_cast<int32_t>(value));
  }
}
