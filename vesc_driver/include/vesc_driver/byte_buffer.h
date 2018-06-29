/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_BYTE_BUFFER_H
#define VESC_DRIVER_BYTE_BUFFER_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace vesc_driver
{
/**
 * Represents a buffer of bytes. Has helper methods to read and write numbers of various types. The parsing methods
 * throw std::out_of_range if the buffer contains less bytes than would be necessary to read the number.
 */
class ByteBuffer
{
public:
  ByteBuffer() = default;

  explicit ByteBuffer(std::vector<uint8_t>&& input_bytes);

  void addBytes(const std::vector<uint8_t>& input_bytes);

  size_t getSize() const;

  void clear();

  uint8_t parseUnsignedInt8();
  uint16_t parseUnsignedInt16();
  uint32_t parseUnsignedInt32();

  int8_t parseInt8();
  int16_t parseInt16();
  int32_t parseInt32();

  float parseFloat16();
  float parseFloat32();

  void addUnsignedInt8(uint8_t value);
  void addUnsignedInt16(uint16_t value);
  void addUnsignedInt32(uint32_t value);

  void addInt8(int8_t value);
  void addInt16(int16_t value);
  void addInt32(int32_t value);

  void addFloat16(double value);
  void addFloat32(double value);

  void resetParsing();
  bool canParseFurther() const;

  void advanceTo(size_t index);
  void reverseAdvanceTo(size_t index);

  void advanceBy(size_t index);
  void retreatBy(size_t index);

  void reduceToParsingPoint();

  explicit operator std::vector<uint8_t>() const;

private:
  std::vector<uint8_t> buffer_;
  size_t parsing_index_ = 0;
};
}

#endif //VESC_DRIVER_BYTE_BUFFER_H
