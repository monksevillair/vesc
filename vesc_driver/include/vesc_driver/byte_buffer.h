/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_BYTE_BUFFER_H
#define VESC_DRIVER_BYTE_BUFFER_H

#include <vector>
#include <cstdint>
#include <cstddef>

namespace vesc_driver
{
  class ByteBuffer
  {
  public:
    ByteBuffer();

    explicit ByteBuffer(std::vector<uint8_t> &&input_bytes);

    void addBytes(const std::vector<uint8_t> &input_bytes);

    size_t getSize() const;

    void clear();

    uint8_t parsUnsignedInt8();
    uint16_t parsUnsignedInt16();
    uint32_t parsUnsignedInt32();

    int8_t parsInt8();
    int16_t parsInt16();
    int32_t parsInt32();

    float parsFloat16();
    float parsFloat32();

    void addUnsigedInt8(uint8_t value);
    void addUnsigedInt16(uint16_t value);
    void addUnsigedInt32(uint32_t value);

    void addInt8(int8_t value);
    void addInt16(int16_t value);
    void addInt32(int32_t value);

    void addFloat16(float value);
    void addFloat32(float value);

    void resetParsing();
    bool canFurtherPars();

    void advanceTo(size_t index);
    void reverseAdvanceTo(size_t index);

    void step(size_t index);
    void reverseStep(size_t index);

    void reduceToParsingPoint();

    operator std::vector<uint8_t>() const;

  private:
    std::vector<uint8_t> buffer_;
    size_t parsing_index_;
  };
}

#endif //VESC_DRIVER_BYTE_BUFFER_H
