//
// Created by abuchegger on 06.08.18.
//
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vesc_driver/serial_packet_codec.h>

TEST(SerialPacketCodecTest, EncodeSetPositionPacketTest)
{
  vesc_driver::SerialDataWriter::Buffer buffer;
  vesc_driver::SerialDataWriter writer(buffer);
  vesc_driver::SerialPacketCodec codec;

  vesc_driver::SetPositionPacket packet;
  packet.position = 1.0;
  codec.encode(writer, packet);

  EXPECT_EQ(5, buffer.size());
  EXPECT_EQ(9, buffer.at(0));
  EXPECT_EQ((1000000 >> 24) & 0xFF, buffer.at(1));
  EXPECT_EQ((1000000 >> 16) & 0xFF, buffer.at(2));
  EXPECT_EQ((1000000 >> 8) & 0xFF, buffer.at(3));
  EXPECT_EQ((1000000 >> 0) & 0xFF, buffer.at(4));
}

TEST(SerialPacketCodecTest, EncodeSetBrakePacketTest)
{
  vesc_driver::SerialDataWriter::Buffer buffer;
  vesc_driver::SerialDataWriter writer(buffer);
  vesc_driver::SerialPacketCodec codec;

  vesc_driver::SetBrakePacket packet;
  packet.brake_current = 1.0;
  codec.encode(writer, packet);

  EXPECT_EQ(5, buffer.size());
  EXPECT_EQ(7, buffer.at(0));
  EXPECT_EQ((1000 >> 24) & 0xFF, buffer.at(1));
  EXPECT_EQ((1000 >> 16) & 0xFF, buffer.at(2));
  EXPECT_EQ((1000 >> 8) & 0xFF, buffer.at(3));
  EXPECT_EQ((1000 >> 0) & 0xFF, buffer.at(4));
}

TEST(SerialPacketCodecTest, EncodeForwardCanTest)
{
  vesc_driver::SerialDataWriter::Buffer buffer;
  vesc_driver::SerialDataWriter writer(buffer);
  vesc_driver::SerialPacketCodec codec;

  vesc_driver::SetBrakePacket packet;
  packet.brake_current = 1.0;
  codec.encodeForwardCan(writer, 42, packet);

  EXPECT_EQ(7, buffer.size());
  EXPECT_EQ(34, buffer.at(0));
  EXPECT_EQ(42, buffer.at(1));
  EXPECT_EQ(7, buffer.at(2));
  EXPECT_EQ((1000 >> 24) & 0xFF, buffer.at(3));
  EXPECT_EQ((1000 >> 16) & 0xFF, buffer.at(4));
  EXPECT_EQ((1000 >> 8) & 0xFF, buffer.at(5));
  EXPECT_EQ((1000 >> 0) & 0xFF, buffer.at(6));
}

TEST(SerialPacketCodecTest, DecodeFirmwareVersionPacketTest)
{
  vesc_driver::SerialDataWriter::Buffer buffer({0, 1, 2});
  vesc_driver::SerialDataReader reader(buffer.begin(), buffer.end());
  vesc_driver::SerialPacketCodec codec;

  const boost::optional<vesc_driver::ResponsePacket> packet = codec.decode(reader);

  ASSERT_TRUE(static_cast<bool>(packet));
  EXPECT_EQ(typeid(vesc_driver::FirmwareVersion), packet->type());

  const vesc_driver::FirmwareVersion* firmware_version = boost::get<vesc_driver::FirmwareVersion>(&packet.get());

  ASSERT_NE(nullptr, firmware_version);
  EXPECT_EQ(1, firmware_version->major_version);
  EXPECT_EQ(2, firmware_version->minor_version);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
