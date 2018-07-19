/*
Created by clemens on 6/19/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef VESC_MOTOR_TRANSPORT_FACTORY_H
#define VESC_MOTOR_TRANSPORT_FACTORY_H

#include <cstdint>
#include <map>
#include <ros/node_handle.h>
#include <string>
#include <vesc_driver/types.h>

namespace vesc_motor
{
class TransportFactory
{
public:
  explicit TransportFactory(const ros::NodeHandle& nh, const std::string& parameter_name = "transport_mapping");

  /**
   * Returns a transport by name.
   *
   * @throw std::invalid_argument if no transport with the given name exists.
   */
  vesc_driver::TransportPtr getTransport(const std::string& name);

  /**
   * Creates a transport that connects to a VESC via serial port.
   *
   * @param controller_id the ID of the connected VESC. When messages to other IDs are sent over this transport, they
   *                      will be forwarded via CAN.
   * @param port the name of the serial port.
   * @return the (already connected) transport.
   */
  vesc_driver::TransportPtr createSerialTransport(uint8_t controller_id, const std::string& port);

protected:
  typedef std::map<std::string, vesc_driver::TransportPtr> TransportMap;

  template<typename T>
  static T getRequiredParameter(XmlRpc::XmlRpcValue& transport_mapping, const std::string& parameter_name);

  ros::NodeHandle nh_;
  std::string parameter_name_;
  TransportMap transport_map_;
};
}

#endif //VESC_MOTOR_TRANSPORT_FACTORY_H
