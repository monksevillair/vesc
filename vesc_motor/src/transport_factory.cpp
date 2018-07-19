/*
Created by clemens on 6/19/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_motor/transport_factory.h>
#include <vesc_driver/serial_transport.h>

namespace vesc_motor
{
TransportFactory::TransportFactory(const ros::NodeHandle& nh, const std::string& parameter_name)
  : nh_(nh), parameter_name_(parameter_name)
{
}

vesc_driver::TransportPtr TransportFactory::getTransport(const std::string& name)
{
  const TransportMap::const_iterator transport_it = transport_map_.find(name);
  if (transport_it != transport_map_.end())
  {
    return transport_it->second;
  }

  // If transport is not in map yet, try to create it from parameters:

  ROS_DEBUG_STREAM("TransportFactory::getTransport::1");

  if (nh_.hasParam(parameter_name_))
  {
    ROS_DEBUG_STREAM("TransportFactory::getTransport::2");

    XmlRpc::XmlRpcValue transport_mappings;
    nh_.getParam(parameter_name_, transport_mappings);

    if (transport_mappings.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      throw std::invalid_argument(parameter_name_ + " parameter has wrong type");
    }

    ROS_DEBUG_STREAM("TransportFactory::getTransport::3");
    for (int i = 0; i < transport_mappings.size(); ++i)
    {
      ROS_DEBUG_STREAM("TransportFactory::getTransport::4");

      XmlRpc::XmlRpcValue& transport_mapping = transport_mappings[i];

      if (transport_mapping.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        throw std::invalid_argument("transport mapping has wrong type");
      }

      const auto transport_name = getRequiredParameter<std::string>(transport_mapping, "transport_name");
      if (transport_name == name)
      {
        const auto controller_id = getRequiredParameter<int>(transport_mapping, "controller_id");
        const auto port = getRequiredParameter<std::string>(transport_mapping, "port");

        if (!(std::numeric_limits<uint8_t>::min() <= controller_id
          && controller_id <= std::numeric_limits<uint8_t>::max()))
        {
          throw std::invalid_argument("transport mapping controller id is outside valid range");
        }

        ROS_DEBUG_STREAM("TransportFactory::getTransport::5");

        const auto serial_transport = createSerialTransport(static_cast<uint8_t>(controller_id), port);

        ROS_DEBUG_STREAM("TransportFactory::getTransport::7");

        transport_map_[transport_name] = serial_transport;

        ROS_DEBUG_STREAM("TransportFactory::getTransport::8 transport_name: '" << transport_name << "'");

        return serial_transport;
      }
    }
  }
  throw std::invalid_argument("no transport mapping named '" + name + "'");
}

vesc_driver::TransportPtr TransportFactory::createSerialTransport(uint8_t controller_id,
                                                                                    const std::string& port)
{
  ROS_DEBUG_STREAM("TransportFactory::createSerialTransport: create");

  const auto serial_transport = std::make_shared<vesc_driver::SerialTransport>(controller_id, port);

  ROS_DEBUG_STREAM("TransportFactory::createSerialTransport: connect");

  serial_transport->connect();

  ROS_DEBUG_STREAM("TransportFactory::createSerialTransport: end");

  return serial_transport;
}

template<typename T>
T TransportFactory::getRequiredParameter(XmlRpc::XmlRpcValue& transport_mapping, const std::string& parameter_name)
{
  if (!transport_mapping.hasMember(parameter_name))
  {
    throw std::invalid_argument("transport mapping lacks parameter " + parameter_name);
  }

  return static_cast<T>(transport_mapping[parameter_name]);
}
}
