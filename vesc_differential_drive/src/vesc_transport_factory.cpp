/*
Created by clemens on 6/19/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_differential_drive/vesc_transport_factory.h>

namespace vesc_differential_drive
{
  VescTransportFactory::VescTransportFactory(const ros::NodeHandle &nh) : nh_(nh)
  {
    ROS_INFO_STREAM("VescTransportFactory::VescTransportFactory::1");

    if (nh_.hasParam("transport_mapping"))
    {
      ROS_INFO_STREAM("VescTransportFactory::VescTransportFactory::2");

      XmlRpc::XmlRpcValue transport_mappings;
      nh_.getParam("transport_mapping", transport_mappings);

      if (transport_mappings.getType() != XmlRpc::XmlRpcValue::TypeArray)
        throw std::invalid_argument("transport mappings has wrong type");

      ROS_INFO_STREAM("VescTransportFactory::VescTransportFactory::3");
      for (size_t i = 0; i < transport_mappings.size(); ++i)
      {
        ROS_INFO_STREAM("VescTransportFactory::VescTransportFactory::4");

        XmlRpc::XmlRpcValue transport_mapping = transport_mappings[i];

        if (transport_mapping.getType() != XmlRpc::XmlRpcValue::TypeStruct)
          throw std::invalid_argument("transport mappings has wrong type");

        if (!transport_mapping.hasMember("transport_name"))
          throw std::invalid_argument("transport mappings has not specified a transport name");
        std::string transport_name = transport_mapping["transport_name"];

        if (!transport_mapping.hasMember("controller_id"))
          throw std::invalid_argument("transport mappings has not specified a controller id");
        int controller_id = transport_mapping["controller_id"];

        if (!transport_mapping.hasMember("port"))
          throw std::invalid_argument("transport mappings has not specified a port");
        std::string port = transport_mapping["port"];

        ROS_INFO_STREAM("VescTransportFactory::VescTransportFactory::5");

        std::shared_ptr<vesc_driver::SerialTransport> serial_transport =
            std::make_shared<vesc_driver::SerialTransport>(controller_id);

        ROS_INFO_STREAM("VescTransportFactory::VescTransportFactory::6");

        serial_transport->connect(port);

        ROS_INFO_STREAM("VescTransportFactory::VescTransportFactory::7");

        transport_map_[transport_name] = serial_transport;

        ROS_INFO_STREAM("VescTransportFactory::VescTransportFactory::8");
      }
    }
  }

  std::shared_ptr<vesc_driver::SerialTransport>
  VescTransportFactory::getSerialTransport(const std::string &transport_name)
  {
    auto transport_map_it = transport_map_.find(transport_name);

    if (transport_map_it == transport_map_.end())
      return std::shared_ptr<vesc_driver::SerialTransport>();

    return transport_map_it->second;
  }
}