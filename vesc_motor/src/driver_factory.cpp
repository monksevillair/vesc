//
// Created by abuchegger on 19.07.18.
//
#include <vesc_motor/driver_factory.h>
#include <limits>
#include <vesc_driver/vesc_driver_impl.h>
#include <vesc_driver/vesc_driver_mockup.h>
#include <vesc_motor/transport_factory.h>

namespace vesc_motor
{
DriverFactory::DriverFactory(TransportFactoryPtr transport_factory, bool use_mockup)
  : transport_factory_(std::move(transport_factory)), use_mockup_(use_mockup)
{
}

vesc_driver::VescDriverInterfacePtr DriverFactory::createDriver(
  const ros::NodeHandle& nh, std::chrono::duration<double> execution_duration,
  vesc_driver::VescDriverInterface::StateHandlerFunction state_handler_function)
{
  if (!use_mockup_)
  {
    int controller_id = 0;
    if (nh.getParam("controller_id", controller_id)
      && 0 <= controller_id && controller_id <= std::numeric_limits<uint8_t>::max())
    {
      vesc_driver::TransportPtr transport;
      std::string port;
      std::string transport_name;

      if (nh.getParam("port", port))
      {
        transport = transport_factory_->createSerialTransport(controller_id, port);
      }
      else if (nh.getParam("transport_name", transport_name))
      {
        transport = transport_factory_->getTransport(transport_name);
      }

      if (transport)
      {
        return std::make_shared<vesc_driver::VescDriverImpl>(
          execution_duration, state_handler_function, controller_id, transport);
      }
      else
      {
        ROS_ERROR_STREAM("Could not determine type of transport from " << nh.getNamespace());
      }
    }
  }

  return std::make_shared<vesc_driver::VescDriverMockup>(execution_duration, state_handler_function);
}
}
