/*
Created by clemens on 6/19/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef vesc_motor_VESC_TRANSPORT_FACTORY_H
#define vesc_motor_VESC_TRANSPORT_FACTORY_H

#include <ros/ros.h>
#include <vesc_driver/serial_transport.h>
#include <map>

namespace vesc_motor
{
class VescTransportFactory
{
public:
  explicit VescTransportFactory(const ros::NodeHandle &nh);

  std::shared_ptr<vesc_driver::SerialTransport> getSerialTransport(const std::string &transport_name);

private:
  ros::NodeHandle nh_;

  std::map<std::string, std::shared_ptr<vesc_driver::SerialTransport>> transport_map_;
};
}

#endif //vesc_motor_VESC_TRANSPORT_FACTORY_H
