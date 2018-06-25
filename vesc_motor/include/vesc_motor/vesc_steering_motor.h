/*
Created by clemens on 6/25/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_MOTOR_VESC_STEERING_MOTOR_H
#define VESC_MOTOR_VESC_STEERING_MOTOR_H

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <vesc_motor/SteeringMotorConfig.h>
#include <vesc_driver/vesc_driver_interface.h>
#include <chrono>
#include <vesc_motor/vesc_transport_factory.h>

namespace vesc_motor
{
  class VescSteeringMotor
  {
  public:
    VescSteeringMotor(const ros::NodeHandle& private_nh, std::shared_ptr<VescTransportFactory> transport_factory,
                      double execution_duration);

    /**
     * Gets the current position motor of the motor in rad, estimated at the given time.
     * @param time time at which velocity is estimated.
     * @return the estimated position in rad.
     */
    double getPosition(const ros::Time& time);

    /**
     * Commands the motor to the position in rad.
     * @param position the position command in rad.
     */
    void setPosition(double position);

    /**
     * Gets the motor controller's supply voltage in V.
     * @return the supply voltage in V.
     */
    double getSupplyVoltage();

  private:
    void reconfigure(SteeringMotorConfig& config, uint32_t level);
    void stateCB(const vesc_driver::MotorControllerState& state);

    bool predict(const ros::Time &time);
    void correct(double position);

    ros::NodeHandle private_nh_;
    dynamic_reconfigure::Server<SteeringMotorConfig> reconfigure_server_;
    SteeringMotorConfig config_;

    std::shared_ptr<VescTransportFactory> transport_factory_;

    std::chrono::duration<double> execution_duration_;

    boost::mutex driver_mutex_;
    boost::shared_ptr<vesc_driver::VescDriverInterface> driver_;

    boost::mutex state_mutex_;
    cv::KalmanFilter position_kf_;
    ros::Time last_prediction_time_;
    double supply_voltage_;
  };
}

#endif //VESC_MOTOR_VESC_STEERING_MOTOR_H
