/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef VESC_ACKERMANN_OPTIONAL_DATA_PUBLISHER_H
#define VESC_ACKERMANN_OPTIONAL_DATA_PUBLISHER_H

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <string>

namespace vesc_ackermann
{
template<typename M>
class OptionalDataPublisher
{
public:
  OptionalDataPublisher(ros::NodeHandle& nh, const std::string& topic_name, bool should_publish)
  {
    if (should_publish)
    {
      publisher_ = nh.advertise<M>(topic_name, 1);
    }
  }

  void publish(const typename M::_data_type& data)
  {
    if (publisher_)
    {
      M msg;
      msg.data = data;
      publisher_.publish(msg);
    }
  }

private:
  ros::Publisher publisher_;
};
}

#endif //VESC_ACKERMANN_OPTIONAL_DATA_PUBLISHER_H
