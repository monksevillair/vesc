/*
Created by clemens on 6/12/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_PERIODIC_TASK_H
#define VESC_DRIVER_PERIODIC_TASK_H

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace vesc_driver
{
class PeriodicTask
{
public:
  PeriodicTask(const std::function<void()>& task, const std::chrono::duration<double>& period);
  ~PeriodicTask();

  void start();
  void stop();

protected:
  typedef std::chrono::system_clock Clock;

  bool isRunning(const Clock::time_point& end_of_period);
  void executionLoop();

  std::mutex run_mutex_;
  std::condition_variable run_condition_;
  bool running_ = false;

  std::function<void()> task_;
  Clock::duration period_;
  std::thread execution_thread_;
};
}

#endif //VESC_DRIVER_PERIODIC_TASK_H
