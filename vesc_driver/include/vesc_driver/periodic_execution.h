/*
Created by clemens on 6/12/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_PERIODIC_EXECUTION_H
#define VESC_DRIVER_PERIODIC_EXECUTION_H

#include <mutex>
#include <thread>
#include <chrono>

namespace vesc_driver
{
  class PeriodicExecution
  {
  public:
    typedef std::chrono::system_clock Clock;

    explicit PeriodicExecution(const std::chrono::duration<double> &execution_duration);

    void stop();

  protected:
    virtual void execution() = 0;

  private:
    void executionLoop();

    std::mutex run_mutex_;
    bool run_;

    std::thread execution_thread_;
    Clock::duration execution_duration_;
  };
}

#endif //VESC_DRIVER_PERIODIC_EXECUTION_H
