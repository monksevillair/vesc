/*
Created by clemens on 6/12/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/periodic_execution.h>

namespace vesc_driver
{
  PeriodicExecution::PeriodicExecution(const std::chrono::duration<double> &execution_duration) :
      execution_duration_(std::chrono::duration_cast<Clock::duration>(execution_duration)), run_(true),
      execution_thread_(&PeriodicExecution::executionLoop, this)
  { }

  void PeriodicExecution::stop()
  {
    std::lock_guard<std::mutex> run_lock(run_mutex_);
    run_ = false;
  }

  void PeriodicExecution::executionLoop()
  {
    std::unique_lock<std::mutex> run_lock(run_mutex_);
    while (run_)
    {
      run_lock.unlock();

      Clock::time_point sleep_till = Clock::now() + execution_duration_;

      execution();

      std::this_thread::sleep_until(sleep_till);

      run_lock.lock();
    }
  }
}
