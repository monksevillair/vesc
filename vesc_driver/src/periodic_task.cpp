/*
Created by clemens on 6/12/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/periodic_task.h>

namespace vesc_driver
{
PeriodicTask::PeriodicTask(const std::chrono::duration<double>& period)
  : period_(std::chrono::duration_cast<Clock::duration>(period)),
    execution_thread_(&PeriodicTask::executionLoop, this)
{}

PeriodicTask::~PeriodicTask()
{
  stop();
}

void PeriodicTask::stop()
{
  if (execution_thread_.joinable())
  {
    std::lock_guard<std::mutex> run_lock(run_mutex_);
    running_ = false;
    run_condition_.notify_all();
    execution_thread_.join();
  }
}

bool PeriodicTask::isRunning(const Clock::time_point& end_of_period)
{
  // Wait until stopped or timeout:
  std::unique_lock<std::mutex> run_lock(run_mutex_);
  while (running_ && run_condition_.wait_until(run_lock, end_of_period) != std::cv_status::timeout)
  {
  }
  return running_;
}

void PeriodicTask::executionLoop()
{
  for (Clock::time_point end_of_period = Clock::now(); isRunning(end_of_period); end_of_period += period_)
  {
    execute();
  }
}
}
