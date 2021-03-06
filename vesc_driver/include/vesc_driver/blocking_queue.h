/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_BLOCKING_QUEUE_H
#define VESC_DRIVER_BLOCKING_QUEUE_H

#include <condition_variable>
#include <exception>
#include <mutex>
#include <queue>

namespace vesc_driver
{
class InterruptException : public std::exception
{
};

template<class E>
class BlockingQueue
{
public:
  void push(E&& element)
  {
    std::lock_guard<std::mutex> queue_lock(queue_mutex_);

    queue_.push(std::move(element));

    queue_condition_variable_.notify_all();
  }

  E pop() throw(InterruptException)
  {
    std::unique_lock<std::mutex> queue_lock(queue_mutex_);

    while (queue_.empty() && !interrupt_)
    {
      queue_condition_variable_.wait(queue_lock);
    }

    if (interrupt_)
    {
      throw InterruptException();
    }

    E element = queue_.front();
    queue_.pop();

    return element;
  }

  void interrupt()
  {
    std::lock_guard<std::mutex> queue_lock(queue_mutex_);

    interrupt_ = true;

    queue_condition_variable_.notify_all();
  }

private:
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_variable_;

  std::queue<E> queue_;

  bool interrupt_ = false;
};
}

#endif //VESC_DRIVER_BLOCKING_QUEUE_H
