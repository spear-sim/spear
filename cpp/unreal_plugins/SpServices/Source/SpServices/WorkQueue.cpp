//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/WorkQueue.h"

#include <mutex>

#include "SpCore/Boost.h"

void WorkQueue::run()
{
    // run all scheduled work and wait for executor_work_guard_.reset() to be called from a worker thread
    io_context_.run();

    // reinitialize io_context_ and executor_work_guard_ to prepare for the next call to run()
    mutex_.lock();
    io_context_.restart();
    new(&executor_work_guard_) boost::asio::executor_work_guard<boost::asio::io_context::executor_type>(io_context_.get_executor());
    mutex_.unlock();
}

void WorkQueue::reset()
{
    // request io_context_.run() to stop executing once all of its scheduled work is finished
    mutex_.lock();
    executor_work_guard_.reset();
    mutex_.unlock();
}
