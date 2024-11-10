//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpServices/WorkQueue.h"

#include <exception>
#include <mutex> // std::lock_guard

#include "SpCore/Boost.h"

void WorkQueue::initialize()
{
    std::lock_guard<std::mutex> lock(mutex_);
    error_state_ = WorkQueueErrorState::NoError;
}

void WorkQueue::run()
{
    // run all scheduled work and wait for executor_work_guard_.reset() to be called from a worker thread
    io_context_.run();

    // local copy of error state, needs to be outside of the lock's scope
    WorkQueueErrorState error_state = WorkQueueErrorState::NoError;
    std::exception_ptr exception_ptr = nullptr;

    {
        std::lock_guard<std::mutex> lock(mutex_);

        // reinitialize io_context_ and executor_work_guard_ to prepare for the next call to run()
        io_context_.restart();
        new(&executor_work_guard_) boost::asio::executor_work_guard<boost::asio::io_context::executor_type>(io_context_.get_executor());

        // make a local copy of the shared error state
        error_state = error_state_;
        exception_ptr = exception_ptr_;
    }

    // if there is an error, it means there was an exception on the RPC worker thread, so rethrow now that
    // we're on the game thread so a higher-level system (e.g., EngineService) can put itself into an error
    // state
    if (error_state == WorkQueueErrorState::Error) {
        std::rethrow_exception(exception_ptr);
    }
}

void WorkQueue::reset()
{    
    // request io_context_.run() to stop executing once all of its scheduled work is finished
    std::lock_guard<std::mutex> lock(mutex_);
    executor_work_guard_.reset();
}
