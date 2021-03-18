//
// Created by cv-bhlab on 3/5/21.
//

#ifndef HYSLAM_THREADSAFEQUEUE_H
#define HYSLAM_THREADSAFEQUEUE_H

#include <queue>
#include <mutex>
#include <iostream>
#include <algorithm>

namespace HYSLAM {

template<typename T>
class ThreadSafeQueue {
    std::queue<T> queue_;
    mutable std::mutex mutex_;

    // Moved out of public interface to prevent races between this
    // and pop().
    bool empty() const {
        return queue_.empty();
    }

public:
    ThreadSafeQueue() = default;
    ThreadSafeQueue(const ThreadSafeQueue<T> &) = delete ;
    ThreadSafeQueue& operator=(const ThreadSafeQueue<T> &) = delete ;

    ThreadSafeQueue(ThreadSafeQueue<T>&& other) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_ = std::move(other.queue_);
    }

    virtual ~ThreadSafeQueue() { }

    unsigned long size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    T pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return T();
        }
        T tmp = queue_.front();
        queue_.pop();
        return tmp;
    }

    void push(const T &item) {
      //  std::cout << "item pushed to queue" << std::endl;
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(item);
    }

    void clear(){
        std::lock_guard<std::mutex> lock(mutex_);
        std::queue<T> empty_queue;
        std::swap(queue_, empty_queue);
    }
};
}//end namespace

#endif //HYSLAM_THREADSAFEQUEUE_H
