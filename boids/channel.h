#pragma once

#include <mutex>
#include <queue>
#include <optional>

template<typename T>
struct Item {
public:
    std::queue<T> queue;
    std::mutex lock;
    std::condition_variable flag;
    bool closed = false;
};

template<typename T>
class Channel {
private:
    std::shared_ptr<Item<T>> item;
    bool isProducer;

    Channel(std::shared_ptr<Item<T>> item, bool isProducer) :
        item(std::move(item)), isProducer(isProducer) {}

public:
    // Disable copy constructor, to ensure there is only one of each Channel
    Channel(Channel const&) = delete;
    // Enable move constructor, to allow Channel ownership to be transferred
    Channel(Channel&&) = default;
    // Disable copy operator, for same reason
    Channel& operator=(Channel&) = delete;
    // Enable move operator, for same reason
    Channel& operator=(Channel&&) = default;

    ~Channel() {
        this->close();
    }

    bool write(T data) {
        if (this->isProducer && !this->item->closed) {
            this->item->lock.lock();
            this->item->queue.push(std::move(data));
            this->item->flag.notify_one();
            this->item->lock.unlock();

            return true;
        }
        else {
            return false;
        }
    }

    std::optional<T> read() {
        std::optional<T> data;

        if (!this->isProducer) {
            std::unique_lock<std::mutex> flagLock(this->item->lock);

            this->item->flag.wait(flagLock, [this]() {
                return !this->item->queue.empty() || this->item->closed;
                });

            if (!this->item->queue.empty()) {
                data = std::move(this->item->queue.front());
                this->item->queue.pop();
            }

            flagLock.unlock();
        }

        return data;
    }

    void close() {
        if (this->item) {
            this->item->lock.lock();
            this->item->closed = true;
            this->item->flag.notify_all();
            this->item->lock.unlock();
        }
    }

    template<typename U>
    friend std::pair<Channel<U>, Channel<U>> make_channel();
};

template<typename T>
std::pair<Channel<T>, Channel<T>> make_channel() {
    std::shared_ptr<Item<T>> itemPtr = std::make_shared<Item<T>>();

    Channel<T> producer(itemPtr, true);
    Channel<T> consumer(itemPtr, false);

    return { std::move(producer), std::move(consumer) };
}