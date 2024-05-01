#pragma once

#include <atomic>
#include <mutex>
#include <condition_variable>

class Barrier {
private:
	std::mutex m;
	std::condition_variable flag;

	int expectedCount;
	int arrivedCount;

public:
	Barrier(int expectedCount) : expectedCount(expectedCount), arrivedCount(0) {}

	void arrive() {
		std::unique_lock<std::mutex> lock(m);

		this->arrivedCount++;

		if (this->arrivedCount == this->expectedCount) {
			this->flag.notify_all();
		} else {
			this->flag.wait(lock);
		}
	}

	int getExpectedCount() {
		std::unique_lock<std::mutex> lock(m);
		return this->expectedCount;
	}

	void setExpectedCount(int expectedCount) {
		std::unique_lock<std::mutex> lock(m);
		this->expectedCount = expectedCount;
	}

	int getArrivedCount() {
		std::unique_lock<std::mutex> lock(m);
		return this->arrivedCount;
	}

	void setArrivedCount(int arrivedCount) {
		std::unique_lock<std::mutex> lock(m);
		this->arrivedCount = arrivedCount;
	}
};