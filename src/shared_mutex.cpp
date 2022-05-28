#include <chrono>
#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>

void print(const std::string& str) {
    static std::mutex mtx;
    std::lock_guard<std::mutex> lock(mtx);
    std::cout << str << std::endl;
}

void sleep(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
    std::shared_mutex mtx;  // shared mutex

    // write process
    auto write = [&] {
        std::lock_guard<std::shared_mutex> lock(mtx);
        print("write start.");
        sleep(2);
        print("write done.");
    };

    // read process
    auto read = [&] {
        std::shared_lock<std::shared_mutex> lock(mtx);
        print("read start.");
        sleep(2);
        print("read done.");
    };

    std::vector<std::thread> threads;
    threads.emplace_back(std::thread([&] {
        read();
    }));
    sleep(1);
    threads.emplace_back(std::thread([&] {
        read();
    }));
    sleep(1);
    threads.emplace_back(std::thread([&] {
        write();
    }));
    sleep(1);
    threads.emplace_back(std::thread([&] {
        read();
    }));
    sleep(1);
    threads.emplace_back(std::thread([&] {
        read();
    }));
    sleep(1);
    threads.emplace_back(std::thread([&] {
        write();
    }));
    sleep(1);
    threads.emplace_back(std::thread([&] {
        read();
    }));
    sleep(1);
    threads.emplace_back(std::thread([&] {
        read();
    }));
    sleep(1);

    for (auto&& i : threads) {
        i.join();
    }

    return 0;
}
