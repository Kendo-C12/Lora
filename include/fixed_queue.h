// FixedQueue.h
#ifndef FIXED_QUEUE_H
#define FIXED_QUEUE_H

#include <Arduino.h>

template <typename T>
class FixedQueue {
private:
    T* arr;
    int front;
    int rear;
    int capacity;
    int count;

public:
    // Constructor with optional size parameter
    FixedQueue(int size = 10) {
        capacity = size;
        arr = new T[capacity];
        front = 0;
        rear = -1;
        count = 0;
    }

    // Destructor
    ~FixedQueue() {
        delete[] arr;
    }

    // Check if queue is empty
    bool empty() {
        return count == 0;
    }

    // Push element to queue
    bool push(T value) {
        if (count == capacity) {
            return false;
        }
        rear = (rear + 1) % capacity;
        arr[rear] = value;
        count++;
        return true;
    }

    // Pop element from queue
    bool pop() {
        if (empty()) {
            return false;
        }
        front = (front + 1) % capacity;
        count--;
        return true;
    }

    // Get front element without removing
    T getFront() {
        if (empty()) {
            return T();
        }
        return arr[front];
    }

    // Get current size
    int size() {
        return count;
    }

    // Get capacity
    int getCapacity() {
        return capacity;
    }

    // Check if queue is full
    bool isFull() {
        return count == capacity;
    }

    // Clear the queue
    void clear() {
        front = 0;
        rear = -1;
        count = 0;
    }
};

#endif
