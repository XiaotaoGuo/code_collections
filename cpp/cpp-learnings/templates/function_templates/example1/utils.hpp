#pragma once
#include <iostream>

template <typename T>
T getSum(T* arr, int N) {
    T sum = T(0);
    for (int i = 0; i < N; ++i) {
        sum = sum + arr[i];
    }

    return sum;
}

void func1();
void func2();