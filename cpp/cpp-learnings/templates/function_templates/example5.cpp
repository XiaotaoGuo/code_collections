#include <iostream>

template <typename T>
T getSum(T t) {
    return t;
}

template <typename T, typename... Types>
T getSum(T first, Types... rest) {
    return first + getSum(rest...);
}

int main() {
    std::cout << "sum: " << getSum(1, 2, 3.0, 4) << std::endl;
    return 0;
}