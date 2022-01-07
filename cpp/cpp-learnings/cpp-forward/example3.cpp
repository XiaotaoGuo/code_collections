#include <iostream>

void showValue(const int& val) { std::cout << "lvalue: " << val << std::endl; }

void showValue(int&& val) { std::cout << "rvalue: " << val << std::endl; }

template <typename T>
void print(T&& t) {
    showValue(t);
}

int main() {
    int a = 1;
    print(a);

    print(std::move(a));
    print(10);
    return 0;
}