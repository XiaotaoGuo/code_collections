#include <iostream>

template <typename T>
void print(T&& t) {
    std::cout << t << std::endl;
}

int main() {
    int a = 1;
    print(a);

    print(10);
    return 0;
}