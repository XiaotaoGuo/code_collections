#include <iostream>

template <typename T>
class Base {
public:
    Base() { std::cout << "General base.\n"; }
};

template <>
class Base<int> {
public:
    Base() { std::cout << "Specialized base.\n"; }
};

int main() {
    Base<int> b_int;
    Base<double> b_double;
    return 0;
}
