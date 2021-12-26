#include <iostream>

template <typename T1>
class Outter {
public:
    Outter() { std::cout << "General Outter.\n"; }

    template <typename T2>
    class Inner {
    public:
        Inner() { std::cout << "General Inner.\n"; }
    };
};

template <>
template <typename T>
class Outter<int>::Inner {
public:
    Inner() { std::cout << "Specialized Inner.\n"; }
};

int main() {
    Outter<int>::Inner<int> a;
    Outter<double>::Inner<int> b;
    return 0;
}
