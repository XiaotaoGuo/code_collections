#include <boost/type_index.hpp>
#include <iostream>

template <typename T1, typename T2>
class Base {
public:
    Base() {
        std::cout << "General: T1 = " << boost::typeindex::type_id<T1>().pretty_name()
                  << ", T2 = " << boost::typeindex::type_id<T2>().pretty_name() << std::endl;
    }
};

template <typename T>
class Base<int, T> {
public:
    Base() { std::cout << "Specialzied: T1 = int, T2 = " << boost::typeindex::type_id<T>().pretty_name() << std::endl; }
};

int main() {
    Base<int, double> a;
    Base<double, double> b;
    return 0;
}
