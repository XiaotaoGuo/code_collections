#include <iostream>

enum class A { aa = 1, bb, cc };

enum B { ee /* = 0*/, ff };

enum C { gg, hh };

int main() {
    // static_cast does a scoped enum to int/double conversion
    A a = A::aa;
    int b = static_cast<int>(a);
    double c = static_cast<double>(a);
    std::cout << b << ", " << c << std::endl;  // 1, 1

    // static_cast does a enum/int conversion
    int d = 0;
    B e = static_cast<B>(d);  // 0 -> B::ee
    C f = static_cast<C>(e);  // B::ee -> C::gg

    return 0;
}