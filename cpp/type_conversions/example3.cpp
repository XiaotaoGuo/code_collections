#include <iostream>
class Base {};

class Derived : public Base {
public:
    int a = 1.0;
};

int main() {
    Base a;
    // static_cast does a down casting in wrong situation
    Base* b = &a;
    Derived* c = static_cast<Derived*>(b);
    Derived& d = static_cast<Derived&>(a);

    // will run, but is random value
    std::cout << d.a << std::endl;

    // static_cast can cast a void* to pointer of any type
    void* e = static_cast<void*>(c);
    Derived* f = static_cast<Derived*>(e);

    // upcast of a pointer to member
    int Derived::*pa = &Derived::a;
    std::cout << a.*static_cast<int Base::*>(pa) << std::endl;
    return 0;
}