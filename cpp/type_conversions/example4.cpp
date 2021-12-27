#include <iostream>
class Base {
public:
    virtual void func() {}
};

class MiddleA : public virtual Base {};

class MiddleB : public virtual Base {};

/**
 *       Base
 *      /    \
 *     /      \
 * MiddleA  MiddleB
 *    \       /
 *     \     /
 *     Derived
 */
class CrossDerived : public MiddleA, public MiddleB {};

class Derived : public Base {};

int main() {
    CrossDerived a;
    MiddleB& b = a;

    // down cast
    CrossDerived& c = dynamic_cast<CrossDerived&>(b);
    // side cast
    MiddleA& d = dynamic_cast<MiddleA&>(b);

    // most common use case
    Base* e = new Derived();
    Derived* f = dynamic_cast<Derived*>(e);

    return 0;
}