#include <iostream>

class Base {
public:
    virtual void info() const { std::cout << "Base::Info()" << std::endl; }
};

class Derived : public Base {
public:
    virtual void info() const override { std::cout << "Derived::Info()" << std::endl; }
};

int main(int argc, char** argv) {
    Base* actual_base = new Base();
    Base* actual_derived = new Derived();

    actual_base->info();
    actual_derived->info();

    delete actual_base;
    delete actual_derived;
}