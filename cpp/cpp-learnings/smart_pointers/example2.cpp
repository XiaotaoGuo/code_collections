#include <iostream>
#include <memory>

class Base {
public:
    Base(int id) : id(id), count(new int(1000)) { std::cout << "Base::Base() for " << id << std::endl; }

    ~Base() {
        delete count;
        std::cout << "Base::~Base() for " << id << std::endl;
    }

private:
    int id = 0;
    int* count = nullptr;
};

int main() {
    // shared_ptr can be used to shared management for resource
    std::cout << "-------------\n";
    {
        std::shared_ptr<Base> p1 = std::make_shared<Base>(1);
        std::cout << "Use count for 1: " << p1.use_count() << std::endl;
        std::cout << "================\n";
        {
            std::shared_ptr<Base> p2 = p1;
            std::cout << "Use count for 1: " << p1.use_count() << std::endl;
        }
        std::cout << "================\n";
        std::cout << "Use count for 1: " << p1.use_count() << std::endl;
    }
    std::cout << "-------------\n";

    return 0;
}
