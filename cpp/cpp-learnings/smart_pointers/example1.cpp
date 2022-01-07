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
    // unqiue_ptr would dispose resources when going out of scope
    std::cout << "-------------\n";
    { std::unique_ptr<Base> p2 = std::make_unique<Base>(1); }
    std::cout << "-------------\n";

    // unqiue_ptr can pass resource to another smart pointers
    std::cout << "-------------\n";
    {
        std::unique_ptr<Base> p2 = nullptr;
        std::cout << "================\n";
        {
            std::unique_ptr<Base> p3 = std::make_unique<Base>(2);
            p2 = std::move(p3);
        }
        std::cout << "==================\n";
    }
    std::cout << "-------------\n";

    return 0;
}
