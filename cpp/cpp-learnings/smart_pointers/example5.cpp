#include <iostream>
#include <memory>

template <typename T>
class Unique_ptr {
public:
    /******************* Construct and Assign ********************/
    Unique_ptr(T* res = nullptr) : res_(res) {}
    Unique_ptr& operator=(T* res) {
        delete res_;
        res_ = res;
        return *this;
    }

    Unique_ptr(const Unique_ptr& rhs) = delete;
    Unique_ptr& operator=(const Unique_ptr& rhs) = delete;

    Unique_ptr(Unique_ptr&& rhs) {
        if (rhs.res_ == res_) return;  // do no change
        if (res_) delete res_;

        res_ = rhs.res_;
        rhs.res_ = nullptr;  // make sure that make rhs own nullptr
    }
    Unique_ptr& operator=(Unique_ptr&& rhs) {
        if (rhs.res_ == res_) return *this;  // do no change
        if (res_) delete res_;

        res_ = rhs.res_;
        rhs.res_ = nullptr;  // make sure that make rhs own nullptr

        return *this;
    }
    /******************* Construct and Assign ********************/

    /******************* Usage ************************/
    explicit operator bool() const { return res_; }
    T* operator*() const { return res_; }
    T& operator->() const { return *res_; }
    /******************* Usage ************************/

    ~Unique_ptr() {
        if (res_) delete res_;
    }

private:
    T* res_ = nullptr;
};

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
    { Unique_ptr<Base> p2(new Base(1)); }
    std::cout << "-------------\n";

    // unqiue_ptr can pass resource to another smart pointers
    std::cout << "-------------\n";
    {
        Unique_ptr<Base> p2 = nullptr;
        std::cout << "================\n";
        {
            Unique_ptr<Base> p3 = new Base(2);
            p2 = std::move(p3);
        }
        std::cout << "==================\n";
    }
    std::cout << "-------------\n";

    return 0;
}
