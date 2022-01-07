#include <iostream>
#include <memory>
#include <vector>

template <typename T>
class Shared_ptr {
public:
    /******************* Construct and Assign ********************/
    Shared_ptr(T* res = nullptr) : res_(res), count_(new int) {
        if (res_) *count_ = 1;
    }
    Shared_ptr& operator=(T* res) {
        if (res == res_) return *this;

        (*count_)--;
        if (*count_ == 0) delete res_;

        res_ = res;
        if (res_) *count_ = 1;
        return *this;
    }

    Shared_ptr(const Shared_ptr& rhs) : res_(rhs.res_), count_(rhs.count_) { (*count_)++; }
    Shared_ptr& operator=(const Shared_ptr& rhs) {
        if (rhs.res_ == res_) return *this;

        (*count_)--;
        if (*count_ == 0) delete res_;

        res_ = rhs.res;
        count_ = rhs.count_;
        *count_++;

        return *this;
    }
    /******************* Construct and Assign ********************/

    /******************* Usage ************************/
    explicit operator bool() const { return res_; }
    T* operator*() const { return res_; }
    T& operator->() const { return *res_; }
    int use_count() const { return *count_; }
    /******************* Usage ************************/

    ~Shared_ptr() {
        (*count_)--;
        if (*count_ == 0) {
            if (res_) delete res_;
            delete count_;
        }
    }

private:
    T* res_ = nullptr;
    int* count_ = nullptr;
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
    // shared_ptr can be used to shared management for resource
    std::cout << "-------------\n";
    {
        Shared_ptr<Base> p1 = new Base(1);
        std::cout << "Use count for 1: " << p1.use_count() << std::endl;
        std::cout << "================\n";
        {
            Shared_ptr<Base> p2 = p1;
            std::cout << "Use count for 1: " << p1.use_count() << std::endl;
        }
        std::cout << "================\n";
        std::cout << "Use count for 1: " << p1.use_count() << std::endl;
    }
    std::cout << "-------------\n";

    return 0;
}
