#include <stdio.h>
#include <string>

class Base {
public:
    Base(const std::string& id, int num) : id_(id), num_(num) {}

    void assignID(const std::string& new_id) { assign(new_id, id_); }
    void assignNum(int new_num) { assign(new_num, num_); }

    void info() { printf("Id: %s, num: %d\n", id_.c_str(), num_); }

    // Wrong! won't work
    // template <typename T>
    // virtual void doThings() {
    //     // do something in base
    // }

private:
    template <typename T>
    void assign(T src, T& dst) {
        dst = src;
    }

    std::string id_;
    int num_;
};

// class Derived : public Base {
// public:
//     // Wrong! won't work
//     template <typename T>
//     virtual void doThings() override {
//         // do something in derived
//     }
// };

int main() {
    Base b("A", 10);
    b.info();

    b.assignID("B");
    b.info();

    b.assignNum(20);
    b.info();

    return 0;
}