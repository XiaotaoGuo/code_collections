#include <string>

class Base {
public:
    Base(int a) {}
};

int main() {
    // static_cast does a implicit conversion
    int a = 10;
    Base b = static_cast<Base>(a + 1);

    // static_cast does a inverse of standard conversions through qualification check
    std::string str = "str";
    char* c = &str[0];
    const char* const* d = &c;
    const char* const* const e = static_cast<const char* const* const>(d);

    // static_cast does a array-to-pointer
    int f[5] = {1, 2, 3, 4, 5};
    int* g = static_cast<int*>(f);

    // discard value of a only for suppressing unused warning
    int h;
    static_cast<void>(h);

    return 0;
}