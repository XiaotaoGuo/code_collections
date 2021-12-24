#include <iostream>

template <typename T1, typename T2>
void showTwoEntities(T1 t1, T2 t2) {
    std::cout << "General template: ";
    std::cout << t1 << ", " << t2 << std::endl;
}

template <>
void showTwoEntities(std::string t1, int t2) {
    std::cout << "Specialized template: ";
    std::cout << t1 << ", " << t2 << std::endl;
}

int main() {
    std::string str = "A";
    int num = 10;
    showTwoEntities(num, num);
    showTwoEntities(str, num);
    return 0;
}