#include <iostream>

void showTwoEntities(std::string t1, std::string t2) {
    std::cout << "No template: ";
    std::cout << t1 << ", " << t2 << std::endl;
}

template <typename T>
void showTwoEntities(T t1, T t2) {
    std::cout << "Single type template: ";
    std::cout << t1 << ", " << t2 << std::endl;
}

template <typename T1, typename T2>
void showTwoEntities(T1 t1, T2 t2) {
    std::cout << "General template: ";
    std::cout << t1 << ", " << t2 << std::endl;
}

template <>
void showTwoEntities(std::string t1, std::string t2) {
    std::cout << "Specialized template: ";
    std::cout << t1 << ", " << t2 << std::endl;
}

int main() {
    std::string str = "A";
    showTwoEntities<std::string, std::string>(str, str);
    return 0;
}