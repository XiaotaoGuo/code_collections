#include "utils.hpp"

template int getSum(int*, int);

void func1() {
    int a[5] = {1, 2, 3, 4, 5};
    getSum(a, 5);
}