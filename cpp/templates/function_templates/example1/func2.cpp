#include "utils.hpp"

template int getSum(int*, int);

void func2() {
    int a[3] = {1, 2, 3};
    getSum(a, 3);
}