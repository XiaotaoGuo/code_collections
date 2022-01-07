#include <stdio.h>

int getSum(int* arr, int N) {
    int sum = 0;
    for (int i = 0; i < N; ++i) {
        sum += arr[i];
    }

    return sum;
}

double getSum(double* arr, int N) {
    double sum = 0;
    for (int i = 0; i < N; ++i) {
        sum += arr[i];
    }

    return sum;
}

// template <typename T>
// T getSum(T* arr, int N) {
//     T sum = T(0);
//     for (int i = 0; i < N; ++i) {
//         sum = sum + arr[i];
//     }

//     return sum;
// }

int main() {
    int a[5] = {1, 2, 3, 4, 5};
    int i_sum = getSum(a, 5);

    double b[3] = {1.0, 1.5, 2.0};
    double d_sum = getSum(b, 3);

    printf("sum of a = %d, sum of b = %f.\n", i_sum, d_sum);

    return 0;
}