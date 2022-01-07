#include <iostream>

struct Data {
    char a;  // 1 bytes
    char b;  // 1 bytes
    char c;  // 1 bytes
    char d;  // 1 bytes
};

struct PackedData {
    int data;  // 4 bytes
};

int main() {
    // intialized data
    Data d;

    // packed it and send it for some data stream
    PackedData* pd = reinterpret_cast<PackedData*>(&d);

    // data assign, memory layout for pd->data would be '61 62 63 64' or '64 63 62 61' depending on machine
    pd->data = ('a' << 3 * 8) | ('b' << 2 * 8) | ('c' << 1 * 8) | 'd';
    std::cout << pd->data << std::endl;

    // optional, changes have been made on d
    Data* d2 = reinterpret_cast<Data*>(pd);
    std::cout << d2->a << d2->b << d2->c << d2->d << std::endl;

    return 0;
}