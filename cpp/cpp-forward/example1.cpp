int main() {
    using lrint = int&;
    using rrint = int&&;

    int a = 1;

    lrint b = a;
    // (int&)& c = b;  // error!

    lrint& c = a;
    lrint&& d = a;

    rrint&& e = 1;
    return 0;
}