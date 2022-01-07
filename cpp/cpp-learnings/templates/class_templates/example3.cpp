template <typename T>
class Base {
public:
    Base(T t) : t(t) {}

    void func1() { t = t + T(1.0); }

    void func2() { t = t - T(1.0); }

private:
    T t;
};

int main() {
    Base<int> b_int(1);

    Base<float>* pb_float;

    Base<double>* pb_double = new Base<double>(10.0);
    pb_double->func1();
    return 0;
}
