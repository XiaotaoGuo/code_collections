template <typename T>
class Base {
public:
    Base(T t) : t(t) {}

private:
    T t;
};

template class Base<int>;

int main() { return 0; }