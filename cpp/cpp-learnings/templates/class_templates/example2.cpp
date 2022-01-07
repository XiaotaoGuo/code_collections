template <typename T>
class Base {
public:
    Base(T t) : t(t) {}

private:
    T t;
};

// extern template class Base<int>;

int main() { return 0; }