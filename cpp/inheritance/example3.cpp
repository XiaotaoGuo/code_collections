class Base {
public:
    bool flag = true;
    virtual ~Base() {}
    virtual void flip() { flag = !flag; }

    void reset() { flag = true; }
};

class Derived : public Base {
public:
    bool extra_flag = true;
    int count = 0;
    virtual ~Derived() {}

    virtual void flip() override {
        flag = !flag;
        extra_flag = !extra_flag;
    }

    void iterate() { count++; }
};

int main() {
    Base* actual_derived = new Derived();
    actual_derived->flip();
    actual_derived->reset();
    delete actual_derived;
}