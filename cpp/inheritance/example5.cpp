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

int main(int argc, char** argv) {
    Base fake_derived = Derived();
    fake_derived.flip();
}