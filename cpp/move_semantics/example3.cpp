#include <stdio.h>

class Resource {
public:
    Resource(int a_) : a(a_) {}

public:
    int a;
};

class ResourceOwner {
public:
    ResourceOwner(int a) { the_resource = new Resource(a); }

    ResourceOwner(const ResourceOwner& other) {
        printf("copy %d\n", other.the_resource->a);
        the_resource = new Resource(other.the_resource->a);
    }

    ResourceOwner& operator=(const ResourceOwner& other) {
        printf("clean %d\n", the_resource->a);
        if (the_resource) {
            delete the_resource;
        }

        printf("assign %d\n", other.the_resource->a);
        the_resource = new Resource(other.the_resource->a);

        return *this;
    }

    ~ResourceOwner() {
        if (the_resource) {
            printf("destructor %d\n", the_resource->a);
            delete the_resource;
        }
    }

private:
    Resource* the_resource = nullptr;
};

void testCopy() {  // case 1
    printf("=====start testCopy()=====\n");

    ResourceOwner res1(1);
    ResourceOwner res2 = res1;  // copy res1

    printf("=====destructors for stack vars, ignore=====\n");
}

void testAssign() {  // case 2
    printf("=====start testAssign()=====\n");

    ResourceOwner res1(1);
    ResourceOwner res2(2);
    res2 = res1;  // copy res1, assign res1, destrctor res2

    printf("=====destructors for stack vars, ignore=====\n");
}

void testRValue() {  // case 3
    printf("=====start testRValue()=====\n");

    ResourceOwner res2(2);
    res2 = ResourceOwner(1);  // copy res1, assign res1, destructor res2, destructor res1

    printf("=====destructors for stack vars, ignore=====\n");
}

int main() {
    testCopy();
    testAssign();
    testRValue();
}