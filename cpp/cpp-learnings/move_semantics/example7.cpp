#include <stdio.h>
#include <utility>

class Resource {
public:
    Resource(int a_) : a(a_) {}

public:
    int a;
};

class ResourceOwner {
public:
    ResourceOwner() {}

    ResourceOwner(int a) {
        printf("ResourceOwner: base constructor %d\n", a);
        the_resource = new Resource(a);
    }

    ResourceOwner(const ResourceOwner& other) {
        printf("ResourceOwner: lvalue copy constructor %d\n", other.the_resource->a);
        the_resource = new Resource(other.the_resource->a);
    }

    ResourceOwner(ResourceOwner&& other) {
        printf("ResourceOwner: rvalue copy constructor %d\n", other.the_resource->a);
        the_resource = other.the_resource;
        other.the_resource = nullptr;
    }

    ResourceOwner& operator=(const ResourceOwner& other) {
        if (the_resource) {
            delete the_resource;
        }

        printf("ResourceOwner: lvalue assign %d\n", other.the_resource->a);
        the_resource = new Resource(other.the_resource->a);

        return *this;
    }

    ResourceOwner& operator=(ResourceOwner&& other) {
        if (the_resource) {
            delete the_resource;
        }

        printf("ResourceOwner: rvlue assign %d\n", other.the_resource->a);
        the_resource = other.the_resource;
        other.the_resource = nullptr;

        return *this;
    }

    ~ResourceOwner() {
        if (the_resource) {
            printf("ResourceOwner: destructor %d\n", the_resource->a);
            delete the_resource;
        }
    }

private:
    Resource* the_resource = nullptr;
};

ResourceOwner factory(int a) { return ResourceOwner(a); }

int main() {
    ResourceOwner owner = factory(10);
    return 0;
}