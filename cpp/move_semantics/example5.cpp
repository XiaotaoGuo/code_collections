#include <stdio.h>

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
        printf("ResourceOwner: lvalue copy %d\n", other.the_resource->a);
        the_resource = new Resource(other.the_resource->a);
    }

    ResourceOwner(ResourceOwner&& other) {
        printf("ResourceOwner: rvalue copy %d\n", other.the_resource->a);
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

        printf("ResourceOwner: rvlue move %d\n", other.the_resource->a);
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

class ResourceHolder {
public:
    ResourceHolder(int a) : the_resource_owner(a) {}

    ResourceHolder(ResourceHolder&& other) {
        printf("ResourceHolder: rvalue copy\n");
        the_resource_owner = other.the_resource_owner;
    }

private:
    ResourceOwner the_resource_owner;
};

int main() {
    ResourceHolder holder = ResourceHolder(10);
    return 0;
}