int add(int a, int b)
{
    return a + b;
}

int main()
{
    int a = 10;
    int b = a;
    int c = b + 1;
    int sum = add(10, 20);

    int *p = &a;

    return 0;
}