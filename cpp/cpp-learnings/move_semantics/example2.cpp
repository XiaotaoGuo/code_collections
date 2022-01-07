int add(int a, int b)
{
    return a + b;
}

int main()
{
    int a = 10;

    int *p_a = &a;
    *p_a = 20;

    int& left_ref_a = a;
    left_ref_a = 20;

    // int&& ref_a = a; // error!
    int&& right_ref_b = 10;
    right_ref_b = 20;

    int&& right_ref_c = add(10, 20);
    right_ref_c = 50;

    return 0;
}