#include "../ShuntingYard.h"
#include <gtest/gtest.h>
#include <cmath>
#define EPSILON_ 0.001

double calc_alg(string_view in) 
{
	ShuntingYard sh1;
	return sh1.Calculation(in);
}
double calc_alg_1(string_view in)
{
    ShuntingYard_2 sh1;
    return sh1.Calculation(in);
}
bool isEq(double value1, double value2, double epsilon)
{
    return fabs(value1 - value2) < epsilon;
}

TEST(Test_variants, _ShuntingYard)
{
    double expected = 68.127;
    string input1 = "(24.5+ 4)*(3.55-1.23)+ 2.89/ 1.44";
    double actual =  calc_alg(input1);
    if (isEq(expected, actual, EPSILON_)) expected = actual;
    ASSERT_EQ(expected, actual);
}

TEST(Test_variants, _ShuntingYard_2)
{
    double expected = 748.7;
    string input1 = "25.2 + 36.5 * 21 - 43 ";
    double actual = calc_alg_1(input1);
    if (isEq(expected, actual, EPSILON_)) expected = actual;
    ASSERT_EQ(expected, actual);
}


int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}