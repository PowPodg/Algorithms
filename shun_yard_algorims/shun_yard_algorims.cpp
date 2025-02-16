#include "ShuntingYard.h"
#include <iostream>
#include<iomanip>
using namespace std;

int main()
{
	//1 var
	string input1 = "(24.5+ 4)*(3.55-1.23)+ 2.89/ 1.44";// = 68.127;
	ShuntingYard sh1;
	auto res1 = sh1.Calculation(input1);

	//2 var - without the brackets!!!!!!
	string input2 = "25.2 + 36.5 * 21 - 43 ";//  = 748.7
	ShuntingYard_2 sh2;
	auto res2 = sh2.Calculation(input2);
    cout << "RESULT_1 : "<< fixed<<setprecision(3)<<res1<<endl;
	cout << "RESULT_2 : "<< fixed<< setprecision(3)<<res2<<endl;
	cin.get();
	return 0;
}


