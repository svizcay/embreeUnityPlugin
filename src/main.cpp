#include <iostream>

#include <voxel_visibility_checker.h>

using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char * argv[])
{
    cout << "pluing testing app" << endl;

    initPlugin ();

	float origin[] = { 0, 0, 0 };
	float direction[] = { 0, -1, 0 };
	bool *result = new bool[5];
	cout << "result before ";
	for (int i = 0; i < 5; i++) {
		result[i] = false;
		cout << result[i] << " ";
	}
	cout << endl;
	*result = false;

	bool result2 = false;
	int *result3 = new int[5];
	cout << "result before " << *result << endl;
	cout << "result2 before " << result2 << endl;

	cout << "result3 before ";
	for (int i = 0; i < 5; i++) {
		result3[i] = 9;
		cout << result3[i] << " ";
	}
	cout << endl;


	test(origin, direction, result, result3);
	test(origin, direction, &result2, result3);

	cout << "result after ";
	for (int i = 0; i < 5; i++) {
		cout << result[i] << " ";
	}
	cout << endl;

	cout << "result after " << *result << endl;
	cout << "result2 after " << result2 << endl;

	cout << "result3 after ";
	for (int i = 0; i < 5; i++) {
		cout << result3[i] << " ";
	}
	cout << endl;

	delete[] result;
	delete[] result3;


    finishPlugin ();

    cout << "done" << endl;

	return 0;
}
