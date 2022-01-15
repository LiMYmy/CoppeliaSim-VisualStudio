#include "controller/eigentest.h"
#include "Eigen/Dense"

#include <time.h>
#include <iostream>

using namespace Eigen;
using namespace std;

#define N 999

void eigen_test()
{
	double time1 = (double)clock() / CLOCKS_PER_SEC;

	MatrixXd m = MatrixXd::Random(N, N);            //��ʼ����̬����m,ʹ��Random����,��ʼ����ֵ��[-1,1]������,�����СN * N
	//cout << "m =" << endl << m << endl;
	MatrixXd n(N, N);
	n = m * m;
	n = m.inverse(); //�����
	auto a = m.determinant(); //����ʽ
	auto b = m.eigenvalues(); //����ֵ

	double time2 = (double)clock() / CLOCKS_PER_SEC;
	cout << "used time: " << double(time2 - time1) << endl;
}
