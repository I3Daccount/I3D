#include <Eigen/Sparse>
#include <Eigen/SparseLU>

// need some data source... 
int main() {
	int dimention = 100;
	Eigen::SparseMatrix<float> A(dimention, dimention);
	Eigen::VectorXf U(dimention), V(dimention), B(dimention),C(dimention);

	Eigen::SparseLU<Eigen::SparseMatrix<float>> solver;
	solver.compute(A);
	U = solver.solve(B);
	V = solver.solve(C);
}