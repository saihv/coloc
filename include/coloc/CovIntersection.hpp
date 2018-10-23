#pragma once

#include <cstdio>
#include <dlib/optimization.h>
#include <cmath>
#include <iostream>

#define DIM 3

using namespace dlib;

class CovIntersection
{
public:
	void loadData(matrix<double, DIM, DIM> covA, matrix<double, DIM, DIM> covB, matrix<double, DIM, 1> posA, matrix<double, DIM, 1> posB)
	{
		CA = covA;
		CB = covB;

		ca = posA;
		cb = posB;
	}

	static double function(double x)
	{
		// double value = sum(diag(inv(inv(CA) + inv(CB) - inv(x*CA + (1-x)*CB))));
		return sum(diag(inv(inv(CA) + inv(CB) - inv(x*CA + (1 - x)*CB))));
		// return value;
	}

	static matrix<double, DIM, DIM> CA, CB;
	static matrix<double, DIM, 1> ca, cb;

	void optimize()
	{
		minValue = find_min_single_variable(&CovIntersection::function, starting_point, begin, end, eps, max_iter, initial_search_radius);
		minX = starting_point;
	}

	void computeFusedValues()
	{
		covFused = inv(inv(CA) + inv(CB) - inv(minX*CA + (1 - minX)*CB));

		matrix <double, DIM, DIM> KICI, LICI;
		KICI = covFused * (inv(CA) - minX * inv(minX*CA + (1 - minX)*CB));
		LICI = covFused * (inv(CB) - (1 - minX) * inv(minX*CA + (1 - minX)*CB));

		poseFused = KICI * ca + LICI * cb;
	}

	double minValue;
	double minX;

	matrix <double, DIM, DIM> covFused;
	matrix <double, DIM, 1> poseFused;

private:
	const double begin = 0.0;
	const double end = 1.0;
	double starting_point = 0.0;
	const double eps = 1e-3;
	const long max_iter = 100;
	const double initial_search_radius = 0.01;
	// print variables
};

matrix <double, DIM, DIM> CovIntersection::CA;
matrix <double, DIM, DIM> CovIntersection::CB;
matrix <double, DIM, 1> CovIntersection::ca;
matrix <double, DIM, 1> CovIntersection::cb;