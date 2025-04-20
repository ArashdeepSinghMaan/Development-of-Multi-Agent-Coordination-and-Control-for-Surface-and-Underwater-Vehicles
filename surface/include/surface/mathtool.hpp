#ifndef MATHTOOL_HPP
#define MATHTOOL_HPP


#include<cmath>
#include<vector>

std::vector <double> normalize(std::vector<double> any);
std::vector <double> cross_product(std::vector <double>  first, std::vector <double> doja);
double magni_tude(std::vector <double> first);
std::vector<double> matrixVectorMultiply(const double matrix[3][3], const std::vector<double> vec);
double norm(const std::vector<double> vec) ;
std::vector<double> vector_difference (std::vector <double> one,std::vector <double> two,std::vector <double> neew);
std::vector <double> weighted (std::vector <double> vec,double weigh);
//scaling_vector
std::vector <double> scale (double nn,std::vector <double> vdf);
std::vector<double> weightedSum(const std::vector<double> v1, const std::vector<double> v2, double weight);



#endif