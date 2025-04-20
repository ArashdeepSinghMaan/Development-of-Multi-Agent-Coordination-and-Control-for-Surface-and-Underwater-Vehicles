#include "/home/arash/ros2_ws/src/surface/include/surface/mathtool.hpp"

#include<vector>
#include <cmath>

std::vector<double> normalize(std::vector<double> any){
    double sum{0};
    double mag{0};

    std::vector<double> cop=any;

    for(double & com: cop){
        com=com*com;
        sum=sum+com;
        

    }
    mag =sqrt(sum);
    
    for(double & co :any){
        co=co/mag;
    }
    return any;  
};

std::vector <double> cross_product(std::vector <double>  first, std::vector <double> doja){

    return{
        first[1]*doja[2] -first[2]*doja[1],
        first[2]*doja[0] -first[0]*doja[2],
        first[0] *doja[1] -first[1]*doja[0]
    };
};

double magni_tude(std::vector <double> first){
            
    double val=sqrt(first[0]*first[0] + first[1]*first[1] + first[2]*first[2]);
    return val;
};

std::vector<double> matrixVectorMultiply(const double matrix[3][3], const std::vector<double> vec) {
           
    // Resultant vector (m x 1)
   std::vector<double> result(3, 0.0);

    // Matrix-Vector Multiplication
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i] += matrix[i][j] * vec[j];
        }
    }

    return result;
};

double norm(const std::vector<double> vec) {
    double sum = 0.0;
    for (double val : vec) sum += val * val;
    return std::sqrt(sum);
};

std::vector<double> vector_difference (std::vector <double> one,std::vector <double> two,std::vector <double> neew){
            
    for(size_t i=0;i<3;i++){
        neew[i]=one[i]-two[i];
    };
    return neew;
};
std::vector <double> weighted (std::vector <double> vec,double weigh){
    for (size_t i=0;i<3;i++){
        vec[i]=vec[i]*weigh;
    }
return vec;
};
std::vector <double> scale (double nn,std::vector <double> vdf){
    for (size_t i=0;i<3;i++){
        vdf[i]=nn*vdf[i];
    }
    return vdf;
};
std::vector<double> weightedSum(const std::vector<double> v1, const std::vector<double> v2, double weight) {
    std::vector<double> result(v1.size());
    for (size_t i = 0; i < v1.size(); i++) {
        result[i] = (1 - weight) * v1[i] + weight * v2[i];
    }
    return result;
};

