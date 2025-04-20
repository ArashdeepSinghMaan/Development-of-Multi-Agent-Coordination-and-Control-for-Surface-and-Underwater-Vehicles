#include"/home/arash/ros2_ws/src/surface/include/surface/directional_space.hpp"
#include "/home/arash/ros2_ws/src/surface/include/surface/mathtool.hpp"
#include<vector>
#include<cmath>

std::vector <double> direction_space_per_obstacle (std::vector <double> last_obstacle_normal){
    std::vector < double> last_ =last_obstacle_normal;
    double sum=0;
    double null_matrix_ [3][3];
    for(double & com: last_){
        com=com*com;
        sum=sum+com;
       
    }
    
double magni_tude=sqrt(sum);
if (magni_tude>0){

    //Find Unit Vector
    //Vector by It's magni_tude
    
    for(double & ve:last_obstacle_normal){
        ve=ve /magni_tude;
    }
  
    std::vector <double> unit_vector= last_obstacle_normal;

    // This will be first column of null_matrix)which is ortthogonal
  // double null_matrix[3][3]  ={0};


   for (size_t i=0;i<3;i++){
    null_matrix_[i][0]=unit_vector[i];
   }



   std::vector<double> secondi ={}; 
   if (unit_vector[0]<1.1 &&unit_vector[0] <0.9){
    secondi ={0,1,0}; 
   }
   else{
   secondi ={1,0,0}; 
   }
//Find a vector which is perpendicular to first column vector.
   std::vector<double> second=cross_product(unit_vector,secondi);
  
 
//Find a vector which is perpendicular to first and second column vector.
   std::vector<double> third=cross_product(unit_vector,second);
   if(magni_tude(second)>0){
    second=normalize(second);}
  
    if(magni_tude(third)>0){
   third=normalize(third);
    }
   for (size_t i=0;i<3;i++){
    null_matrix_[i][1]=second[i];
   }

   for (size_t i=0;i<3;i++){
    null_matrix_[i][2]=third[i];
   }
/*
   for(size_t i=0;i<3;i++){
    for(size_t j=0;j<3;j++){
        cout<<"Index "<<i<<j<<"  "<<null_matrix[i][j]<<endl;
    }
   }
*/
//Trnaspose of Null Matrix
   double transpose_null_matrix[3][3]={{null_matrix_[0][0],null_matrix_[1][0],null_matrix_[2][0]},{null_matrix_[0][1],null_matrix_[1][1],null_matrix_[2][1]},{null_matrix_[0][2],null_matrix_[1][2],null_matrix_[2][2]}};

/*
    for(size_t i=0;i<3;i++){
        for(size_t j=0;j<3;j++){
            cout<<"Index "<<i<<j<<"  "<<transpose_null_matrix[i][j]<<endl;
        }
    };
*/
    double v_x{1};
    double v_y{2};
    double v_z{3};

    //Dot Product with Global Vector
    //To get Directional Space
    std::vector <double> v_d={v_x,
        v_y, 
        v_z};
     magn_original =   magni_tude(v_d);
     //RCLCPP_INFO(this->get_logger(),"Orginal magni_tude [%.2f]",magn_original);
    std::vector <double> directional_space= matrixVectorMultiply(transpose_null_matrix,v_d);
    if(magni_tude(directional_space)>0){
    directional_space=(directional_space);}
    


    //Define a convergence radius
    //double conv_radius{2};

    // We taking convergence dynamics from initial velocity vector from 
    //First Controller

    std::vector <double> conv_vector={0}; 
    if(magni_tude(v_d)>0){
        conv_vector =normalize(second);}

    
    // To get back global vector from tangent space

return directional_space;

}
else{
std::vector <double> no={0};
return no;
}
}