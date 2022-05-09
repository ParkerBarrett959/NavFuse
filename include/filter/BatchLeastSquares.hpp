//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Batch Least Squares Header                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for Batch Least Squares class                                           //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Kalman Filter Class
class BatchLeastSquares {

    // Public Class Members/Functions
    public:

        /* @UnweightedLinearLeastSquares
            Inputs:
                yk: (nxm)x1 dimensional vector of measurements (stacked)
                Hk: (nxm)xn dimensional matrix of measurement models (stacked)
            Outputs:
                xk: nx1 dimensional estimated state vector
                J: Double precision Batch Least Squares cost function value
            Description:
                Function which takes in a batch of measurements and measurement models, yk, and Hk
                stacked column-wise and estimates the unweighted linear least squares state and 
                cost function value estimates  
        */
        bool UnweightedLinearLeastSquares(Eigen::VectorXd &yk,
                                          Eigen::MatrixXd &Hk,
                                          Eigen::VectorXd &xk,
                                          double &J);

        /* @WeightedLinearLeastSquares
            Inputs:
                yk: (nxm)x1 dimensional vector of measurements (stacked)
                Hk: (nxm)xn dimensional matrix of measurement models (stacked)
                Wk: (nxm)x(nxm) dimensional matrix of measurment weights
            Outputs:
                xk: nx1 dimensional estimated state vector
                J: Double precision Batch Least Squares cost function value
            Description:
                Function which takes in a batch of measurements and measurement models, yk, and Hk
                stacked column-wise and estimates the weighted linear least squares state and 
                cost function value estimates  
        */
        bool WeightedLinearLeastSquares(Eigen::VectorXd &yk,
                                        Eigen::MatrixXd &Hk,
                                        Eigen::MatrixXd &Wk,
                                        Eigen::VectorXd &xk,
                                        double &J);

    // Private Class Members/Function
    private:

        /* @ComputeCostFunction
            Inputs:
                yk: (nxm)x1 dimensional vector of measurements (stacked)
                Hk: (nxm)xn dimensional matrix of measurement models (stacked)
                xk: nx1 dimensional estimated state vector
            Outputs:
                J: Double precision Batch Least Squares cost function value
            Description:
                Function which takes in a batch of measurements and measurement models, yk and Hk
                stacked column-wise, along with the state estimates and computes the cost function 
                cost function value estimates  
        */
        bool ComputeCostFunction(Eigen::VectorXd &yk,
                                 Eigen::MatrixXd &Hk,
                                 Eigen::VectorXd &xk,
                                 double &J);


};

