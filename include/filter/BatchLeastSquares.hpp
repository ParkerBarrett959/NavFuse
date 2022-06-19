//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Batch Least Squares Header                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for Batch Least Squares class                                           //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <iostream>
#include <Eigen/Dense>

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

        /* @UnweightedNonlinearLeastSquares
            Inputs:
                yk: (nxm)x1 dimensional vector of measurements (stacked)
                Hk: (nxm)xn dimensional matrix of measurement models (stacked)
                yx: (nxm)x1 dimensionsal vector of predicted measurments 
                xk: nx1 dimensional initial estimate of state vector
                eps: Convergence Criteria
            Outputs:
                xkp1: nx1 dimensional estimated state vector
                J: Double precision Batch Least Squares cost function value
            Description:
                Function which takes in a batch of measurements and measurement models, yk, and Hk
                stacked column-wise. The algorithm recursively iterates until
                the estimate converges on the unweighted nonlinear least squares state and cost 
                function value estimates  
        */
        bool UnweightedNonlinearLeastSquares(Eigen::VectorXd &yk,
                                             Eigen::MatrixXd &Hk,
                                             Eigen::VectorXd &xk,
                                             double &eps,
                                             Eigen::VectorXd &xkp1,
                                             double &J);

        /* @WeightedNonlinearLeastSquares
            Inputs:
                yk: (nxm)x1 dimensional vector of measurements (stacked)
                Hk: (nxm)xn dimensional matrix of measurement models (stacked)
                xk: nx1 dimensional initial estimate of state vector
                Wk: (nxm)x(nxm) dimensional matrix of measurment weights
                eps: Convergence Criteria
            Outputs:
                xkp1: nx1 dimensional estimated state vector
                J: Double precision Batch Least Squares cost function value
            Description:
                Function which takes in a batch of measurements and measurement models, yk, and Hk
                stacked column-wise. The algorithm recursively iterates until
                the estimate converges on the weighted nonlinear least squares state and cost 
                function value estimates  
        */
        bool WeightedNonlinearLeastSquares(Eigen::VectorXd &yk,
                                             Eigen::MatrixXd &Hk,
                                             Eigen::VectorXd &xk,
                                             Eigen::MatrixXd &Wk,
                                             double &eps,
                                             Eigen::VectorXd &xkp1,
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

        /* @ComputePredictedNonlinearMeasurement
            Inputs:
                xk: nx1 dimensional estimated state vector
            Outputs:
                yx: (nxm)x1 dimensional vector of predicted measurements
            Description:
                Function which takes in the state vector and computes the nonlinear predicted 
                measurement of the form yx = h(xk). The template function is a pass through, the
                actual nonlinear dynamics of the system must be filled. 
        */
        bool ComputePredictedNonlinearMeasurement(Eigen::VectorXd &xk,
                                                  Eigen::VectorXd &yx);


};

