//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     Navigation Utility Functions                                 //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  // 
// Author:      Parker Barrett                                                                      //
// Overview:    Header file for navigation utilities class.                                         //           
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Include Headers
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Nav Utilities Class
class NavUtils {

    // Public Class Members/Functions
    public:

        /* @computeDcmFromQuaternion
            Inputs:
                qA2B: 4x1 dimensional quaternion relating reference frame A to frame B
            Outputs:
                RA2B: 3x3 dimensional direction cosines matrix relating reference frame A to frame B
            Description:
                Function which takes in a quaternion and computes the direction cosines matrix.
        */
        bool computeDcmFromQuaternion(Eigen::VectorXd &qA2B,
                                      Eigen::MatrixXd &RA2B);

        /* @computeQuaternionFromRotationVec
            Inputs:
                phi: 3x1 dimensional rotation vector
            Outputs:
                qA2B: 4x1 dimensional quaternion relating reference frame A to frame B
            Description:
                Function which takes in a rotation vector and computes the quaternion.
        */
        bool computeQuaternionFromRotationVec(Eigen::VectorXd &phi,
                                              Eigen::VectorXd &qA2B);

        /* @computeQuaternionFromRotationVec
            Inputs:
                qA2B: 4x1 dimensional quaternion relating reference frame A to frame B
            Outputs:
                QA2B: 4x4 dimensional quaternion equivalent matrix relating reference frame A to frame B
            Description:
                Function which takes in a quaternion and computes the matrix equivalent.
        */
        bool buildQuaternionEquivalent(Eigen::VectorXd &qA2B,
                                       Eigen::MatrixXd &QA2B);

        /* @skewSymmetric
            Inputs:
                vec: 3x1 dimensional vector
            Outputs:
                vecX: 3x3 dimensional skew symmetric matrix
            Description:
                Function which takes in a vector and computes the skew symmetric (cross-product equivalent)
                matrix.
        */
        bool skewSymmetric(Eigen::Vector3d &vec,
                           Eigen::Matrix3d &vecX);

    // Private Class Members/Function
    private:

};

