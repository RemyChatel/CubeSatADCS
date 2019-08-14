/**
 * @file   Estimators.cpp
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * @brief  Source code for the Estimator Library
 * 
 * @see Estimators.h
 * 
 * # License
 * <b>(C) Copyright 2019 Remy CHATEL</b>
 * 
 * Licensed Under  GPL v3.0 License
 * http://www.gnu.org/licenses/gpl-3.0.html
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include "Estimators.h"

void Estimators::QUEST(float quat[4], int N, float **s_eci, float **s_body, float *omega, float tolerance = 1e-5){
    Matrix q = Matrix::zeros(4,1);
    Matrix s_a[N];      // ECI frame vector array
    Matrix s_b[N];      // Body frame vector array

    // Converting arrays to Matrix objects
    for(int i = 0; i < N; i++){
        s_a[i] = Matrix(3,1, s_eci[i]);
        s_b[i] = Matrix(3,1, s_body[i]);
    }

    // Running the QUEST algo with matrices as inputs
    QUEST(&q, N, s_a, s_b, omega, tolerance);

    // Returning the quaternion as an array
    quat[0] = q(1);
    quat[1] = q(2);
    quat[2] = q(3);
    quat[3] = q(4);
}

void Estimators::QUEST(Matrix *quat, int N, Matrix *s_eci, Matrix *s_body, float *omega, float tolerance){
    // This algorithm is described in "Spacecraft Dynamics and Control An Introduction"
    // by de Ruiter, Damaren and Forbes, chapter 26

    // Variable to store the solution
    float lambda;       // The estimated eigen value of the problem
    float gamma;        // Quaternion's rotation (of the eigen vector)
    Matrix x;           // Quaternion's vector (of the eigen vector)
    
    // Internal computation variables
    float lambda0 = 0;  // A reasonable initial value for lambda is sum of weights
    Matrix k12(3,1);
    Matrix B(3,3), S(3,3), Id(3,3);
    float k22;
    float a, b, c, d;
    float alpha, beta;
    float detS;
    float trAdjS;
    float normQ;

    // Preparation of the Newton optimisation problem to find lambda
    for(int i = 0; i < N; i++){
        lambda0 += omega[i];
    }

    // Normalization of all vectors
    for(int i = 0; i < N; i++){
        s_eci[i] /= s_eci[i].norm();
        s_body[i] /= s_body[i].norm();
    }

    // Computation of the required factors
    for(int i = 0; i < N; i++){
        B += omega[i] * ( s_eci[i] * s_body[i].Transpose() );
    }

    S = B + B.Transpose();
    B = B.Transpose();
    detS = S.det();

    k22 = B.trace();

    k12(1) = B(2,3) - B(3,2);
    k12(2) = B(3,1) - B(1,3);
    k12(3) = B(1,2) - B(2,1);

    trAdjS  = S(2,2)*S(3,3) - S(3,2)*S(2,3) 
            + S(1,1)*S(3,3) - S(1,3)*S(3,1) 
            + S(1,1)*S(2,2) - S(1,2)*S(2,1);

    a = k22 * k22 - trAdjS;
    b = k22 * k22 + Matrix::dot(k12, k12);
    c = detS + Matrix::dot(k12, S * k12);
    d = Matrix::dot(k12, S * S * k12);

    // Newton's optimization method to find lambda
    int iteration = 0;
    lambda = lambda0;
    float lambda_prev = 0;
    while(fabs(lambda-lambda_prev) > tolerance && iteration < 10000){
        lambda_prev = lambda;
        lambda -= (lambda*lambda*lambda*lambda - (a + b) * lambda * lambda - c *lambda + (a * b + c * k22 - d)) / (4 * lambda * lambda * lambda - 2 * (a + b) * lambda - c);
        iteration++;
    }

    // Then find the eigen vector associated with the eigen value lambda
    Id = Matrix::eye(3);

    alpha = lambda * lambda - a;
    beta = lambda - k22;
    gamma = (lambda + k22) * alpha - detS; 
    x = (alpha * Id + beta * S + S * S) * k12;

    normQ = sqrt(gamma * gamma + Matrix::dot(x,x));
    x *= -1/normQ;
    gamma /= normQ;

    // Returning the quaternion
    (*quat)(1) = gamma;
    (*quat)(2) = x(1);
    (*quat)(3) = x(2);
    (*quat)(4) = x(3);
}