/*******************************************************************************
 * @file SimpleMatrix.h
 * @version 1.0
 * @date 2019
 * @author Remy CHATEL
 * 
 *******************************************************************************
 * @section License
 * 
 * <b>(C) Copyright 2019 Remy CHATEL </b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/
 
#ifndef SIMPLE_MATRIX_H
#define SIMPLE_MATRIX_H
#include <cmath>

/**
 * @namespace SimpleMatrix
 *
 * @brief A simple and lightweight library for 3-elements vectors and 3x3 elements
 * Matrices intended for embeded C++.
 * 
 * # Description
 * 
 * Let A and B be 3x3 matrices, c and d be 3x1 vectors and e a float.
 * 
 * This library implement the following methods
 * 
 * - A + B (and A += B)
 * - A - B (and A -= B)
 * - A * B (and A* = B)
 * - c * A (and c *= A)
 * - A * c
 * - e * A
 * - A * e (and A *= e)
 * - A / e (and A /= e)
 * - det(A) (the determinant)
 * - tr(A) (the trace)
 * - transpose(A) (the transpose)
 * - adj(A) (the classical adjoint or adjugate)
 * - c + d (and c += d)
 * - c - d (and c -= d)
 * - c * e (and c *= e)
 * - e * c
 * - c / e (and c /= e)
 * - c . d (dot or scalar product transpose(c)*d)
 * - c x d (cross product)
 * - c * transpose(d)
 * - norm(c)
 * - normalize(c) (c /= norm(c))
 * 
 * # Dependencies and data type
 * 
 * @attention This library uses float (32-bits) and not double (64-bits)
 * to make best use of the Floating Point Unit of 32-bits microcontrollers.
 * 
 * @attention This library depends on `<cmath>` in order to perform the square root
 * for the norm. If the norm is not used, `include <cmath>`, norm() and
 * normalize() functions can be removed (or commented out).
 */
namespace SimpleMatrix{

/**
 * @class Vector
 * @brief A 3 element Vector class
 *
 * @details This class supports addition/substraction of two vectors, 
 * multiplication/division by a float, dot and cross products, norm
 * and vector times vector multiplication.
 */
class Vector{
public:

    /**
     * @brief Standard constructor of a Vector
     */
    Vector();

    /**
     * @brief Array-based constructor of a Vector
     * @param coeffs The 3-elements array with the initial coefficients
     */
    Vector(float coeffs[3]);

    /**
     * @brief Copy constructor
     * @param a The vector to copy
     */
    Vector(Vector *a);

    /**
     * @brief Standard destructor of a Vector
     */
    ~Vector(void);

    /**
     * @brief Coefficient GET method
     * @param coeffs The array in which to place the coefficients
     */
    void getCoef(float coeffs[3]);

    /** Single Coefficient GET method
     * @param id The coefficient's position
     * @return The value of the coefficient stored at the given position
     */
    float getCoef(int id);

    /**
     * @brief Coefficient SET method
     * @param coeff The array of coefficient to place in the vector
     */
    void setCoef(float *coeff);

    /**
     * @brief Single Coefficient SET method
     * @param id The position of the coefficient to modify
     * @param coeff The new value of the coefficient
     */
    void setCoef(int id, float coeff);

    /**
     * @brief Overload of += operator
     * @param b The vector to add
     */
    void operator+=(Vector b);

    /**
     * @brief Overload of + operator
     * @param b The vector to add
     * @return The sum of the two vectors
     */
    friend Vector operator+(Vector lhs, const Vector& rhs){
        lhs += rhs;
        return lhs;
    }

    /**
     * @brief Overload of -= operator
     * @param b The vector to add
     */
    void operator-=(Vector b);
    
    /**
     * @brief Overload of - operator
     * @param b The vector to substract
     * @return The substraction of the two vectors
     */
    friend Vector operator-(Vector lhs, const Vector& rhs){
        lhs -= rhs;
        return lhs;
    }

    /**
     * @brief Overload of *= operator for float
     * @param a The multiplying float
     */
    void operator*=(float a);

    /**
     * @brief Overload of * operator for float on the right
     * @param a The multiplying float
     * @return The vector multiplied by the float
     */
    friend Vector operator*(Vector lhs, const float& rhs){
        lhs *= rhs;
        return lhs;
    }

    /**
     * @brief Overload of * operator for float on the left
     * @param a The multiplying float
     * @return The vector multiplied by the float
     */
    friend Vector operator*(const float& lhs, Vector rhs){
        rhs *= lhs;
        return rhs;
    }

    /**
     * @brief Overload of /= operator for float
     * @param a The dividing float
     */
    Vector& operator/=(const float& a);

    /**
     * @brief Overload of / operator for float
     * @param a The dividing float
     * @return The Vector divided by the float
     */
    friend Vector operator/(Vector lhs, const float& rhs){
        lhs /= rhs;
        return lhs;
    }


    /**
     * @brief The dot (or scalar) product
     * @param b The second vector
     * @return The dot product of the two vectors
     */
    float dot(Vector b);

    /**
     * @brief The cross product
     * @param b The second vector
     * @return The cross product of the two vectors
     */
    Vector cross(Vector b);

    /**
     * @brief The norm of the vector
     * @return The norm of the vector
     */
    float norm();

    /**
     * @brief The normalize the vector
     */
    void normalize();

private:
    /**
     * The coefficients of the Vector
     */
    float coef[3];
};

/**
 * @class Matrix
 * @brief A 3x3 element Matrix class
 *
 * @details This class supports addition/substraction of two matrices, 
 * multiplication/division by a float, multiplication wiht matrices and
 * vectors, transpose, trace, adjugate and determinant operators.
 */
class Matrix{
public:
    /**
     * @brief Standard constructor for a Matrix
     */
    Matrix();

    /**
     * @brief Array-based constructor for a Matrix
     * @param coeffs The coefficient to initialize the matrix
     */
    Matrix(float coeffs[9]);

    /**
     * @brief Copy constructor for a Matrix
     * @param a The pointer to the matrix to copy
     */
    Matrix(Matrix *a);

    /**
     * @brief Deconstructor for the matrix
     */
    ~Matrix(void);

    /**
     * @brief Coeffient GET method
     * @param coeffs The array where to store the coefficients
     */
    void getCoef(float coeffs[9]);

    /**
     * @brief Coeffient GET method
     * @param id The position of the coefficient
     * @return The value of the coefficient
     */
    float getCoef(int id);

    /**
     * @brief Coefficient GET method
     * @param line The line of the coefficient
     * @param col The column of the coefficient
     * @return The value of the coefficient
     */
    float getCoef(int line, int col);

    /**
     * @brief Coeffient SET method
     * @param id The position of the coefficient
     * @param a The new value of the coefficient
     */
    void setCoef(int id, float a);

    /**
     * @brief Coefficient SET method
     * @param line The line of the coefficient
     * @param col The column of the coefficient
     * @param a The new value of the coefficient
     */
    void setCoef(int line, int col, float a);

    /**
     * @brief Coeffient SET method
     * @param coeffs The values of the coefficient
     */
    void setCoef(float *coeffs);

    /**
     * @brief Overload of the += operator for Matrices
     * @param b The matrix to add
     */
    Matrix& operator+=(const Matrix& b);
    
    /**
     * @brief Overload of the + operator for Matrices
     * @param b The matrix to add
     */
   friend Matrix operator+(Matrix lhs, const Matrix& rhs){
        lhs += rhs;
        return lhs;
    }

    /**
     * @brief Overload of the -= operator for Matrices
     * @param b The matrix to substract
     */
    Matrix& operator-=(const Matrix& b);

    /**
     * @brief Overload of the - operator for Matrices
     * @param b The matrix to substract
     */
    friend Matrix operator-(Matrix lhs, const Matrix& rhs){
        lhs -= rhs;
        return lhs;
    }

    /**
     * @brief Overload of the *= operator for Matrices
     * @param b The multiplying matrix
     */
    Matrix& operator*=(Matrix b);

    /**
     * @brief Overload of the * operator for Matrices
     * @param b The multiplying matrix
     */
    friend Matrix operator*(Matrix lhs, Matrix rhs){
        lhs *= rhs;
        return lhs;
    }

    /**
     * @brief Overload of the *= operator to multiply a matrix by a float
     * @param b The multiplying float
     */
    Matrix& operator*=(const float& b);

    /**
     * @brief Overload of the * operator to multiply a matrix by a float
     * from the right side (Matrix * float)
     * @param b The multiplying float
     */
    friend Matrix operator*(Matrix lhs, const float& rhs){
        lhs *= rhs;
        return lhs;
    };

    /**
     * @brief Overload of the * operator to multiply a matrix by a float
     * from the left side (float * Matrix)
     * @param b The multiplying float
     */
    friend Matrix operator*(const float& lhs, Matrix rhs){
        rhs *= lhs;
        return rhs;
    };

    /**
     * @brief Overload of the * operator to multiply a matrix by a vector
     * from the right (Matrix * Vector)
     * @param b The multiplying vector
     */
    Vector operator*(Vector rhs);

    /**
     * @brief Overload of the * operator to multiply a matrix by a vector
     * from the left (Vector * Matrix)
     * @param b The multiplying vector
     */
     friend Vector operator*(Vector lhs, Matrix rhs){
        Vector tmp;
        float vec_coef[3];
        float mat_coef[9];
        lhs.getCoef(vec_coef);
        rhs.getCoef(mat_coef);
        for(int i = 0; i < 3; i++){
            tmp.setCoef(i, mat_coef[i] * vec_coef[0] + mat_coef[3+i] * vec_coef[1] + mat_coef[6+i] * vec_coef[2]);
        }
        return tmp;
     }

    /**
     * @brief Overload of the /= operator to divide a matrix by a float
     * @param b The dividing float
     */
    Matrix& operator/=(const float& b);

    /**
     * @brief Overload of the / operator to divide a matrix by a float
     * @param b The dividing float
     */
    friend Matrix operator/(Matrix lhs, const float& rhs){
        lhs /= rhs;
        return lhs;
    }

    /**
     * @brief The transpose operator
     */
    Matrix transpose();

    /**
     * @brief The adjugate (or classical adjoint) operator
     */
    Matrix adj();

    /**
     * @brief The trace operator
     */
    float tr();

    /**
     * @brief The determinant operator
     */
    float det();

    /**
     * @brief Inverse the Matrix
     */
    Matrix inv();

    /**
    * @brief Overload of * for multiplying vectors with vectors
    * ( a * transpose(b) )
    * @param a The vector to multiply
    * @param b The multiplying vector
    * @return The matrix
    */
    static Matrix vec_mul(Vector lhs, Vector rhs);

private:
    /**
     * The coefficients of the matrix
     */
    float coef[9];
};

} // namespace SimpleMatrix
#endif // SIMPLE_MATRIX_H