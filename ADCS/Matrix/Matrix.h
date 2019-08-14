/**
 * @file   Matrix.h
 * @defgroup MatrixGr Linear Algebra
 * @version 2.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * 
 * @brief
 * A library for 2D matrix linear algebra operations
 * 
 * @details
 * # Description
 * This library implement the most of the algebra of the 2D matrices (n x m).
 * 
 * The size and shape of the matrix is arbitrary and can be changed
 * during the execution of the program.
 * 
 * This library implements the following operations regarding matrices
 * - Addition/Substraction between matrices and its neutral (the Zeros matrix)
 * - Multiplication between matrices and its neutral (the Eye matrix)
 * - Multiplication between matrices and scalar
 * - Equality check between two Matrices
 * - Transpose and Inverse
 * - Determinant and Trace
 * - Dot product (or scalar product) and norm for Vectors (1xn or nx1 matrices)
 * - Cross product for 3x1 vectors
 * - Shape transform operations
 * 
 * Adapted from Ernesto Palacios Matrix library, 
 * https://os.mbed.com/users/Yo_Robot/code/Matrix/
 * 
 * @see Matrix
 * 
 * # Example code
 * 
 * @see Matrix.test.cpp
 * 
 * # Dependencies
 * This library depends on <std::cmath> and <std::vector> to provide
 * an matrix of arbitrary size and the required mathematical operations
 * 
 * If printing matrices and error message is required, then you can just include
 * "mbed.h" before including "Matrix.h" and it should activate the printf. 
 * Otherwise, define MATRIX_USE_PRINTF.
 * 
 * @attention This library uses float only (NOT double) and therefore
 * expect 6 to 7 significant figures
 * 
 * @warning When using Mbed studio, the absence of "mbed.h" makes the IDE
 * issue two errors that do not affect compilation.
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

#ifndef MATRIX_H
#define MATRIX_H

#include <cmath>
#include <vector>

#define MATRIX_USE_PRINTF // Comment this line to remove Mbed dependency

/**
 * @ingroup MatrixGr
 * @brief
 * A library for 2D matrix linear algebra operations
 * 
 * @class Matrix
 * 
 * @see Matrix.h
 * @see Matrix
 * @nosubgrouping
 */
class Matrix{
public:
///@name Constructors
    /**
     * @brief Default constructor
     */
    Matrix();

    /**
     * @brief Empty matrix constructor
     * @param Rows The number of rows of the matrix
     * @param Cols The number of columns of the matrix
     * @return A matrix of the desired size
     */
    Matrix( int Rows, int Cols );

    /**
     * @brief Copy constructor
     * @param base The matrix to copy
     */
    Matrix( const Matrix& base );

    /**
     * @brief Creates a matrix of a given size filled with the given coefficients
     * @param Rows The number of rows of the matrix
     * @param Cols The number of columns of the matrix
     * @param coef The coefficients to place in the matrix
     */
    Matrix( int Rows, int Cols , float * coef );

    /**
     * @brief Creates a square identity matrix of a given size
     * @param size The size of the matrix
     * @return The identity matrix
     */
    static Matrix eye(int size);

    /**
     * @brief Creates a matrix of a given size filled with ones
     * @param rows The number of rows of the matrix
     * @param cols The number of columns of the matrix
     * @return A matrix filled with ones
     */
    static Matrix ones(int rows, int cols);

    /**
     * @brief Creates a matrix of a given size filled with zeros
     * @param rows The number of rows of the matrix
     * @param cols The number of columns of the matrix
     * @return A matrix filled with zeros
     */
    static Matrix zeros(int rows, int cols);

    /**
     * @brief
     * Creates a diagonal (quare) matrix with the given coefficients
     * @param size The size of the matrix
     * @param coefs The array containing the coefficients to place on the diagonal
     */
    static Matrix diag(int size, float *coefs);

///@name Operators
    /**
     * @brief
     * Subindex for Matrix elements assignation. (INDEX STARTS AT 1)
     * @param row
     * @param col
     * @return pointer to the element.
     */
    float& operator() ( int row, int col );

    /**
     * @brief
     * Subindex for Matrix elements. (INDEX STARTS AT 1)
     * @param row
     * @param col
     * @return the element.
     */
    float  operator() ( int row, int col ) const;

    /**
     * @brief
     * Subindex for Vector elements assignation. (INDEX STARTS AT 1)
     * @param index
     * @return pointer to the element.
     */
    float& operator() ( int index );

    /**
     * @brief
     * Subindex for Vector elements. (INDEX STARTS AT 1)
     * @param index
     * @return the element.
     */
    float  operator() ( int index ) const;

    /**
     * @brief
     * Overwrites all data. To be used Carefully!
     * @param rightM The matrix to copy
     * @return The reference to the new matrix
     */
    Matrix& operator = ( const Matrix& rightM );

    /**
     * @brief
     * Overload opeartor for the compare Matrices
     * @param leftM The left hand side matrix of the comparison
     * @param rightM The right hand side matrix of the comparison
     * @return Boolean 'false' if different.
     */
    friend bool operator == ( const Matrix& leftM, const Matrix& rightM );

    /**
     * @brief
     * Overload opeartor for the compare Matrices
     * @param leftM The left hand side matrix of the comparison
     * @param rightM The right hand side matrix of the comparison
     * @return Boolean 'true' if different
     */
    friend bool operator != ( const Matrix& leftM, const Matrix& rightM );

    /**
     * @brief
     * Overload Compound increase.
     * @param leftM The matrix that receive the addition
     * @param rightM The matrix to add
     * @return A new Matrix to be assigned to itself.
     */
    friend Matrix& operator += ( Matrix& leftM, const Matrix& rightM );

    /**
     * @brief
     * Overload Compound decrease.
     * @param leftM The matrix that receive the substraction
     * @param rightM The matrix to substract
     * @return A new Matrix to be assigned to itself
     */
    friend Matrix& operator -= ( Matrix& leftM, const Matrix& rightM );

    /**
     * @brief
     * Overload Compound CrossProduct Matrix operation.
     * @param leftM The matrix that receive the multiplication
     * @param rightM The multiplying matrix
     * @return A new Matrix to be assigned to itself
     */
    friend Matrix& operator *=( Matrix& leftM, const Matrix& rightM );

    /** @brief
     * Overload Compund Element-by-element scalar multiplication.
     * @param leftM The matrix that receive the multiplication
     * @param number The multiplying scalar
     * @return A new Matrix to be assigned to itself
     */
    friend Matrix& operator *=( Matrix& leftM, float number );

    /** @brief
     * Overload Compund Element-by-element scalar division.
     * @param leftM The matrix that receive the multiplication
     * @param number The dividing scalar
     * @return A new Matrix to be assigned to itself
     */
    friend Matrix& operator /=( Matrix& leftM, float number );

    /**
     * @brief
     * All elements in matrix are multiplied by (-1).
     * @return A new Matrix object with inverted values.
     */
    const Matrix operator -();

    /**
     * @brief
     * Overload Compound add with scalar.
     * Because the '=' operator checks for self Assign, no extra operations
     * are needed.
     * @param leftM The matrix that receive the addition
     * @param number The scalar to add
     * @return Same Matrix to self Assign.
     */
    friend const Matrix operator +=( Matrix& leftM, float number );

    /**
     * @brief
     * Compound substract with scalar.
     * @param leftM The matrix that receive the substraction
     * @param number The number to substract
     * @return Same matrix to self Assign.
     */
    friend const Matrix operator -=( Matrix& leftM, float number );

    /**
     * @brief
     * Adds two matrices of the same dimensions, element-by-element.
     * If diferent dimensions -> ERROR.
     * @param leftM The left hand side matrix of the addition
     * @param rightM The right hand side matrix of the addition
     * @return A new object Matrix with the result.
     */
    friend const Matrix operator +( const Matrix& leftM, const Matrix& rightM);

    /**
     * @brief
     * Adds the given nomber to each element of matrix.
     * Mimic MATLAB operation.
     * @param leftM The left hand side matrix of the addition
     * @param number The scalar to add
     * @return A new matrix object with the result.
     */
    friend const Matrix operator +( const Matrix& leftM, float number );

    /**
     * @brief
     * Adds the given number to each element in Matrix.
     * @param leftM The left hand side matrix of the addition
     * @param number The scalar to add
     * @return A new Matrix object with the result.
     */
    friend const Matrix operator +( float number, const Matrix& leftM );


    /**
     * @brief
     * Substracts two matrices of the same size, element-by-element.
     * If different dimensions -> ERROR.
     * @param leftM The left hand side matrix of the substraction
     * @param rightM The right hand side matrix of the substraction
     * @return  A new object Matrix with the result.
     */
    friend const Matrix operator -( const Matrix& leftM, const Matrix& rightM );


    /**
     * @brief
     * Substracts each element in Matrix by number.
     * @param leftM The left hand side matrix of the substraction
     * @param number The scalar to substract
     * @return A new matrix object with the result.
     */
    friend const Matrix operator -( const Matrix& leftM, float number );


    /**
     * @brief
     * Substracts each element in Matrix by number
     * @param leftM The left hand side matrix of the substraction
     * @param number The scalar to substract
     * @return A new matrix object with the result.
     */
    friend const Matrix operator -( float number, const Matrix& leftM );


    /**
     * @brief
     * Preforms Crossproduct between two matrices.
     * @param leftM The left hand side matrix of the substraction
     * @param rightM The right hand side matrix of the substraction
     * @return A new matrix with the resultof the multiplication
     */
    friend const Matrix operator *( const Matrix& leftM, const Matrix& rightM );


    /**
     * @brief
     * Multiplies a scalar number with each element on Matrix.
     * @param leftM The left hand side matrix of the multiplication
     * @param number The multiplying scalar
     * @return A new matrix with the resultof the multiplication.
     */
    friend const Matrix operator *( const Matrix& leftM, float number );

    /**
     * @brief
     * Divide each element on Matrix by a scalar
     * @param leftM The left hand side matrix of the multiplication
     * @param number The dividing scalar
     * @return A new matrix with the resultof the multiplication.
     */
    friend const Matrix operator /( const Matrix& leftM, float number );

    /**
     * @brief
     * Multiplies a scalar number with each element on Matrix.
     * @param leftM The left hand side matrix of the multiplication
     * @param number The multiplying scalar
     * @return A new matrix with the resultof the multiplication
     */
    friend const Matrix operator *( float number, const Matrix& leftM );


    /**
     * @brief
     * Inputs numbers into a Matrix, the matrix needs to be costructed as
     * Matrix( _nRows, _nCols ).
     * This does NOT work on an only declared Matrix such as:
     * Matrix obj;
     * obj << 5; //Error
     * @param leftM The matrix that will receive the elements
     * @param number The element to add at the end of the matrix
     * @return The reference to the filled matrix
     */
    friend Matrix& operator <<( Matrix& leftM, float number );

///@name Matrix checks
    /**
     * @brief
     * Returns TRUE if the matrix is zero, FALSE otherwhise
     */
    bool isZero() const;


    /**
     * @brief
     * Determines weather a Matrix is a Single Column or Row.
     */
    bool isVector() const;

    /**
     * @brief
     * Returns true if the matrix is square
     */
    bool isSquare() const;

///@name Matrix shape Methods
    /**
     * @brief
     * Shatters the matrix into a single Row Vector.
     * Important: Returns NEW matrix, does no modify existing one.
     * @param Mat The matrix to compact in a One Row Vector
     * @return The Row Vector containing all the elements
     */
    static const Matrix ToPackedVector( const Matrix& Mat );

    /** 
     * @brief
     * Invoking this static method will increase a Row in Mat in the desired
     * position.
     * The current Row will be moved down to allocate space, and all elements will
     * be initialized to zero in the new row.
     * @param Mat Matrix in wich to insert a Row (INDEX STARTS AT 1)
     * @param index Position where to insert the row (index start at one)
     */
    static void AddRow( Matrix& Mat, int index );

    /**
     * @brief
     * Adds to Receip a new Row from another Matrix in desired index.
     * Must be same size.
     * The Row matrix must be SingleRow Matrix, you can use ExportRow
     * to extract a Row from another Matrix.
     * @param Receip Matrix to be Modified.
     * @param Row Row to be added.
     * @param index position in wich to be added, _nRow + 1 for last position. (INDEX STARTS AT 1)
     */
    static void AddRow( Matrix& Receip, const Matrix& Row, int index );

    /** 
     * @brief
     * Invoking this static method will increase a Column in Matrix in the
     * desired Position.
     * @param Mat Matrix in wich to insert a Column
     * @param index Position where to insert the column (INDEX STARTS AT 1)
     */
    static void AddCol( Matrix& Mat, int index );

    /**
     * @brief
     * This will copy a Column Matrix into Receip in desired Position,
     * Must be same size.
     * The Col Matrix must be a SingleCol Matrix, you can use ExportCol
     * to extract a Column from another Matrix.
     * @param Receip Matrix to be modified.
     * @param Col Data to be copied.
     * @param index Postion in Receip Matrix . (INDEX STARTS AT 1)
     */
    static void AddCol( Matrix& Receip, const Matrix& Col, int index  );

    /**
     * @brief
     * Static Function Deletes Row from Matrix, Static to prevent missuse
     * @param Mat Matrix to delete Row from
     * @param Row Position of the row to be deleted (INDEX STARTS AT 1)
     */
    static void DeleteRow( Matrix& Mat, int Row );

    /**
     * @brief
     * Static Function Deletes Column from Matrix, it's Static to prevent
     * missuse.
     * Print error and does nothing if out of limits.
     * @param Mat Matrix to delete from.
     * @param Col Position of the column to be deleted (INDEX STARTS AT 1)
     */
    static void DeleteCol( Matrix& Mat, int Col );

    /**
     * @brief
     * This method extracts a Row from a Matrix and Saves it in Mat.
     * If Row is out of the parameters it does nothing, but prints a warning.
     * @param row Position of the row to export (INDEX STARTS AT 1)
     * @param Mat Matrix to extract from.
     * @return New Row Matrix.
     */
    static const Matrix ExportRow( const Matrix& Mat, int row );

    /**
     * @brief
     * This method extracts a Column from a Matrix and returns the Column
     * as a new Matrix.
     * If Row is out of the parameters, it does nothing and prints a warning.
     * @param col Position of the column to export (INDEX STARTS AT 1)
     * @param Mat Matrix to extract from.
     * @return New Row Matrix.
     */
    static const Matrix ExportCol( const Matrix& Mat, int col );

    /**
     * @brief
     * This function resizes the Matrix to fit new data or cropped it,
     * operator << can overwrite entire Matrix.
    *
     * @param Rows New Number of Rows
     * @param Cols New numbler of columns
     */
    void Resize( int Rows, int Cols );

    /**
     * @brief
     * Makes all values on Matrix object zero.
     * Also make posible use the '<<' operator to add elements and keep
     * track of last element added.
     */
    void Clear();

    /**
     * @brief
     * Assigns a float number to the matrix in a specified position
     * Index starts at [1][1].
    *
     * @param number   Number to be set
     * @param Row      Row of Matrix (INDEX STARTS AT 1)
     * @param Col      Column of Matrix (INDEX STARTS AT 1)
     */
    void add( int Row, int Col, float number );

    /**
     * @brief
     * Returns the sum of every cell in the Matrix.
     * @return The sum of all elements in the matrix
     */
    float sum() const;

///@name Getters and Setters
    /**
     * @brief
     * Return the number in position [Row],[Col] of the storage vector
     * @attention This method uses indexes that start at zero and NOT one like other functions
     * @param Row = number of row in matrix  (INDEX STARTS AT ZERO)
     * @param Col = number of Col in matrix  (INDEX STARTS AT ZERO)
     * @return Num = float number in matrix
     */
    float getNumber( int Row, int Col ) const;

    /**
     * @brief
     * Return the coefficients of the matrix in a linear array
     * @param coef The array where to store the coefficients
     */
    void getCoef(float *coef) const;

    /**
     * @brief
     * Retuns the number of Columns in Matrix, index starts at 1.
     */
    int  getCols() const;

    /**
     * @brief
     * Retruns the number of Rows in Matrix, index starts at 1.
     */
    int  getRows() const;

    /**
     * @brief Provide the size of the matrix
     * @return The size of the matrix
     */
    int size();

    /**
     * @brief
     * Prints the matrix if MATRIX_USE_PRINTF has been defined
     */
    void print() const;

///@name Linear algebra Methods
    /**
     * @brief
     * Transposes Matrix, return new Object.
     * @return Transposed Matrix
     */
    Matrix Transpose() const;

    /**
     * @brief
     * Calculate the inverse of a [n,n] Matrix BUT you check first if 
     * the determinant is != 0, Same matrix will be return if Det( Mat ) == 0.
     * @return Matrix Inverse
     */
    Matrix Inv() const;

    /**
     * @brief
     * Calculates an approximation of the inverse by using a Taylor expansion
     * 
     * From DOI: 10.1109/LCSYS.2018.2854238
     * 
     * If M = A + B
     * 
     * M^-1 ~= A^-1 - A^-1*B*A^-1 + A^-1*B*A^-1*B*A^-1 - ...
     * 
     * @param order The order up to which perform the Taylor expansion
     * @return The inverse of the matrix
     */
    Matrix TaylorInv(int order) const;

    /**
     * @brief
     * Returns the dot Product of any two same leght vectors.
     * In this case a vector is defined as a [1,n] or [n,1] Matrix.
     * Very Flexible Function.
     * @param leftM First Vector
     * @param rightM Second Vector
     * @return Dot Product or Scalar Product.
     */    
    static float dot( const Matrix& leftM, const Matrix& rightM );

    /**
     * @brief Calculates the determinant of a Matrix.
     * @return the determinant.
     */
    float det() const;

    /**
     * @brief
     * Returns the trace of the matrix
     * @return the trace.
     */
    float trace();

    /**
     * @brief Compute the norm of an (n x 1) or (1 x n) vector
     * @return The norm of the vector
     */
    float norm() const;

    /**
     * @brief Compute the cross product of two (3x1) vectors
     * @param leftM The left hand side vector
     * @param rightM The right hand side vector
     * @return The cross product of the two vectors
     */
    static Matrix cross(const Matrix& leftM, const Matrix& rightM);

    /**
     * @brief
     * Compute the quaternion multiplication of the two given vectors
     * @param leftM The left hand side quaternion
     * @param rightM The right hand side quaternion
     * @return The quaternion resulting from the multiplication
     */
    static Matrix quatmul(const Matrix& leftM, const Matrix& rightM);

    /**
     * @brief
     * Compute the quaternion inverse
     * @param leftM The quaternion to invert
     * @return The inverted quaternion
     */
    static Matrix quatInv(const Matrix& leftM);

    /**
     * @brief
     * Compute the quaternion conjugate
     * @param leftM The quaternion to conjugate
     * @return The conjugate quaternion
     */
    static Matrix quatConj(const Matrix& leftM);

///@name Kinematics Methods
    /**
     * @brief
     * Calculates the Rotation Matrix Transform along 'x' axis in radians.
     * @param radians rotation angle in rad.
     * @return Rotation Matrix[4,4] along 'x' axis.
     */
    static Matrix RotX( float radians );

    /**
     * @brief
     * Calculates the Rotation Matrix Transform along 'y' axis in radians.
     * @param radians rotation angle in rad.
     * @return Rotation Matrix[4,4] along 'y' axis.
     */
    static Matrix RotY( float radians );

    /**
     * @brief
     * Calculates the Rotation Matrix Transform along 'z' axis in radians.
     * @param radians rotation angle in rad.
     * @return Rotation Matrix[4,4] along 'z' axis.
     */
    static Matrix RotZ( float radians );

    /**
     * @brief
     * Create the 3-2-1 (or Z-Y-X) direction cosine Matrix
     * @param yaw The yaw/heading angle (psi on z)
     * @param pitch The pitch/attitude angle (theta on y)
     * @param roll The roll/bank angle (phi on x)
     * @return The direction cosine matrix
     */
    static Matrix Rot321(float roll, float pitch, float yaw);

    /**
     * @brief
     * Create the 3-2-1 (or Z-Y-X) cosine direction Matrix
     * @param euler The euler angles [roll, pitch, yaw] or [phi, theta, psi]
     * @return The direction cosine matrix
     */
    static Matrix Rot321(Matrix euler);

    /**
     * @brief
     * Calculates the Translation Matrix to coordenates (x' y' z').
     * @param x axis translation
     * @param y axis translation
     * @param z axis translation
     * @return Translation Matrix[4,4] x'y'z'.
     */
    static Matrix Transl( float x, float y, float z );

    /**
     * @brief
     * Converts a rotation quaternion to a 3-2-1 rotation matrix (Z->Y->X)
     * @param quat The rotation quaternion to convert [x,y,z,eta] with eta the salar part
     * @return The rotation matrix (Z->Y->X) in a 3x3 matrix
     */
    static Matrix quat2rot(Matrix quat);

    /**
     * @brief
     * Converts a rotation quaternion in Euler angles (3-2-1 convention)
     * @param quat The rotation quaternion [x,y,z,eta] with eta the scalar part
     * @return The euler angles [yaw,pitch,roll]=[phi,theta,psi]
     */
    static Matrix quat2euler(Matrix quat);

    /**
     * @brief
     * Converts Euler angle to rotation quaternion (3-2-1 convention)
     * @param euler Euler angles vector [yaw,pitch,roll]=[phi,theta,psi]
     * @return The rotation quaternion [eta,x,y,z] with eta the scalar part
     */
    static Matrix euler2quat(Matrix euler);

    /**
     * @brief
     * Converts Euler angles to 1-2-3 rotation matrix (X->Y->Z)
     * @param euler An array to hold the euler angles [yaw,pitch,roll]=[phi,theta,psi]
     * @return The rotation matrix (X->Y->Z)
     */
    static Matrix euler2rot123(Matrix euler);

    /**
     * @brief
     * Converts Euler angles to 3-2-1 rotation matrix (Z->Y->X)
     * @param euler An array to hold the euler angles [yaw,pitch,roll]=[phi,theta,psi]
     * @return The rotation matrix (Z->Y->X)
     */
    static Matrix euler2rot(Matrix euler);

    /**
     * @brief
     * Converts 3-2-1 rotation matrix (Z->Y->X) to Euler angles (3-2-1 convention)
     * @param rot The array where to store the rotation matrix (Z->Y->X)
     * @return The euler angles [yaw,pitch,roll]=[phi,theta,psi]
     */
    static Matrix rot2euler(Matrix rot);

    /**
     * @brief
     * Converts 3-2-1 rotation matrix (Z->Y->X) to rotation quaternion
     * @param rot The array where to store the rotation matrix (Z->Y->X)
     * @return The rotation quaternion [x,y,z,eta] with eta the salar part
     */
    static Matrix rot2quat(Matrix rot);

private:
    /** 2-D Vector Array */
    std::vector < std::vector<float> > _matrix;

    /** Number of Rows in Matrix */
    int _nRows;

    /** Number of Columns in Matrix */
    int _nCols;

    /** Last Element Row position in Matrix */
    int _pRow;

    /** Last Element Col position in Matrix */
    int _pCol;

}; // Matrix

static float NullCoef;
extern float NullCoef;

#endif    // MATRIX_H 

