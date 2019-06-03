/**
 * @brief  Source Code for the Matrix Class
 * @file   Matrix.h
 * @author Remy CHATEL
 *
 * Last modified June 2019
 * Adapted from Ernesto Palacios Matrix library
 * 
 * https://os.mbed.com/users/Yo_Robot/code/Matrix/
 *
 * Develop Under  GPL v3.0 License
 * http://www.gnu.org/licenses/gpl-3.0.html
 *
 */

#ifndef MATRIX_H
#define MATRIX_H

#include "mbed.h"
#include <cmath>
#include <vector>


class Matrix{
public:
// Constructors
    /**
     * @brief Default constructor
     */
    Matrix();

    /**
     * @brief Empty matrix constructor
     * @param rows The number of rows of the matrix
     * @param cols The number of columns of the matrix
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
     * @param rows The number of rows of the matrix
     * @param cols The number of columns of the matrix
     * @return A matrix filled with the coefficients
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
     * Calculates the Rotation Matrix Transform along 'x' axis in radians.
     * @param radians rotation angle.
     * @return Rotation Matrix[4,4] along 'x' axis.
     */
    static Matrix RotX( float radians );

    /**
     * @brief
     * Calculates the Rotation Matrix Transform along 'y' axis in radians.
     * @param radians rotation angle.
     * @return Rotation Matrix[4,4] along 'y' axis.
     */
    static Matrix RotY( float radians );

    /**
     * @brief
     * Calculates the Rotation Matrix Transform along 'z' axis in radians.
     * @param radians rotation angle.
     * @return Rotation Matrix[4,4] along 'z' axis.
     */
    static Matrix RotZ( float radians );

    /**
     * @brief
     * Calculates the Translation Matrix to coordenates (x' y' z').
     * @param x axis translation
     * @param y axis translation
     * @param z axis translation
     * @return Translation Matrix[4,4] x'y'z'.
     */
    static Matrix Transl( float x, float y, float z );

// Operators
    /**
     * @brief
     * Subindex for Matrix elements assignation.
     * @param row
     * @param col
     * @return pointer to the element.
     */
    float& operator() ( int row, int col );


    /**
     * @brief
    *Subindex for Matrix element.
     * @param row
     * @param col
     * @return the element.
     */
    float  operator() ( int row, int col ) const;



    /**
     * @brief
     * Overwrites all data. To be used Carefully!
     */
    Matrix& operator = ( const Matrix& rightM );


    /**
     * @brief
     * Overload opeartor for the compare Matrices
    *
     * @param rightM
     * @return Boolean 'false' if different.
     */
    friend bool operator == ( const Matrix& leftM, const Matrix& rightM );


    /**
     * @brief
     * Overload opeartor for the compare Matrices
    *
     * @param rightM
     * @return Boolean 'true' if different
     */
    friend bool operator != ( const Matrix& leftM, const Matrix& rightM );


    /**
     * @brief
     * Overload Compound assignment.
     * @param rightM
     * @return A new Matrix to be assigned to itself.
     */
    friend Matrix& operator += ( Matrix& leftM, const Matrix& rightM );


    /**
     * @brief
     * Overload Compound decrease.
     * @param rightM Right hand matrix
     * @return A new Matrix to be assigned to itself
     */
    friend Matrix& operator -= ( Matrix& leftM, const Matrix& rightM );


    /**
     * @brief
     * Overload Compound CrossProduct Matrix operation.
     * @param rightM
     * @return
     */
    friend Matrix& operator *=( Matrix& leftM, const Matrix& rightM );


    /* * @brief
     * Overload Compund Element-by-elemnt scalar multiplication.
     * @param number
     * @return
     */
    friend Matrix& operator *=( Matrix& leftM, float number );



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
     * @return Same Matrix to self Assign.
     */
    friend const Matrix operator +=( Matrix& leftM, float number );


    /**
     * @brief
     * Compound substract with scalar.
     * @return Same matrix to self Assign.
     */
    friend const Matrix operator -=( Matrix& leftM, float number );


    /**
     * @brief
     * Adds two matrices of the same dimensions, element-by-element.
     * If diferent dimensions -> ERROR.
     * @return A new object Matrix with the result.
     */
    friend const Matrix operator +( const Matrix& leftM, const Matrix& rightM);


    /**
     * @brief
     * Adds the given nomber to each element of matrix.
     * Mimic MATLAB operation.
     * @return A new matrix object with the result.
     */
    friend const Matrix operator +( const Matrix& leftM, float number );



    /**
     * @brief
     * Adds the given number to each element in Matrix.
     * @return A new Matrix object with the result.
     */
    friend const Matrix operator +( float number, const Matrix& leftM );


    /**
     * @brief
     * Substracts two matrices of the same size, element-by-element.
     * If different dimensions -> ERROR.
     * @return  A new object Matrix with the result.
     */
    friend const Matrix operator -( const Matrix& leftM, const Matrix& rightM );


    /**
     * @brief
     * Substracts each element in Matrix by number.
     * @return A new matrix object with the result.
     */
    friend const Matrix operator -( const Matrix& leftM, float number );


    /**
     * @brief
     * Substracts each element in Matrix by number
     * @return A new matrix object with the result.
     */
    friend const Matrix operator -( float number, const Matrix& leftM );


    /**
     * @brief
     * Preforms Crossproduct between two matrices.
     * @return
     */
    friend const Matrix operator *( const Matrix& leftM, const Matrix& rightM );


    /**
     * @brief
     * Multiplies a scalar number with each element on Matrix.
     * @return A new object with the result.
     */
    friend const Matrix operator *( const Matrix& leftM, float number );


    /**
     * @brief
     * Multiplies a scalar number with each element on Matrix.
     * @return
     */
    friend const Matrix operator *( float number, const Matrix& leftM );


    /**
     * @brief
     * Inputs numbers into a Matrix, the matrix needs to be costructed as
     * Matrix( _nRows, _nCols ).
     * This does NOT work on an only declared Matrix such as:
     * Matrix obj;
     * obj << 5; //Error
     * @return
     */
    friend Matrix& operator <<( Matrix& leftM, float number );

// Matrix checks
    /**
     * @brief
     * Returns TRUE if the matrix is zero, FALSE otherwhise
     * @param mat: Matrix to be tested
     */
    bool isZero() const;


    /**
     * @brief
     * Determines weather a Matrix is a Single Column or Row.
     */
    bool isVector() const;

// Matrix shape Methods
    /**
     * @brief
     * Shatters the matrix into a single Row Vector.
     * Important: Returns NEW matrix, does no modify existing one.
     */
    static const Matrix ToPackedVector( const Matrix& Mat );

    /** 
     * @brief
     * Invoking this static method will increase a Row in Mat in the desired
     * position.
     * The current Row will be moved down to allocate space, and all elements will
     * be initialized to zero in the new row.
     * @param Mat: Matrix in wich to insert a Row
     * @param Row: Number of row to insert, starts with one, not zero.
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
     * @param index position in wich to be added, _nRow + 1 last position.
     */
    static void AddRow( Matrix& Receip, const Matrix& Row, int index );

    /** 
     * @brief
     * Invoking this static method will increase a Column in Matrix in the
     * desired Position.
     * @param Mat: Matrix in wich to insert a Column
     * @param Col: Number of column, strats with one, not zero.
     */
    static void AddCol( Matrix& Mat, int index );

    /**
     * @brief
     * This will copy a Column Matrix into Receip in desired Position,
     * Must be same size.
     * The Col Matrix must be a SingleCol Matrix, you can use ExportCol
     * to extract a Column from another Matrix.
     * @param Receip Matrix to be modified.
     * @param Column Data to be copied.
     * @param index Postion in Receip Matrix .
     */
    static void AddCol( Matrix& Receip, const Matrix& Col, int index  );

    /**
     * @brief
     * Static Function Deletes Row from Matrix, Static to prevent missuse
     * @param Mat: Matrix to delete Row from
     * @param Row: Number of Row (first Row = 1)
     */
    static void DeleteRow( Matrix& Mat, int Row );

    /**
     * @brief
     * Static Function Deletes Column from Matrix, it's Static to prevent
     * missuse.
     * Print error and does nothing if out of limits.
     * @param Col: Number of Col to delete (first Col = 1)
     * @param Mat: Matrix to delete from.
     */
    static void DeleteCol( Matrix& Mat, int Col );

    /**
     * @brief
     * This method extracts a Row from a Matrix and Saves it in Mat.
     * If Row is out of the parameters it does nothing, but prints a warning.
     * @param Row: number of row to extract elements. this->_nRows.
     * @param Mat: Matrix to extract from.
     * @return New Row Matrix.
     */
    static const Matrix ExportRow( const Matrix& Mat, int row );

    /**
     * @brief
     * This method extracts a Column from a Matrix and returns the Column
     * as a new Matrix.
     * If Row is out of the parameters, it does nothing and prints a warning.
     * @param Col: number of Column to extract elements. this->_nCols.
     * @param Mat: Matrix to extract from.
     * @return New Row Matrix.
     */
    static const Matrix ExportCol( const Matrix& Mat, int col );

    /**
     * @brief
     * This function resizes the Matrix to fit new data or cropped it,
     * operator << can overwrite entire Matrix.
    *
     * @param Rows: New Number of Rows
     * @param Cols: New numbler of columns
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
     * @param number:   Number to be set
     * @param Row:      Row of Matrix
     * @param Col:      Column of Matrix
     */
    void add( int Row, int Col, float number );

    /**
     * @brief
     * Returns the sum of every cell in the Matrix.
     */
    float sum() const;

// Getters and Setters
    /**
     * @brief
     * Return the number in position [Row],[Col]
     * @param Row = number of row in matrix
     * @param Col = number of Col in matrix
     * @return Num = float number in matrix
     */
    float getNumber( int Row, int Col ) const;

    /**
     * @brief
     * Return the coefficients of the matrix in a linear array
     * @param coeffs The array where to store the coefficients
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

// Linear algebra Methods
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
    Matrix Inv();

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
    float det();

    /**
     * @brief
     * Returns the trace of the matrix
     * @return the trace.
     */
    float trace();

    /**
     * @brief Compute the norm of an (n x 1) or (1 x n) vector
     * @param Mat the vector to calculate
     * @return The norm of the vector
     */
    float norm();

    /**
     * @brief Compute the cross product of two (3x1) vectors
     * @param leftM The left hand side vector
     * @param rightM The right hand side vector
     * @return The cross product of the two vectors
     */
    static Matrix cross(const Matrix& leftM, const Matrix& rightM);

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


static Matrix NullMatrix;
static float NullCoef;
extern Matrix NullMatrix;
extern float NullCoef;

#endif    // MATRIX_H 

