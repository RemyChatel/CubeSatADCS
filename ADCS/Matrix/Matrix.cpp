/**
 * @brief  Source Code for the Matrix Class
 * @file   Matrix.cpp
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
#include "Matrix.h"


// Constructors
    Matrix::Matrix() {
        _nCols = 0;
        _nRows = 0;

        _pRow = 0;
        _pCol = 0;

    }

    Matrix::Matrix(int Rows, int Cols): _nRows(Rows), _nCols(Cols) {
        _matrix.resize(_nRows);
        for( int i = 0; i < _nRows; i++ )
            _matrix[i].resize(_nCols);

        _pRow = 0;
        _pCol = 0;

        this->Clear();  //Make all elements zero by default.
    }

    Matrix::Matrix(const Matrix& base) {
        _nCols = base._nCols;
        _nRows = base._nRows;

        _pRow  = base._pRow;
        _pCol  = base._pCol;

        _matrix.resize(_nRows);
        for( int i = 0; i < _nRows; i++ )
            _matrix[i].resize(_nCols);

        for( int i = 0; i < _nRows; i++ )
            for( int j = 0; j < _nCols; j++ )
                _matrix[i][j] = base._matrix[i][j];
    }

    Matrix::Matrix( int Rows, int Cols , float* coef ){
        _nRows = Rows;
        _nCols = Cols;

        _matrix.resize(_nRows);
        for( int i = 0; i < _nRows; i++ )
            _matrix[i].resize(_nCols);

        _pRow = Rows;
        _pCol = Cols;
        int iteration = 0;
        for(int i = 0; i < _nRows; i++){
            for(int j = 0; j < _nCols; j++){
                _matrix[i][j] = coef[iteration];
                iteration++;
            }
        }
    }

    Matrix Matrix::eye(int size){
        Matrix tmp = zeros(size, size);
        for(int i = 0; i < size; i++){
            tmp._matrix[i][i] = 1;
        }
        return tmp;
    }

    Matrix Matrix::ones(int rows, int cols){
        Matrix tmp(rows, cols);
        for(int i = 0; i < rows; i++){
            for(int j = 0; j < cols; j++){
                tmp._matrix[i][j] = 1;
            }
        }
        return tmp;
    }

    Matrix Matrix::zeros(int rows, int cols){
        Matrix tmp(rows, cols);
        for(int i = 0; i < rows; i++){
            for(int j = 0; j < cols; j++){
                tmp._matrix[i][j] = 0;
            }
        }
        return tmp;
    }

// Operators
    float& Matrix::operator ()(int row, int col) {
        //--row; --col;

        if( row >= _nRows || col >= _nCols)
        {
            NullCoef = nanf("");
            return NullCoef;
        }else{
            return _matrix[row][col];
        }
    }

    float Matrix::operator ()(int row, int col) const {
        //--row; --col;

        if( row >= _nRows || col >= _nCols)
        {
            return nanf("");
        }else{
            return _matrix[row][col];
        }
    }

    Matrix& Matrix::operator = ( const Matrix& rightM ) {
        if (this != &rightM )
        {

            _nRows = rightM._nRows;
            _nCols = rightM._nCols;

            _matrix.resize( rightM._nRows );
            for( int i = 0; i < rightM._nRows; i++ )
                _matrix [i].resize(rightM._nCols);

            for( int i = 0; i < _nRows; i++ )
                for( int j = 0; j < _nCols; j++ )
                    _matrix[i][j] = rightM._matrix[i][j];
        }
        return *this;

    }

    const Matrix Matrix::operator -() {
        Matrix result( _nRows, _nCols );

        for( int i = 0; i < _nRows; i++ )
            for( int j = 0; j < _nCols; j++ )
                result._matrix[i][j] = _matrix[i][j] * -1;

        return result;
    }

    bool operator == ( const Matrix& leftM, const Matrix& rightM ) {
        if( leftM._nRows == rightM._nRows  &&  leftM._nCols == rightM._nCols )
        {
            bool equal = false;

            for( int i = 0; i < leftM._nRows; i++ )
                for( int j = 0; j < leftM._nCols; j++ )
                    if( leftM._matrix[i][j] != rightM._matrix[i][j] )
                        equal = equal || true;

            return !equal;

        }else{  return false;  }
    }

    bool operator != ( const Matrix& leftM, const Matrix& rightM ) {
        return !( leftM == rightM );
    }

    Matrix& operator +=( Matrix& leftM, const Matrix& rightM ) {
        if( leftM._nRows == rightM._nRows  &&  leftM._nCols == rightM._nCols )
        {
            for( int i = 0; i < leftM._nRows; i++ )
                for( int j = 0; j < leftM._nCols; j++ )
                    leftM._matrix[i][j] += rightM._matrix[i][j];

            return leftM;

        }else{
            return NullMatrix;
        }
    }

    Matrix& operator -=( Matrix& leftM, const Matrix& rightM ) {
        if( leftM._nRows == rightM._nRows  &&  leftM._nCols == rightM._nCols )
        {
            for( int i = 0; i < leftM._nRows; i++ )
                for( int j = 0; j < leftM._nCols; j++ )
                    leftM._matrix[i][j] -= rightM._matrix[i][j];

            return leftM;

        }else{
            return NullMatrix;
        }
    }

    Matrix& operator *=( Matrix& leftM, const Matrix& rightM ) {
        if( leftM._nCols == rightM._nRows )
        {
            Matrix resultM ( leftM._nRows, rightM._nCols );

            for( int i = 0; i < resultM._nRows; i++ )
                for( int j = 0; j < resultM._nCols; j++ )
                    for( int m = 0; m < rightM._nRows; m++ )
                        resultM._matrix[i][j] += leftM._matrix[i][m] * rightM._matrix[m][j];

            // return resultM;
            leftM = resultM;
            return leftM;
        }else{
            return NullMatrix;
        }
    }

    Matrix& operator *=( Matrix& leftM, float number ) {
        for( int i = 0; i < leftM._nRows; i++ )
                for( int j = 0; j < leftM._nCols; j++ )
                    leftM._matrix[i][j] *= number;

        return leftM;
    }

    const Matrix operator +=( Matrix& leftM, float number ) {
        for( int i = 0; i < leftM._nRows; i++ )
            for( int j = 0; j < leftM._nCols; j++ )
                leftM._matrix[i][j] += number;
        return leftM;
    }

    const Matrix operator -=( Matrix& leftM, float number ) {
        for( int i = 0; i < leftM._nRows; i++ )
            for( int j = 0; j < leftM._nCols; j++ )
                leftM._matrix[i][j] -= number;
        return leftM;
    }

    const Matrix operator +( const Matrix& leftM, const Matrix& rightM) {
        if( leftM._nRows == rightM._nRows  &&  leftM._nCols == rightM._nCols )
        {
            Matrix result( leftM._nRows, leftM._nCols );

            for( int i = 0; i < leftM._nRows; i++ )
            for( int j = 0; j < leftM._nCols; j++ )
                result._matrix[i][j] = leftM._matrix[i][j] + rightM._matrix[i][j];

            return result;

        }else{
            Matrix null;
            return null;
        }
    }

    const Matrix operator +( const Matrix& leftM, float number ) {
        Matrix result( leftM._nRows, leftM._nCols );

        for( int i = 0; i < leftM._nRows; i++ )
            for( int j = 0; j < leftM._nCols; j++ )
                result._matrix[i][j] = leftM._matrix[i][j] + number;

        return result;
    }

    const Matrix operator +( float number, const Matrix& leftM ) {
        return ( leftM + number );
    }

    const Matrix operator -( const Matrix& leftM, const Matrix& rightM ) {
        if( leftM._nRows == rightM._nRows  &&  leftM._nCols == rightM._nCols )
        {
            Matrix result( leftM._nRows, leftM._nCols );

            for( int i = 0; i < leftM._nRows; i++ )
            for( int j = 0; j < leftM._nCols; j++ )
                result._matrix[i][j] = leftM._matrix[i][j] - rightM._matrix[i][j];

            return result;

        }else{
            Matrix null;
            return null;
        }
    }

    const Matrix operator -( const Matrix& leftM, float number ) {
        Matrix result( leftM._nRows, leftM._nCols );

        for( int i = 0; i < leftM._nRows; i++ )
        for( int j = 0; j < leftM._nCols; j++ )
            result._matrix[i][j] = leftM._matrix[i][j] - number;

        return result;
    }

    const Matrix operator -( float number, const Matrix& leftM ) {
        return ( leftM - number );
    }

    const Matrix operator *( const Matrix& leftM, const Matrix& rightM ) {
        if( leftM._nCols == rightM._nRows )
        {
            Matrix resultM ( leftM._nRows, rightM._nCols );
            resultM.Clear();

            for( int i = 0; i < resultM._nRows; i++ )
                for( int j = 0; j < resultM._nCols; j++ )
                    for( int m = 0; m < rightM._nRows; m++ )
                        resultM._matrix[i][j] += leftM._matrix[i][m] * rightM._matrix[m][j];

            return resultM;

        } else {
            Matrix null;
            return null;
        }
    }

    const Matrix operator *( const Matrix& leftM, float number ) {
        Matrix result( leftM._nRows, leftM._nCols );

        for( int i = 0; i < leftM._nRows; i++ )
        for( int j = 0; j < leftM._nCols; j++ )
            result._matrix[i][j] = leftM._matrix[i][j] * number;

        return result;
    }

    const Matrix operator *( float number, const Matrix& leftM ) {
        return ( leftM * number );
    }

    Matrix& operator <<( Matrix& leftM, float number ) {
        if( leftM._pCol == leftM._nCols ) //end of Row
        {
            leftM._pCol = 0;
            leftM._pRow++;
        }
        if( leftM._pRow > leftM._nRows )
        {
            return leftM;

        }else{

            leftM._matrix[ leftM._pRow ][ leftM._pCol ] = number;
            leftM._pCol++;

            return leftM;
        }
    }

// Matrix checks
    bool Matrix::isZero() const {
        bool zero = false;
        for( int i = 0; i < this->_nRows; i++ )
            for( int j = 0; j < this->_nCols; j++ )
                if( _matrix[i][j] != 0 )
                    zero = zero || true;
        return !zero;
    }

    bool Matrix::isVector() const {
        if( _nRows == 1 || _nCols == 1 )
            return true;
        else
            return false;
    }

// Matrix shape Methods
    const Matrix Matrix::ToPackedVector( const Matrix& Mat ) {

        Matrix Crushed( 1, Mat._nRows * Mat._nCols );

        int cont = 0;

        for( int i = 0; i < Mat._nRows; i++ )
            for( int j = 0; j < Mat._nCols; j++ )
            {
                Crushed._matrix[0][cont] = Mat._matrix[i][j];
                cont++;
            }

        Crushed._pRow = Crushed._nRows;
        Crushed._pCol = Crushed._nCols;

        return Crushed;
    }

    void Matrix::AddRow(Matrix& Mat, int index) {
        --index;

        if( index > Mat._nRows + 1)
        {
        }else{

        Mat._nRows++;
        Mat._matrix.resize( Mat._nRows );

        Mat._matrix[ Mat._nRows - 1 ].resize( Mat._nCols );

        for( int i = Mat._nRows - 1; i > index; i-- )
            for( int j = 0; j < Mat._nCols; j++ )
                Mat._matrix[i][j] = Mat._matrix[i - 1][j];

        for( int j = 0; j < Mat._nCols; j++ )
            Mat._matrix[index][j] = 0.0;
        }
    }

    void Matrix::AddRow(Matrix& Receip, const Matrix& Row, int index) {
        Matrix::AddRow( Receip, index );  //Make Room

        --index;
        for( int i = 0; i < Receip._nCols; i++ )
            Receip._matrix[index][i] = Row._matrix[0][i];   //Copy Data.

    }

    void Matrix::AddCol( Matrix& Mat, int index ) {
        --index;

        if( index > Mat._nCols + 1 )
        {

        }else{


                Mat._nCols++;
                for( int i = 0; i < Mat._nRows; i++ )
                    Mat._matrix[i].resize( Mat._nCols );

                for( int i = 0; i < Mat._nRows; i++ )
                    for( int j = Mat._nCols; j > index; j-- )
                        Mat._matrix[i][j] = Mat._matrix[i][j - 1];

                for( int i = 0; i < Mat._nRows; i++ )
                    Mat._matrix[i][index] = 0.0;

        }
    }

    void Matrix::AddCol(Matrix& Receip, const Matrix& Row, int index) {
        Matrix::AddCol( Receip, index ); // Make Rom

        --index;
        for( int i = 0; i < Receip._nRows; i++ )
            Receip._matrix[i][index] = Row._matrix[i][0];   //Copy Data.
    }

    void Matrix::DeleteCol( Matrix& Mat, int Col) {
        --Col; // Because of Column zero.

        if( Col > Mat._nCols )
        {

        } else {

            for( int i = 0; i < Mat._nRows; i++ )
                for( int j = Col; j < Mat._nCols; j++ )
                    Mat._matrix[i][j] = Mat._matrix[i][j+1];

            // If adressing last element of Column,
            // wich no longer exists
            if( Mat._pCol == Mat._nCols )
                Mat._pCol--;

            // Decrease one column
            Mat._nCols--;

            //Erase last Column
            for( int i = 0; i < Mat._nRows; i++ )
                Mat._matrix[i].reserve(Mat._nCols);
        }
    }

    void Matrix::DeleteRow(Matrix& Mat, int Row) {
        --Row;

        if( Row > Mat._nRows )
        {
        }else{

            for( int i = Row; i < Mat._nRows - 1; i++ )

                for( int j = 0; j < Mat._nCols; j++ )
                    Mat._matrix[i][j] = Mat._matrix[i+1][j];
            Mat._nRows--;
            Mat._matrix.resize(Mat._nRows);
        }
    }

    const Matrix Matrix::ExportRow( const Matrix& Mat, int row ) {
        --row;

        if( row > Mat._nRows )
        {
            return NullMatrix;
        } else {

            Matrix SingleRow( 1 , Mat._nCols );
            SingleRow.Clear();

            for( int j = 0; j < Mat._nCols; j++ )
            SingleRow._matrix[0][j] = Mat._matrix[row][j];

            SingleRow._pCol = SingleRow._nCols;
            SingleRow._pRow = 0;

            return SingleRow;
        }
    }

    const Matrix Matrix::ExportCol( const Matrix& Mat, int col ) {
        --col;

        if( col > Mat._nCols )
        {
            return NullMatrix;
        }else{

            Matrix SingleCol( Mat._nRows, 1 );
            for(int i = 0; i < Mat._nRows; i++ )
                SingleCol._matrix[i][0] = Mat._matrix[i][col];

            SingleCol._pCol = 0;
            SingleCol._pRow = SingleCol._nRows;

            return SingleCol;
        }
    }

    void Matrix::Resize( int Rows, int Cols ) {
        _nRows = Rows;  //Decreases one because internally
        _nCols = Cols; // Index starts at zero.

        _matrix.resize( _nRows );

        for( int i = 0; i< _nRows ; i++ )
            _matrix[i].resize(_nCols);

        _pRow = 0; // If matrix is resized the <<
        _pCol = 0; // operator overwrites everything!
    }

    void Matrix::Clear() {
        for( int i = 0; i < _nRows; i++ )
        for( int j = 0; j < _nCols; j++ )
            _matrix[i][j] = 0;

        _pCol = 0;  // New data can be added
        _pRow = 0;
    }

    void Matrix::add(int Row, int Col, float number) {
        --Col; --Row;

        if( Row > _nRows || Col > _nCols )
        {

        }else{
            _matrix[Row][Col] = number;
        }
    }

    float Matrix::sum() const {
        float total = 0;

        for( int i = 0; i < _nRows; i++ )
            for( int j = 0; j < _nCols; j++ )
                total += _matrix[i][j];
        return total;
    }

// Getters and Setters
    float Matrix::getNumber( int Row, int Col ) const
    { return this->_matrix[Row][Col]; }

    void Matrix::getCoef(float *coef) const{
        Matrix tmp;
        this->ToPackedVector(tmp);
        for(int i = 0; i < tmp._nCols; i++){
            coef[i] = tmp._matrix[0][i];
        }
    }

    int Matrix::getRows() const{ return this->_nRows; }

    int Matrix::getCols() const{ return this->_nCols; }

    int Matrix::size(){
        if(_nRows != 0){
            return _matrix.size() * _matrix[0].size();
        } else {
            return 0;
        }
    }

// Linear Algebra Methods
Matrix Matrix::Transpose() const
{
    Matrix result( _nCols, _nRows ); //Transpose Matrix

    for( int i = 0; i < result._nRows; i++ )
        for( int j = 0; j < result._nCols; j++ )
            result._matrix[i][j] = _matrix[j][i];

    return result;
}

Matrix Matrix::Inv()
{
    if( _nRows == _nCols )
    {
        if( _nRows == 2 )   // 2x2 Matrices
        {
            float det = this->det();
            if( det != 0 )
            {
                Matrix Inv(2,2);
                Inv._matrix[0][0] =  _matrix[1][1];
                Inv._matrix[1][0] = -_matrix[1][0];
                Inv._matrix[0][1] = -_matrix[0][1];
                Inv._matrix[1][1] =  _matrix[0][0] ;

                Inv *= 1/det;

                return Inv;

            }else{
                return *this;
            }

        }else{   // nxn Matrices
            float det = this->det();
            if( det!= 0 )
            {
                Matrix tmp( *this ); //
                Matrix SubMat;

                // Matrix of Co-factors
                for( int i = 0; i < _nRows; i++ )
                    for( int j = 0; j < _nCols; j++ )
                    {
                        SubMat = *this ;

                        Matrix::DeleteRow( SubMat, i+1 );
                        Matrix::DeleteCol( SubMat, j+1 );

                        if( (i+j)%2 == 0 )
                            tmp._matrix[i][j] = SubMat.det();
                        else
                            tmp._matrix[i][j] = -SubMat.det();
                    }

                // Adjugate Matrix
                tmp = tmp.Transpose();

                // Inverse Matrix
                tmp = 1/det * tmp;

                return tmp;

            }else{
                return *this;
            }
        }

    }else{
        return NullMatrix;
    }
}

float Matrix::dot(const Matrix& leftM, const Matrix& rightM)
{
    if( leftM.isVector() && rightM.isVector() )
    {
        if( leftM._nRows == 1 )
        {
            if( rightM._nRows == 1 )
            {
                if( leftM._nCols == rightM._nCols )
                {
                    // Calculate ( 1,n )( 1,n )
                    float dotP;
                    Matrix Cross;

                    Cross = leftM * rightM.Transpose();
                    dotP = Cross.sum();

                    return dotP;
                }
            }else{
                if( leftM._nCols == rightM._nRows )
                {
                    // Calculate (1, n)( n, 1 )
                    float dotP;
                    Matrix Cross;

                    Cross = leftM * rightM;
                    dotP = Cross.sum();

                    return dotP;
                }
            }
        }else{
            if( rightM._nRows == 1 )
            {
                if( leftM._nRows == rightM._nCols )
                {
                    // Calculate ( n,1 )( 1,n )
                    float dotP;
                    Matrix Cross;

                    Cross = leftM.Transpose()  * rightM.Transpose();
                    dotP = Cross.sum();

                    return dotP;

                }

            }else{
                if( leftM._nRows == rightM._nRows )
                {
                    // Calculate (n, 1)( n, 1 )
                    float dotP;
                    Matrix Cross;

                    Cross = leftM.Transpose() *  rightM ;
                    dotP = Cross.sum();

                    return dotP;

                }
            }
        }

    }
    return nanf("");
}


float Matrix::det()
{
    if( _nRows == _nCols  )
    {

        if( _nRows == 2 )  // 2x2 Matrix
        {
            float det;
            det = _matrix[0][0] * _matrix[1][1] -
                  _matrix[1][0] * _matrix[0][1];
            return det;
        }
        else if( _nRows == 3 ) // 3x3 Matrix
        {
            Matrix D( *this );  //Copy Initial matrix
            Matrix::AddCol(D, Matrix::ExportCol(*this, 1), 4); //Repeat First Column
            Matrix::AddCol(D, Matrix::ExportCol(*this, 2), 5); //Repeat Second Column

            float det = 0;
            for( int i = 0; i < 3; i++ ){
                det +=   D._matrix[0][i] * D._matrix[1][1+i] * D._matrix[2][2+i]
                       - D._matrix[0][2+i] * D._matrix[1][1+i] * D._matrix[2][i];
            }
            return det;
        } else {

            float part1= 0;
            float part2= 0;

            //Find +/- on First Row
            for( int i = 0; i < _nCols; i++)
            {
                Matrix reduced( *this );           // Copy Original Matrix
                Matrix::DeleteRow( reduced, 1); // Delete First Row

                if( i%2 == 0 ) //Even Rows
                {

                    Matrix::DeleteCol( reduced, i+1);
                    part1 += _matrix[0][i] * reduced.det();
                }
                else  // Odd Rows
                {
                    Matrix::DeleteCol( reduced, i+1);
                    part2 += _matrix[0][i] * reduced.det();
                }
            }
            return part1 - part2; 
        }

    }
    return nanf("");
}

float Matrix::trace(){
    float sum = 0;
    if( _nRows == _nCols  ) {
        for(int i = 0; i < _nRows; i++){
            sum += _matrix[i][i];
        }
        return sum;
    } else {
        return nanf("");
    }
}

float Matrix::norm(){
    if(this->isVector()){
        float sum = 0;
        sum = dot(*this, *this);
        return sqrt(sum);
    } else {
        return nanf("");
    }
}

Matrix Matrix::cross(const Matrix& leftM, const Matrix& rightM){
    Matrix tmp;
    if(!leftM.isVector() || !rightM.isVector()){
        return tmp;
    } else {
        if(leftM._nCols != 1){
            leftM.Transpose();
        }
        if(rightM._nCols != 1){
            rightM.Transpose();
        }
        tmp = Matrix(3,1);
        tmp._matrix[0][0] = leftM._matrix[1][0] * rightM._matrix[2][0] - leftM._matrix[2][0] * rightM._matrix[1][0];
        tmp._matrix[1][0] = leftM._matrix[2][0] * rightM._matrix[0][0] - leftM._matrix[0][0] * rightM._matrix[2][0];
        tmp._matrix[2][0] = leftM._matrix[0][0] * rightM._matrix[1][0] - leftM._matrix[1][0] * rightM._matrix[0][0];
        return tmp;
    }
}