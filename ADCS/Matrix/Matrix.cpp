/**
 * @file   Matrix.cpp
 * @version 2.0
 * @date 2019
 * @author Remy CHATEL
 * @copyright GNU Public License v3.0
 * @brief  Source Code for the Matrix Class
 * 
 * @details
 * # Description
 * 
 * Adapted from Ernesto Palacios Matrix library, 
 * https://os.mbed.com/users/Yo_Robot/code/Matrix/
 * 
 * @see Matrix
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
#include "Matrix.h"
#ifdef MATRIX_USE_PRINTF
    #include "mbed.h"
#endif

// Constructors
    Matrix::Matrix() {
        _nCols = 0;
        _nRows = 0;

        _pRow = 0;
        _pCol = 0;

        _matrix.resize(0);
        _matrix.shrink_to_fit();
    }

    Matrix::Matrix(int Rows, int Cols): _nRows(Rows), _nCols(Cols) {
        _matrix.resize(_nRows);
        for( int i = 0; i < _nRows; i++ ){
            _matrix[i].resize(_nCols);
            _matrix[i].shrink_to_fit();
        }
        _matrix.shrink_to_fit();

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
        for( int i = 0; i < _nRows; i++ ){
            _matrix[i].resize(_nCols);
            _matrix.shrink_to_fit();
        }
        _matrix.shrink_to_fit();

        for( int i = 0; i < _nRows; i++ )
            for( int j = 0; j < _nCols; j++ )
                _matrix[i][j] = base._matrix[i][j];
    }

    Matrix::Matrix( int Rows, int Cols , float* coef ){
        _nRows = Rows;
        _nCols = Cols;

        _matrix.resize(_nRows);
        for( int i = 0; i < _nRows; i++ ){
            _matrix[i].resize(_nCols);
            _matrix[i].shrink_to_fit();
        }
        _matrix.shrink_to_fit();

        _pRow = 0;
        _pCol = 0;
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

    Matrix Matrix::diag(int size, float *coefs){
        Matrix tmp(size,size);
        for(int i = 0; i < size; i++){
            tmp._matrix[i][i] = coefs[i];
        }
        return tmp;
    }

// Operators
    float& Matrix::operator ()(int row, int col) {
        --row; --col;

        if( row >= _nRows || col >= _nCols)
        {
            #ifdef MATRIX_USE_PRINTF
            printf("Error in operator(): Index out of bounds (/!\\ Indexes start at 1\r\n");
            #endif
            NullCoef = nanf("");
            return NullCoef;
        }else{
            return _matrix[row][col];
        }
    }

    float Matrix::operator ()(int row, int col) const {
        --row; --col;

        if( row >= _nRows || col >= _nCols)
        {
            #ifdef MATRIX_USE_PRINTF
            printf("Error in operator(): Index out of bounds (/!\\ Indexes start at 1\r\n");
            #endif
            return nanf("");
        }else{
            return _matrix[row][col];
        }
    }

    float& Matrix::operator ()(int index) {
        --index;

        if(this->isVector()){
            if(index < _nRows){
                return _matrix[index][0];
            } else if (index < _nCols) {
                return _matrix[0][index];
            }
        } else if( index < _nRows || index < _nCols){
            return _matrix[index][index];
        }
        #ifdef MATRIX_USE_PRINTF
        printf("Error in operator(): Index out of bounds (/!\\ Indexes start at 1\r\n");
        #endif
        NullCoef = nanf("");
        return NullCoef;
    }

    float Matrix::operator ()(int index) const {
        --index;

        if(this->isVector()){
            if(index < _nRows){
                return _matrix[index][0];
            } else if (index < _nCols) {
                return _matrix[0][index];
            }
        } else if( index < _nRows || index < _nCols){
            return _matrix[index][index];
        }
        #ifdef MATRIX_USE_PRINTF
        printf("Error in operator(): Index out of bounds (/!\\ Indexes start at 1\r\n");
        #endif
        return nanf("");
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
            #ifdef MATRIX_USE_PRINTF
            printf("Error in operator+=: Dimensions mismatch\r\n");
            #endif
            leftM = Matrix();
            return leftM;
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
            #ifdef MATRIX_USE_PRINTF
            printf("Dimensions mismatch\r\n");
            #endif
            leftM = Matrix();
            return leftM;
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
            #ifdef MATRIX_USE_PRINTF
            printf("Error in operator*=: Dimensions mismatch\r\n");
            #endif
            leftM = Matrix();
            return leftM;
        }
    }

    Matrix& operator *=( Matrix& leftM, float number ) {
        for( int i = 0; i < leftM._nRows; i++ )
                for( int j = 0; j < leftM._nCols; j++ )
                    leftM._matrix[i][j] *= number;

        return leftM;
    }

    Matrix& operator /=( Matrix& leftM, float number ) {
        for( int i = 0; i < leftM._nRows; i++ )
                for( int j = 0; j < leftM._nCols; j++ )
                    leftM._matrix[i][j] /= number;

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
            #ifdef MATRIX_USE_PRINTF
            printf("Error in operator +: Dimensions mismatch\r\n");
            #endif
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
            #ifdef MATRIX_USE_PRINTF
            printf("Error in operator -: Dimensions mismatch\r\n");
            #endif
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
            #ifdef MATRIX_USE_PRINTF
            printf("Error in operator *: Dimensions mismatch\r\n");
            #endif
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
    
    const Matrix operator /( const Matrix& leftM, float number ) {
        Matrix result( leftM._nRows, leftM._nCols );

        for( int i = 0; i < leftM._nRows; i++ )
        for( int j = 0; j < leftM._nCols; j++ )
            result._matrix[i][j] = leftM._matrix[i][j] / number;

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

    bool Matrix::isSquare() const{
        return (_nRows == _nCols);
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

        if( index > Mat._nRows + 1) {
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::AddRow > Index out of bounds /!\\ Indexes start at 1\r\n");
            #endif
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

        if( index > Mat._nCols + 1 ){
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::AddCol > Index out of bounds /!\\ Indexes start at 1\r\n");
            #endif
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

        if( Col > Mat._nCols ){
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::DeleteCol > Index out of bounds /!\\ Indexes start at 1\r\n");
            #endif
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

        if( Row > Mat._nRows ){
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::DeleteRow > Index out of bounds /!\\ Indexes start at 1\r\n");
            #endif
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
        Matrix SingleRow;

        if( row > Mat._nRows )
        {
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::ExportRow > Index out of bounds /!\\ Indexes start at 1\r\n");
            #endif
            return SingleRow;
        } else {
            SingleRow.Resize( 1 , Mat._nCols );
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
        Matrix SingleCol;

        if( col > Mat._nCols ){
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::ExportCol > Index out of bounds /!\\ Indexes start at 1\r\n");
            #endif
            return SingleCol;
        }else{
            SingleCol.Resize( Mat._nRows, 1 );
            for(int i = 0; i < Mat._nRows; i++ )
                SingleCol._matrix[i][0] = Mat._matrix[i][col];

            SingleCol._pCol = 0;
            SingleCol._pRow = SingleCol._nRows;

            return SingleCol;
        }
    }

    void Matrix::Resize( int Rows, int Cols ) {
        _nRows = Rows;
        _nCols = Cols;

        _matrix.resize( _nRows );

        for( int i = 0; i< _nRows ; i++ ){
            _matrix[i].resize(_nCols);
            _matrix[i].shrink_to_fit();
        }
        _matrix.shrink_to_fit();

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

        if( Row > _nRows || Col > _nCols ){
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::add > Index out of bounds /!\\ Indexes start at 1\r\n");
            #endif
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
    float Matrix::getNumber( int Row, int Col ) const {
        if(Row < this->_nRows && Col < this->_nCols){
            return this->_matrix[Row][Col];
        } else {
            #ifdef MATRIX_USE_PRINTF
            printf("Index out of bounds /!\\ Indexes start at 0 for this method\r\n");
            #endif
            return nanf("");
        }
    }

    void Matrix::getCoef(float *coef) const{
        int count = 0;
        for(int i = 0; i < this->_nRows; i++){
            for(int j = 0; j < this->_nCols; j++){
                coef[count] = this->_matrix[i][j];
                count++;
            }
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

    void Matrix::print() const{
        #ifdef MATRIX_USE_PRINTF
        printf("{{");

        for(int i = 0; i < this->_nRows; i++){
            if(i != 0){
                printf(" {");
            }
            for(int j = 0; j < this->_nCols; j++){
                printf("% 7g", this->_matrix[i][j]);
                if(j!=this->_nCols-1){
                    printf(", ");
                }
            }
            if(i==this->_nRows-1){
                    printf("}}\r\n");
                }
            else{
                printf("},\r\n");
            }
        }
        #endif
    }

// Linear Algebra Methods
    Matrix Matrix::Transpose() const {
        Matrix result( _nCols, _nRows ); //Transpose Matrix

        for( int i = 0; i < result._nRows; i++ )
            for( int j = 0; j < result._nCols; j++ )
                result._matrix[i][j] = _matrix[j][i];

        return result;
    }

    Matrix Matrix::Inv() const {
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
                    #ifdef MATRIX_USE_PRINTF
                    printf("Error in Matrix::Inv > Matrix is Singular (in Inv)\r\n");
                    #endif
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
                    #ifdef MATRIX_USE_PRINTF
                    printf("Error in Matrix::Inv > Matrix is Singular\r\n");
                    #endif
                    return *this;
                }
            }

        }else{
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::Inv > Matrix is not square\r\n");
            #endif
            return *this;
        }
    }

    Matrix Matrix::TaylorInv(int order) const{
        Matrix tmp;
        Matrix mul= zeros(this->getRows(), this->getCols());
        if(this->_nCols != this->_nRows){
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::TaylorInv > Matrix is not square\r\n");
            #endif
            return tmp;
        }
        // Extract the diagonal coefficients
        Matrix diag = zeros(this->getRows(), this->getCols());
        for(int i = 0; i < this->getCols(); i++){
            diag._matrix[i][i] = this->_matrix[i][i];
        }
        // Extract the non diagonal coefficients
        Matrix notdiag = *this - diag;

        // Invert the diagonal terms component-wise
        for(int i = 0; i < diag._nRows; i++){
            diag._matrix[i][i] = 1 / diag._matrix[i][i];
        }

        // Taylor expension of the Inverse
        tmp  = zeros(this->getRows(), this->getCols());
        tmp += diag;
        tmp += - diag * notdiag * diag;
        tmp +=   diag * notdiag * diag * notdiag * diag;
        tmp += - diag * notdiag * diag * notdiag * diag * notdiag * diag;

        tmp  = zeros(this->getRows(), this->getCols());
        tmp += diag;
        for(int i = 1; i <= order; i++){
            mul = diag;
            for(int j = 1; j <= i; j++){
                mul *= notdiag * diag;
            }
            tmp += mul * ((i%2==0)?1:-1);
        }

        return tmp;
    }

    float Matrix::dot(const Matrix& leftM, const Matrix& rightM) {
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
        #ifdef MATRIX_USE_PRINTF
        printf("Error in Matrix::dot > Matrix is not a vector\r\n");
        #endif
        return nanf("");
    }


    float Matrix::det() const{
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
        #ifdef MATRIX_USE_PRINTF
        printf("Error in Matrix::det > Matrix is not square\r\n");
        #endif
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
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::trace > Matrix is not square\r\n");
            #endif
            return nanf("");
        }
    }

    float Matrix::norm() const{
        if(this->isVector()){
            float sum = 0;
            sum = dot(*this, *this);
            return sqrt(sum);
        } else {
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::norm > Matrix is not a vector\r\n");
            #endif
            return nanf("");
        }
    }

    Matrix Matrix::cross(const Matrix& leftM, const Matrix& rightM){
        Matrix tmp;
        if(!leftM.isVector() || !rightM.isVector()){
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::cross > Matrix is not a vector\r\n");
            #endif
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

    Matrix Matrix::quatmul(const Matrix& leftM, const Matrix& rightM){
        Matrix tmp;
        if(!leftM.isVector() || !rightM.isVector()){
            #ifdef MATRIX_USE_PRINTF
            printf("Error in Matrix::TaylorInv > Matrix is not a vector\r\n");
            #endif
            return tmp;
        } else {
            if(leftM._nCols != 1){
                leftM.Transpose();
            }
            if(rightM._nCols != 1){
                rightM.Transpose();
            }
            tmp = zeros(4,1);
            tmp(1) = leftM._matrix[0][0]*rightM._matrix[0][0] - leftM._matrix[1][0]*rightM._matrix[1][0] - leftM._matrix[2][0]*rightM._matrix[2][0] - leftM._matrix[3][0]*rightM._matrix[3][0];
            tmp(2) = leftM._matrix[0][0]*rightM._matrix[1][0] + leftM._matrix[1][0]*rightM._matrix[0][0] + leftM._matrix[2][0]*rightM._matrix[3][0] - leftM._matrix[3][0]*rightM._matrix[2][0];
            tmp(3) = leftM._matrix[0][0]*rightM._matrix[2][0] - leftM._matrix[1][0]*rightM._matrix[3][0] + leftM._matrix[2][0]*rightM._matrix[0][0] + leftM._matrix[3][0]*rightM._matrix[1][0];
            tmp(4) = leftM._matrix[0][0]*rightM._matrix[3][0] + leftM._matrix[1][0]*rightM._matrix[2][0] - leftM._matrix[2][0]*rightM._matrix[1][0] + leftM._matrix[3][0]*rightM._matrix[0][0];
            return tmp;
        }
    }

    Matrix Matrix::quatConj(const Matrix& leftM){
        Matrix out = Matrix(leftM);
        out(2) *= -1;
        out(3) *= -1;
        out(4) *= -1;
        return out;
    }

    Matrix Matrix::quatInv(const Matrix& leftM){
        return quatConj(leftM)/leftM.norm();
    }

// Kinematics Methods
    Matrix Matrix::quat2rot(Matrix quat){
        Matrix rot;
        if(quat.isVector()){
            if(quat._nRows == 1){
                quat.Transpose();
            }
            quat *= 1/quat.norm();

            rot.Resize(3, 3);

            float qw = quat._matrix[0][0], qx = quat._matrix[1][0], qy = quat._matrix[2][0], qz = quat._matrix[3][0];
            float sqw = qw*qw;
            float sqx = qx*qx;
            float sqy = qy*qy;
            float sqz = qz*qz;
            rot._matrix[0][0] = ( sqx - sqy - sqz + sqw) ; // since sqw + sqx + sqy + sqz =1/invs*invs
            rot._matrix[1][1] = (-sqx + sqy - sqz + sqw) ;
            rot._matrix[2][2] = (-sqx - sqy + sqz + sqw) ;
            
            float tmp1 = qx*qy;
            float tmp2 = qz*qw;
            rot._matrix[1][0] = 2.0 * (tmp1 + tmp2) ;
            rot._matrix[0][1] = 2.0 * (tmp1 - tmp2) ;
            
            tmp1 = qx*qz;
            tmp2 = qy*qw;
            rot._matrix[2][0] = 2.0 * (tmp1 - tmp2) ;
            rot._matrix[0][2] = 2.0 * (tmp1 + tmp2) ;
            tmp1 = qy*qz;
            tmp2 = qx*qw;
            rot._matrix[2][1]= 2.0 * (tmp1 + tmp2) ;
            rot._matrix[1][2]= 2.0 * (tmp1 - tmp2) ;
            rot.Transpose();
        }
        return rot;
    }

    Matrix Matrix::quat2euler(Matrix quat){
        Matrix euler;
        if(quat.isVector()){
            if(quat._nRows == 1){
                quat.Transpose();
            }
            quat *= 1/quat.norm();
            euler.Resize(3, 1);
            euler._matrix[0][0] = atan2(2 * ( quat._matrix[0][0] * quat._matrix[1][0] + quat._matrix[2][0] * quat._matrix[3][0] ),
                               1 - 2 * ( quat._matrix[1][0] * quat._matrix[1][0] + quat._matrix[2][0] * quat._matrix[2][0] ) );
            euler._matrix[1][0] = asin( 2 * ( quat._matrix[0][0] * quat._matrix[2][0] - quat._matrix[1][0] * quat._matrix[3][0] ) );
            euler._matrix[2][0] = atan2( 2 * ( quat._matrix[0][0] * quat._matrix[3][0] + quat._matrix[1][0] * quat._matrix[2][0] ) ,
                               1 - 2 * ( quat._matrix[2][0] * quat._matrix[2][0] + quat._matrix[3][0] * quat._matrix[3][0] ) );
        }
        return euler;
    }

    Matrix Matrix::euler2quat(Matrix euler){
        // euler = [phi,theta,psi]
        Matrix quat;
        if( euler.isVector()){
            if(euler._nRows == 1){
                euler.Transpose();
            }
            quat.Resize(4, 1);
            /*
            float psi[4]    = {cos(euler._matrix[2][0]/2), 0, 0, sin(euler._matrix[2][0]/2)};
            float theta[4]  = {cos(euler._matrix[1][0]/2), 0, sin(euler._matrix[1][0]/2), 0};
            float phi[4]    = {cos(euler._matrix[0][0]/2), sin(euler._matrix[0][0]/2), 0, 0};
            Matrix rotPsi(4,1, psi), rotTheta(4,1, theta), rotPhi(4,1,phi);
            quat = rotPsi * rotPhi * rotPhi;
            */
            float cy = cos(euler._matrix[2][0] * 0.5);
            float sy = sin(euler._matrix[2][0] * 0.5);
            float cp = cos(euler._matrix[1][0] * 0.5);
            float sp = sin(euler._matrix[1][0] * 0.5);
            float cr = cos(euler._matrix[0][0] * 0.5);
            float sr = sin(euler._matrix[0][0] * 0.5);

            quat._matrix[0][0] = cy * cp * cr + sy * sp * sr;
            quat._matrix[1][0] = cy * cp * sr - sy * sp * cr;
            quat._matrix[2][0] = sy * cp * sr + cy * sp * cr;
            quat._matrix[3][0] = sy * cp * cr - cy * sp * sr;
        }
        return quat;
    }

    Matrix Matrix::euler2rot123(Matrix euler){
        // euler = [phi,theta,psi] 
        Matrix rot;
        if( euler.isVector()){
            if(euler._nRows == 1){
                euler.Transpose();
            }
            rot = eye(3);
            rot = RotZ(euler._matrix[2][0]) * RotY(euler._matrix[1][0]) * RotX(euler._matrix[0][0]);
        }
        return rot;
    }

    Matrix Matrix::euler2rot(Matrix euler){
        // euler = [phi,theta,psi] 
        Matrix rot;
        if( euler.isVector()){
            if(euler._nRows == 1){
                euler.Transpose();
            }
            rot = eye(3);
            rot = RotX(euler._matrix[0][0]) * RotY(euler._matrix[1][0]) * RotZ(euler._matrix[2][0]);
        }
        return rot;
    }

    Matrix Matrix::rot2euler(Matrix rot){
        Matrix euler;
        if(rot.isSquare() && rot.getRows() == 3){
            euler.Resize(3, 1);
            
            float sy = sqrt(rot._matrix[0][0] * rot._matrix[0][0] + rot._matrix[0][0] * rot._matrix[0][0]);
            if(!(sy < 1e-5)){
                euler._matrix[0][0] = atan2(rot._matrix[2][1], rot._matrix[2][2]);
                euler._matrix[1][0] = atan2(-rot._matrix[2][0], sy);
                euler._matrix[2][0] = atan2(rot._matrix[1][0], rot._matrix[0][0]);
            } else {
                euler._matrix[0][0] = atan2(rot._matrix[2][1], rot._matrix[2][2]);
                euler._matrix[1][0] = atan2(-rot._matrix[2][0], sy);
                euler._matrix[2][0] = 0;
            }
        }
        return euler;
    }

    Matrix Matrix::rot2quat(Matrix rot){
        Matrix quat;
        if(rot.isSquare() && rot.getRows() == 3){
            quat.Resize(4, 1);

            quat._matrix[0][0] = sqrt(rot.trace() + 1) / 2;

            if(quat._matrix[0][0] != 0) {
                quat._matrix[1][0] = -( rot._matrix[2][1] - rot._matrix[1][2] ) / (4 * quat._matrix[0][0]);
                quat._matrix[2][0] = -( rot._matrix[0][2] - rot._matrix[2][0] ) / (4 * quat._matrix[0][0]);
                quat._matrix[3][0] = -( rot._matrix[1][0] - rot._matrix[0][1] ) / (4 * quat._matrix[0][0]);
            } else {
                quat._matrix[1][0] = sqrt( ( rot._matrix[0][0] +1 ) / 2);
                quat._matrix[2][0] = sqrt( ( rot._matrix[1][1] +1 ) / 2);
                quat._matrix[3][0] = sqrt( ( rot._matrix[2][2] +1 ) / 2);

                if(fabs(quat._matrix[1][0]) > 0){
                    quat._matrix[1][0] = fabs(quat._matrix[1][0]);
                    quat._matrix[2][0] = fabs(quat._matrix[2][0]) * ( (rot._matrix[0][1]>0)?1:-1 );
                    quat._matrix[3][0] = fabs(quat._matrix[3][0]) * ( (rot._matrix[0][2]>0)?1:-1 );
                } else if(fabs(quat._matrix[2][0])>0) {
                    quat._matrix[1][0] = fabs(quat._matrix[1][0]) * ( (rot._matrix[0][1]>0)?1:-1 );
                    quat._matrix[2][0] = fabs(quat._matrix[2][0]);
                    quat._matrix[3][0] = fabs(quat._matrix[3][0]) * ( (rot._matrix[1][2]>0)?1:-1 );
                } else if(fabs(quat._matrix[3][0])>0) {
                    quat._matrix[1][0] = fabs(quat._matrix[1][0]) * ( (rot._matrix[0][2]>0)?1:-1 );
                    quat._matrix[2][0] = fabs(quat._matrix[2][0]) * ( (rot._matrix[1][2]>0)?1:-1 );
                    quat._matrix[3][0] = fabs(quat._matrix[3][0]);
                } else {
                    quat._matrix[1][0] = 0;
                    quat._matrix[2][0] = 0;
                    quat._matrix[3][0] = 0;
                }
            }
        }
        return quat;
    }

    Matrix Matrix::RotX( float radians ) {
        float cs = cos( radians );
        float sn = sin( radians );
    
        Matrix rotate = eye(3);
        rotate._matrix[1][1] = cs;
        rotate._matrix[2][2] = cs;
        rotate._matrix[2][1] =-sn;
        rotate._matrix[1][2] = sn;
    
        return rotate;
    
    }
 
    Matrix Matrix::RotY( float radians ) {
        float cs = cos( radians );
        float sn = sin( radians );

        Matrix rotate = eye(3);
        rotate._matrix[0][0] = cs;
        rotate._matrix[2][2] = cs;
        rotate._matrix[0][2] =-sn;
        rotate._matrix[2][0] = sn;

        return rotate;
    }
    
    Matrix Matrix::RotZ( float radians ) {
        float cs = cos( radians );
        float sn = sin( radians );

        Matrix rotate = eye(3);
        rotate._matrix[0][0] = cs;
        rotate._matrix[1][1] = cs;
        rotate._matrix[1][0] =-sn;
        rotate._matrix[0][1] = sn;

        return rotate;
    }

    Matrix Matrix::Rot321(float roll, float pitch, float yaw){
        return RotX(roll) * RotY(pitch) * RotZ(yaw);
    }
    
    Matrix Matrix::Rot321(Matrix euler){
        Matrix rot;
        if(euler.isVector()){
            if(euler._nRows == 1){
                euler.Transpose();
            }
            rot = Rot321(euler._matrix[0][0], euler._matrix[1][0], euler._matrix[2][0]);
        }
        return rot;
    }

    Matrix Matrix::Transl( float x, float y, float z ) {
        Matrix Translation = Matrix::eye( 3 );  //Identity Matrix
        Matrix Position( 4, 1 );                   // Position Matrix
    
        Position << x << y << z << 1;            // position @ x,y,z
    
        Matrix::AddRow( Translation, 4 );             // Add Row
        Matrix::AddCol( Translation, Position, 4 );  // Add Position Matrix
    
        return Translation;
    }