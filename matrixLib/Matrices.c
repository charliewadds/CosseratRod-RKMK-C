#include <assert.h>
#include "Matrices.h"

matrix *matrix_new(uint8_t num_rows, uint8_t num_cols){


    assert(num_rows != 0);
    assert(num_cols != 0);

    matrix *m = malloc(sizeof(matrix));
    m->numCols = num_cols;
    m->numRows = num_rows;
    if(num_cols == num_rows){
        m->square = 1;
    }else{
        m->square = 0;
    }

    //allocate a pointer for each row
    
    m->data = calloc(m->numRows, sizeof(*(m->data)));
    //NP_CHECK(m->data);

    for(int i = num_rows; i>=0; i--){
        
        m->data[i]= calloc(num_cols, sizeof(**m->data));;
        //NP_CHECK(m->data[i]);
    }

    return m;
}

void matrix_free(matrix *m){
    for(int i = m->numRows; i>=0; i--){
        free(m->data[i]);
        
    }

    free(m->data);

    free(m);

}


int *matrix_shape(matrix *m){
    int *shape = malloc(sizeof(int) * 2);

    // if (shape == NULL) {
    //     // Handle memory allocation failure
    //     return NULL;
    // }

    shape[0] = m->numCols;
    shape[1] = m->numRows;

    return shape;
}

matrix *dot(matrix *m1, matrix *m2){
    assert(m1->numCols == m2->numRows);

    matrix *result = matrix_new(m1->numRows, m2->numCols);

    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m2->numCols; j++){
            for(int k = 0; k < m1->numCols; k++){
                result->data[i][j] += m1->data[i][k] * m2->data[k][j];
            }
        }
    }

    return result;
}

matrix *cross(matrix *m1, matrix *m2){
    assert(m1->numCols == m2->numRows);
    assert(m1->numRows == m2->numCols);

    matrix *result = matrix_new(m1->numRows, m2->numCols);

    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m2->numCols; j++){
            result->data[i][j] = m1->data[i][j] * m2->data[j][i];
        }
    }

    return result;
}

matrix *matrix_transpose(matrix *m){
    matrix *result = matrix_new(m->numCols, m->numRows);

    for(int i = 0; i < m->numRows; i++){
        for(int j = 0; j < m->numCols; j++){
            result->data[j][i] = m->data[i][j];
        }
    }

    return result;
}



matrix *eye(uint8_t n){
    matrix *m = matrix_new(n,n);
    for(int i = 0; i < n; i++){
        m->data[i][i] = 1;
    }
    return m;
}


matrix *diag(float *d, uint8_t n){
    matrix *m = matrix_new(n,n);
    for(int i = 0; i < n; i++){
        m->data[i][i] = d[i];
    }
    return m;
}


matrix *zeros(uint8_t num_rows, uint8_t num_cols){
    matrix *m = matrix_new(num_rows, num_cols);
    return m;
}

matrix *ones(uint8_t num_rows, uint8_t num_cols){
    matrix *m = matrix_new(num_rows, num_cols);
    for(int i = 0; i < num_rows; i++){
        for(int j = 0; j < num_cols; j++){
            m->data[i][j] = 1;
        }
    }
    return m;
}


void setSection(matrix *m, uint8_t startRow, uint8_t endRow, uint8_t startCol, uint8_t endCol, matrix *section){
    assert(startRow < endRow);
    assert(startCol < endCol);
    assert(endRow <= m->numRows);
    assert(endCol <= m->numCols);
    assert(section->numRows == (endRow - startRow));
    assert(section->numCols == (endCol - startCol));

    for(int i = startRow; i < endRow; i++){
        for(int j = startCol; j < endCol; j++){
            m->data[i][j] = section->data[i - startRow][j - startCol];
        }
    }
}

matrix *getSection(matrix *m, uint8_t startRow, uint8_t endRow, uint8_t startCol, uint8_t endCol){
    assert(startRow <= endRow);
    assert(startCol <= endCol);
    assert(endRow <= m->numRows);
    assert(endCol <= m->numCols);

    matrix *section = matrix_new((endRow - startRow)+1, (endCol - startCol)+1);//todo is this right? should it add one?

    for(int i = startRow; i < endRow; i++){
        for(int j = startCol; j < endCol; j++){
            section->data[i - startRow][j - startCol] = m->data[i][j];
        }
    }

    return section;
}

//todo this can be improved with different algorithms, matlab uses strassen's algorithm
matrix *matMult(matrix *m1, matrix *m2){
    assert(m1->numCols == m2->numRows);
    assert(m1->numRows == m2->numCols);

    matrix *result = matrix_new(m1->numRows, m2->numCols);

    for(int i = 0; i < m1->numRows; i++){
        for(int j = 0; j < m2->numCols; j++){
            for(int k = 0; k < m1->numCols; k++){
                result->data[i][j] += m1->data[i][k] * m2->data[k][j];
            }
        }
    }

    return result;
}