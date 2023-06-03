# ifndef _LINALG_H_
# define _LINALG_H_

# include "datatypes.h"

/* ---------------------------------------------------------------------
 *                       Matrix Structures
 * --------------------------------------------------------------------- */

typedef struct matrix {
	REAL* mat;
	uint16_t max_nrows;
	uint16_t max_ncols;
} matrix;


typedef struct vector {
	REAL* vec;
	uint16_t max_size;
} vector;


void init_matrix_struct(
	matrix* mat_struct,
	REAL* mat,
	const uint16_t max_nrows,
	const uint16_t max_ncols);


void init_vector_struct(
	vector* vec_struct,
	REAL* vec,
	const uint16_t size);

/*---------------------------------------------------------------------
 *                           Matrix Operations
 * -------------------------------------------------------------------- */

void matrix_inverse(
	matrix* sq_mat,
	const uint16_t mat_size,
	matrix* inv);

/* ---------------------------------------------------------------------
 *                       Matrix - Matrix product
 * --------------------------------------------------------------------- */

void matrix_matrix_multiply_AB(
	const matrix* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const matrix* mat_b, const uint16_t nrows_b, const uint16_t ncols_b,
	matrix* mat_ab);

void matrix_matrix_multiply_ABt(
	const matrix* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const matrix* mat_b, const uint16_t nrows_b, const uint16_t ncols_b,
	matrix* result);

void matrix_matrix_multiply_AtA(
	const matrix* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	matrix* result);

/*

void matrix_matrix_multiply_AtB(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const REAL* mat_b, const uint16_t nrows_b, const uint16_t ncols_b,
	REAL* result);

void matrix_matrix_multiply_AAt(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	REAL* result);

*/

/* ---------------------------------------------------------------------
 *                      Matrix - Vector product
 * --------------------------------------------------------------------- */

void matrix_vector_multiply_MV(
	const matrix* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const vector* vec_b, const uint16_t size,
	vector* result);

/*

void vector_matrix_multiply_VM(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const REAL* vec_b, const uint16_t nrows_b,
	REAL* result);

void matrix_vector_multiply_MtV(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const REAL* vec_b, const uint16_t nrows_b,
	REAL* result);

void vector_matrix_multiply_VMt(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const REAL* vec_b, const uint16_t nrows_b,
	REAL* result);

*/

/* ---------------------------------------------------------------------
 *                      Vector - Vector product
 * --------------------------------------------------------------------- */



# endif
