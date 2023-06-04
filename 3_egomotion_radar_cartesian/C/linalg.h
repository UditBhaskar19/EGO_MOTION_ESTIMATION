# ifndef _LINALG_H_
# define _LINALG_H_

# include "datatypes.h"

void matrix_matrix_multiply_AB(
	const REAL* mat_a,
	const uint16_t nrows_a,
	const uint16_t ncols_a,
	const REAL* mat_b,
	const uint16_t nrows_b,
	const uint16_t ncols_b,
	REAL* result);

void matrix_matrix_multiply_ABt(
	const REAL* mat_a,
	const uint16_t nrows_a,
	const uint16_t ncols_a,
	const REAL* mat_b,
	const uint16_t nrows_b,
	const uint16_t ncols_b,
	REAL* result);

void matrix_matrix_multiply_AtA(
	const REAL* mat_a,
	const uint16_t nrows_a,
	const uint16_t ncols_a,
	REAL* result);

void matrix_vector_multiply_MV(
	const REAL* mat_a,
	const uint16_t nrows_a,
	const uint16_t ncols_a,
	const REAL* vec_b,
	const uint16_t size,
	REAL* result);

void matrix_inverse(
	REAL* sq_mat,
	const uint16_t mat_size,
	REAL* inv);

# endif
