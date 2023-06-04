# include "linalg.h"


void matrix_matrix_multiply_AB(
	const REAL* mat_a,
	const uint16_t nrows_a,
	const uint16_t ncols_a,
	const REAL* mat_b,
	const uint16_t nrows_b,
	const uint16_t ncols_b,
	REAL* result) {

	REAL sum;
	uint16_t i, j, k;

	for (i = 0; i < nrows_a; i++) {
		for (j = 0; j < ncols_b; j++) {
			sum = 0;
			for (k = 0; k < ncols_a; k++) {
				sum += mat_a[i * ncols_a + k] * mat_b[k * ncols_b + j];
			}
			result[i * ncols_b + j] = sum;
		}
	}
}


void matrix_matrix_multiply_ABt(
	const REAL* mat_a,
	const uint16_t nrows_a,
	const uint16_t ncols_a,
	const REAL* mat_b,
	const uint16_t nrows_b,
	const uint16_t ncols_b,
	REAL* result) {

	REAL sum;
	uint16_t i, j, k;

	for (i = 0; i < nrows_a; i++) {
		for (j = 0; j < nrows_b; j++) {
			sum = 0;
			for (k = 0; k < ncols_a; k++) {
				sum += mat_a[i * ncols_a + k] * mat_b[j * ncols_b + k];
			}
			result[i * ncols_b + j] = sum;
		}
	}
}


void matrix_matrix_multiply_AtA(
	const REAL* mat_a,
	const uint16_t nrows_a,
	const uint16_t ncols_a,
	REAL* result) {

	REAL sum;
	uint16_t i, j, k;

	for (i = 0; i < ncols_a; i++) {
		for (j = i; j < ncols_a; j++) {
			sum = 0;
			for (k = 0; k < nrows_a; k++) {
				sum += mat_a[k * ncols_a + i] * mat_a[k * ncols_a + j];
			}
			result[i * ncols_a + j] = sum;
			result[j * ncols_a + i] = sum; // Assign the symmetric element
		}
	}
}


void matrix_vector_multiply_MV(
	const REAL* mat_a,
	const uint16_t nrows_a,
	const uint16_t ncols_a,
	const REAL* vec_b,
	const uint16_t size,
	REAL* result) {

	uint16_t i, j;
	for (i = 0; i < nrows_a; i++) {
		result[i] = 0;
		for (j = 0; j < size; j++) {
			result[i] += mat_a[i * ncols_a + j] * vec_b[j];
		}
	}
}


void matrix_inverse(
	REAL* sq_mat,
	const uint16_t mat_size,
	REAL* inv)
{
	uint16_t i = 0;
	uint16_t j = 0;
	uint16_t k = 0;
	REAL val = 0;

	// create identity matrix;
	for (i = 0; i < mat_size; i++) {
		for (j = 0; j < mat_size; j++) {
			if (i == j)  inv[mat_size * i + j] = 1;
			else inv[mat_size * i + j] = 0;
		}
	}

	/* row elimination */
	for (k = 0; k < mat_size; k++) {
		val = sq_mat[k * mat_size + k];

		for (i = 0; i < mat_size; i++) {
			sq_mat[k * mat_size + i] = sq_mat[k * mat_size + i] / val;
			inv[k * mat_size + i] = inv[k * mat_size + i] / val;
		}

		for (i = 0; i < k; i++) {
			val = sq_mat[i * mat_size + k];
			for (j = 0; j < mat_size; j++) {
				sq_mat[i * mat_size + j] -= (sq_mat[k * mat_size + j] * val);
				inv[i * mat_size + j] -= (inv[k * mat_size + j] * val);
			}
		}

		for (i = k + 1; i < mat_size; i++) {
			val = sq_mat[i * mat_size + k];
			for (j = 0; j < mat_size; j++) {
				sq_mat[i * mat_size + j] -= (sq_mat[k * mat_size + j] * val);
				inv[i * mat_size + j] -= (inv[k * mat_size + j] * val);
			}
		}
	}
}

