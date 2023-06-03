# include "linalg.h"
# include "datatypes.h"


void init_matrix_struct(
	matrix* mat_struct,
	REAL* mat,
	const uint16_t max_nrows,
	const uint16_t max_ncols) {
	mat_struct->mat = mat;
	mat_struct->max_nrows = max_nrows;
	mat_struct->max_ncols = max_ncols;
}


void init_vector_struct(
	vector* vec_struct,
	REAL* vec,
	const uint16_t size) {
	vec_struct->vec = vec;
	vec_struct->max_size = size;
}


void matrix_matrix_multiply_AB(
	const matrix* mat_a, 
	const uint16_t nrows_a, 
	const uint16_t ncols_a,
	const matrix* mat_b, 
	const uint16_t nrows_b, 
	const uint16_t ncols_b,
	matrix* result) {

	REAL sum;
	uint16_t i, j, k;
	for (i = 0; i < nrows_a; i++) {
		for (j = 0; j < ncols_b; j++) {
			sum = 0.0f;
			for (k = 0; k < ncols_a; k++) {
				sum += mat_a->mat[i * mat_a->max_ncols + k] * mat_b->mat[k * mat_b->max_ncols + j];
			}
			result->mat[i * result->max_ncols + j] = sum;
		}
	}
}


void matrix_matrix_multiply_ABt(
	const matrix* mat_a,
	const uint16_t nrows_a, 
	const uint16_t ncols_a,
	const matrix* mat_b, 
	const uint16_t nrows_b, 
	const uint16_t ncols_b,
	matrix* result) {

	REAL sum;
	uint16_t i, j, k;
	for (i = 0; i < nrows_a; i++) {
		for (j = 0; j < nrows_b; j++) {
			sum = 0.0f;
			for (k = 0; k < ncols_a; k++) {
				sum += mat_a->mat[i * mat_a->max_ncols + k] * mat_b->mat[j * mat_b->max_ncols + k];
			}
			result->mat[i * mat_b->max_nrows + j] = sum;
		}
	}
}


void matrix_matrix_multiply_AtA(
	const matrix* mat_a,
	const uint16_t nrows_a, 
	const uint16_t ncols_a,
	matrix* result) {

	REAL sum;
	uint16_t i, j, k;
	for (i = 0; i < ncols_a; i++) {
		for (j = i; j < ncols_a; j++) {
			sum = 0;
			for (k = 0; k < nrows_a; k++) {
				sum += mat_a->mat[k * mat_a->max_ncols + i] * mat_a->mat[k * mat_a->max_ncols + j];
			}
			result->mat[i * mat_a->max_ncols + j] = sum;
			result->mat[j * mat_a->max_ncols + i] = sum; // Assign the symmetric element
		}
	}
}


void matrix_vector_multiply_MV(
	const matrix* mat_a, 
	const uint16_t nrows_a,
	const uint16_t ncols_a, 
	const vector* vec_b,
	const uint16_t size,
	vector* result) {

	uint16_t i, j;
	for (i = 0; i < nrows_a; i++) {
		result->vec[i] = 0.0f;
		for (j = 0; j < size; j++) {
			result->vec[i] += mat_a->mat[i * mat_a->max_ncols + j] * vec_b->vec[j];
		}
	}
}


void matrix_inverse(
	matrix* sq_mat, 
	const uint16_t mat_size,
	matrix* inv)
{
	uint16_t i = 0;
	uint16_t j = 0;
	uint16_t k = 0;
	REAL val = 0;

	// create identity matrix;
	for (i = 0; i < mat_size; i++) {
		for (j = 0; j < mat_size; j++) {
			if (i == j)  inv->mat[inv->max_ncols * i + j] = 1;
			else inv->mat[inv->max_ncols * i + j] = 0;
		}
	}

	//row elimination
	for (k = 0; k < mat_size; k++) {
		val = sq_mat->mat[k * sq_mat->max_ncols + k];

		/*Dividing entire row*/
		for (i = 0; i < mat_size; i++) {
			sq_mat->mat[k * sq_mat->max_ncols + i] = sq_mat->mat[k * sq_mat->max_ncols + i] / val;
			inv->mat[k * inv->max_ncols + i] = inv->mat[k * inv->max_ncols + i] / val;
		}

		/* Row operations*/
		for (i = 0; i < k; i++) {
			val = sq_mat->mat[i * sq_mat->max_ncols + k];
			for (j = 0; j < mat_size; j++) {
				sq_mat->mat[i * sq_mat->max_ncols + j] -= (sq_mat->mat[k * sq_mat->max_ncols + j] * val);
				inv->mat[i * inv->max_ncols + j] -= (inv->mat[k * inv->max_ncols + j] * val);
			}
		}

		for (i = k + 1; i < mat_size; i++) {
			val = sq_mat->mat[i * sq_mat->max_ncols + k];
			for (j = 0; j < mat_size; j++) {
				sq_mat->mat[i * sq_mat->max_ncols + j] -= (sq_mat->mat[k * sq_mat->max_ncols + j] * val);
				inv->mat[i * inv->max_ncols + j] -= (inv->mat[k * inv->max_ncols + j] * val);
			}
		}
	}
}


/* ---------------------------------------------------------------------
 *                       Matrix - Matrix product
 * --------------------------------------------------------------------- */

/*
void matrix_matrix_multiply_AB(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a, 
	const REAL* mat_b, const uint16_t nrows_b, const uint16_t ncols_b, 
	REAL* result) {

	REAL sum;
	uint16_t i, j, k;
	for (i = 0; i < nrows_a; i++) {
		for (j = 0; j < ncols_b; j++) {
			sum = 0.0f;
			for (k = 0; k < ncols_a; k++) {
				sum += *(mat_a + i * ncols_a + k) * *(mat_b + k * ncols_b + j);
			}
			*(result + i * ncols_b + j) = sum;
		}
	}
}

*/

/*

void matrix_matrix_multiply_ABt(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a, 
	const REAL* mat_b, const uint16_t nrows_b, const uint16_t ncols_b, 
	REAL* result) {

	REAL sum;
	uint16_t i, j, k;
	for (i = 0; i < nrows_a; i++) {
		for (j = 0; j < nrows_b; j++) {
			sum = 0.0f;
			for (k = 0; k < ncols_a; k++) {
				sum += *(mat_a + i * ncols_a + k) * *(mat_b + j * ncols_b + k);
			}
			*(result + i * nrows_b + j) = sum;
		}
	}
}


void matrix_matrix_multiply_AtB(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const REAL* mat_b, const uint16_t nrows_b, const uint16_t ncols_b,
	REAL* result) {

	REAL sum;
	uint16_t i, j, k;
	for (i = 0; i < ncols_a; i++) {
		for (j = 0; j < ncols_b; j++) {
			sum = 0.0f;
			for (k = 0; k < nrows_a; k++) {
				sum += *(mat_a + k * ncols_a + i) * *(mat_b + k * ncols_b + j);
			}
			*(result + i * ncols_b + j) = sum;
		}
	}
}

*/



/*
void matrix_matrix_multiply_AtA(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a, 
	REAL* result) {

	REAL sum;
	uint16_t i, j, k;
	for (i = 0; i < ncols_a; i++) {
		for (j = i; j < ncols_a; j++) {
			sum = 0;
			for (k = 0; k < nrows_a; k++) {
				sum += *(mat_a + k * ncols_a + i) * *(mat_a + k * ncols_a + j);
			}
			*(result + i * ncols_a + j) = sum;
			*(result + j * ncols_a + i) = sum; // Assign the symmetric element
		}
	}
}

*/

/*

void matrix_matrix_multiply_AAt(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	REAL* result) {

	REAL sum;
	uint16_t i, j, k;
	for (i = 0; i < nrows_a; i++) {
		for (j = i; j < nrows_a; j++) {
			sum = 0;
			for (k = 0; k < ncols_a; k++) {
				sum += *(mat_a + i * ncols_a + k) * *(mat_a + j * ncols_a + k);
			}
			*(result + i * nrows_a + j) = sum;
			*(result + j * nrows_a + i) = sum; // Assign the symmetric element
		}
	}
}

*/

/* ---------------------------------------------------------------------
 *                      Matrix - Vector product
 * --------------------------------------------------------------------- */

/*

void matrix_vector_multiply_MV(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const REAL* vec_b, const uint16_t size_b,
	REAL* result) {

	REAL sum;
	for (uint16_t i = 0; i < nrows_a; i++) {
		for (uint16_t j = 0; j < size_b; j++) {
			sum += *(mat_a + i * ncols_a + j) * *(vec_b + j);
		}
	}
}

*/

/*
void vector_matrix_multiply_VM(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const REAL* vec_b, const uint16_t nrows_b,
	REAL* result) {

}

void matrix_vector_multiply_MtV(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const REAL* vec_b, const uint16_t nrows_b,
	REAL* result) {

}

void vector_matrix_multiply_VMt(
	const REAL* mat_a, const uint16_t nrows_a, const uint16_t ncols_a,
	const REAL* vec_b, const uint16_t nrows_b,
	REAL* result) {

}

*/