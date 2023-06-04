# include <stdio.h>
# include <stdlib.h>
# include <string.h>

# include "loadcsv.h"

/* ------------------------------------------------------------------------------------------------- */

void init_imu_signal_list(imu_signal_list* imu) {
	imu->IMU = NULL;
	imu->num_ele = 0;
}

/* ------------------------------------------------------------------------------------------------- */

void set_imu_signal_struct(char* line, imu_signal* imu) {
	/* each line in the csv file has the following data in this order
	   timestamp_sec, ax, ay, az, rot_rate_x, rot_rate_y, rot_rate_z, q1, q2, q3, q4 */
	char* sp;
	sp = strtok(line, ","); imu->timestamp_sec = (REAL)atof(sp);
	sp = strtok(NULL, ","); imu->ax = (REAL)atof(sp);
	sp = strtok(NULL, ","); imu->ay = (REAL)atof(sp);
	sp = strtok(NULL, ","); imu->az = (REAL)atof(sp);
	sp = strtok(NULL, ","); imu->rot_rate_x = (REAL)atof(sp);
	sp = strtok(NULL, ","); imu->rot_rate_y = (REAL)atof(sp);
	sp = strtok(NULL, ","); imu->rot_rate_z = (REAL)atof(sp);
	sp = strtok(NULL, ","); imu->q1 = (REAL)atof(sp);
	sp = strtok(NULL, ","); imu->q2 = (REAL)atof(sp);
	sp = strtok(NULL, ","); imu->q3 = (REAL)atof(sp);
	sp = strtok(NULL, ","); imu->q4 = (REAL)atof(sp);
}

/* ------------------------------------------------------------------------------------------------- */

struct imu_signal_node* create_new_imu_signal_node(char* line) {
	struct imu_signal_node* new_node;
	new_node = (struct imu_signal_node*)malloc(sizeof(struct imu_signal_node));
	set_imu_signal_struct(line, &new_node->imu);
	new_node->next = NULL;
	return new_node;
}

/* ------------------------------------------------------------------------------------------------- */

void append_imu_signal(struct imu_signal_node** list, struct imu_signal_node* new_node) {
	if (*list == NULL) {
		*list = new_node;
	}
	else {
		struct imu_signal_node* iter;
		iter = *list;
		while (iter->next != NULL) {
			iter = iter->next;
		}
		iter->next = new_node;
	}
}

void set_imu_signal_list(const char* filepath, imu_signal_list* imu) {

	const size_t n = 1000;
	char line[n], * sp;

	FILE* fp = fopen(filepath, "r");
	if (fp == NULL) {
		printf("Trouble opening file ...");
		exit(0);
	}

	fgets(line, n, fp);   // skip the first line
	struct imu_signal_node* new_node;

	while (fgets(line, n, fp) != NULL) {
		new_node = create_new_imu_signal_node(line);
		append_imu_signal(&imu->IMU, new_node);
		(imu->num_ele)++;
	}
}

/* ------------------------------------------------------------------------------------------------- */

imu_signal_list* create_imu_signal_list(const char* filepath) {
	imu_signal_list* imu_signal;
	imu_signal = (imu_signal_list*)malloc(sizeof(imu_signal_list));
	init_imu_signal_list(imu_signal);
	set_imu_signal_list(filepath, imu_signal);
	return imu_signal;
}

/* ------------------------------------------------------------------------------------------------- */

void delete_imu_signal(struct imu_signal_node** list) {
	struct imu_signal_node* iter, * temp;
	iter = temp = *list;
	while (iter != NULL) {
		temp = iter->next;
		free(iter);
		iter = temp;
	}
	*list = NULL;
}

void delete_imu_signal_list(imu_signal_list* imu_signal) {
	delete_imu_signal(&imu_signal->IMU);
	free(imu_signal);
}

/* ------------------------------------------------------------------------------------------------- */

void print_nodes_imu_signal(struct imu_signal_node* list) {
	while (list != NULL) {
		printf("%10.3f", list->imu.timestamp_sec);
		printf("%10.3f", list->imu.ax);
		printf("%10.3f", list->imu.ay);
		printf("%10.3f", list->imu.az);
		printf("%10.3f", list->imu.q1);
		printf("%10.3f", list->imu.q2);
		printf("%10.3f", list->imu.q3);
		printf("%10.3f", list->imu.q4);
		printf("%10.3f", list->imu.rot_rate_x);
		printf("%10.3f", list->imu.rot_rate_y);
		printf("%10.3f", list->imu.rot_rate_z);
		printf("\n");
		list = list->next;
	}
}

/* ------------------------------------------------------------------------------------------------- */