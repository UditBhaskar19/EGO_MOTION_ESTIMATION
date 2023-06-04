# include <stdio.h>
# include <stdlib.h>
# include <string.h>

# include "loadcsv.h"

/* ------------------------------------------------------------------------------------------------- */

void init_zoe_veh_signal_list(zoe_veh_signal_list* zoe_veh) {
	zoe_veh->VEH = NULL;
	zoe_veh->num_ele = 0;
}

/* ------------------------------------------------------------------------------------------------- */

void set_zoe_veh_signal_struct(char* line, zoe_veh_signal* zoe_veh) {
	/* each line in the csv file has the following data in this order
	   timestamp_sec, FL_wheel_speed, FR_wheel_speed, RR_wheel_speed, RL_wheel_speed, steering_angle, odom_speed, odom, ax, ay */
	char* sp;
	sp = strtok(line, ","); zoe_veh->timestamp_sec = (REAL)atof(sp);
	sp = strtok(NULL, ","); zoe_veh->FL_wheel_speed = (REAL)atof(sp);
	sp = strtok(NULL, ","); zoe_veh->FR_wheel_speed = (REAL)atof(sp);
	sp = strtok(NULL, ","); zoe_veh->RR_wheel_speed = (REAL)atof(sp);
	sp = strtok(NULL, ","); zoe_veh->RL_wheel_speed = (REAL)atof(sp);
	sp = strtok(NULL, ","); zoe_veh->steering_angle = (REAL)atof(sp);
	sp = strtok(NULL, ","); zoe_veh->odom_speed = (REAL)atof(sp);
	sp = strtok(NULL, ","); zoe_veh->odom = (REAL)atof(sp);
	sp = strtok(NULL, ","); zoe_veh->ax = (REAL)atof(sp);
	sp = strtok(NULL, ","); zoe_veh->ay = (REAL)atof(sp);
}

/* ------------------------------------------------------------------------------------------------- */

struct zoe_veh_signal_node* create_new_zoe_veh_signal_node(char* line) {
	struct zoe_veh_signal_node* new_node;
	new_node = (struct zoe_veh_signal_node*)malloc(sizeof(struct zoe_veh_signal_node));
	set_zoe_veh_signal_struct(line, &new_node->veh_signal);
	new_node->next = NULL;
	return new_node;
}

/* ------------------------------------------------------------------------------------------------- */

void append_zoe_veh_signal(struct zoe_veh_signal_node** list, struct zoe_veh_signal_node* new_node) {
	if (*list == NULL) {
		*list = new_node;
	}
	else {
		struct zoe_veh_signal_node* iter;
		iter = *list;
		while (iter->next != NULL) {
			iter = iter->next;
		}
		iter->next = new_node;
	}
}

void set_zoe_veh_signal_list(const char* filepath, zoe_veh_signal_list* zoe_veh) {

	const size_t n = 1000;
	char line[n], * sp;

	FILE* fp = fopen(filepath, "r");
	if (fp == NULL) {
		printf("Trouble opening file ...");
		exit(0);
	}

	fgets(line, n, fp);   // skip the first line
	struct zoe_veh_signal_node* new_node;

	while (fgets(line, n, fp) != NULL) {
		new_node = create_new_zoe_veh_signal_node(line);
		append_zoe_veh_signal(&zoe_veh->VEH, new_node);
		(zoe_veh->num_ele)++;
	}
}

/* ------------------------------------------------------------------------------------------------- */

zoe_veh_signal_list* create_zoe_veh_signal_list(const char* filepath) {
	zoe_veh_signal_list* zoe_veh_signal;
	zoe_veh_signal = (zoe_veh_signal_list*)malloc(sizeof(zoe_veh_signal_list));
	init_zoe_veh_signal_list(zoe_veh_signal);
	set_zoe_veh_signal_list(filepath, zoe_veh_signal);
	return zoe_veh_signal;
}

/* ------------------------------------------------------------------------------------------------- */

void delete_zoe_veh_signal(struct zoe_veh_signal_node** list) {
	struct zoe_veh_signal_node* iter, * temp;
	iter = temp = *list;
	while (iter != NULL) {
		temp = iter->next;
		free(iter);
		iter = temp;
	}
	*list = NULL;
}

void delete_zoe_veh_signal_list(zoe_veh_signal_list* zoe_veh) {
	delete_zoe_veh_signal(&zoe_veh->VEH);
	free(zoe_veh);
}

/* ------------------------------------------------------------------------------------------------- */

void print_nodes_veh_signal(struct zoe_veh_signal_node* list) {
	while (list != NULL) {
		printf("%10.3f", list->veh_signal.timestamp_sec);
		printf("%10.3f", list->veh_signal.ax);
		printf("%10.3f", list->veh_signal.ay);
		printf("%10.3f", list->veh_signal.FL_wheel_speed);
		printf("%10.3f", list->veh_signal.FR_wheel_speed);
		printf("%10.3f", list->veh_signal.RL_wheel_speed);
		printf("%10.3f", list->veh_signal.RR_wheel_speed);
		printf("%10.3f", list->veh_signal.odom);
		printf("%10.3f", list->veh_signal.odom_speed);
		printf("%10.3f", list->veh_signal.steering_angle);
		printf("\n");
		list = list->next;
	}
}

/* ------------------------------------------------------------------------------------------------- */