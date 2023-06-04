# include <stdio.h>
# include <stdlib.h>
# include <string.h>

# include "loadcsv.h"

/* ------------------------------------------------------------------------------------------------- */

void init_gt_pose_list(gt_pose_list* pose) {
	pose->POSE = NULL;
	pose->num_ele = 0;
}

/* ------------------------------------------------------------------------------------------------- */

void set_gt_pose_struct(char* line, gt_pose* pose) {
	/* each line in the csv file has the following data in this order
	   timestamp_sec, px, py, pz, q1, q2, q3, q4, vx, vy, vz, ax, ay, az, rot_rate_x, rot_rate_y, rot_rate_z */
	char* sp;
	sp = strtok(line, ","); pose->timestamp_sec = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->px = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->py = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->pz = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->q1 = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->q2 = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->q3 = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->q4 = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->vx = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->vy = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->vz = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->ax = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->ay = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->az = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->rot_rate_x = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->rot_rate_y = (REAL)atof(sp);
	sp = strtok(NULL, ","); pose->rot_rate_z = (REAL)atof(sp);
}

/* ------------------------------------------------------------------------------------------------- */

struct gt_pose_node* create_new_gt_pose_node(char* line) {
	struct gt_pose_node* new_node;
	new_node = (struct gt_pose_node*)malloc(sizeof(struct gt_pose_node));
	set_gt_pose_struct(line, &new_node->pose);
	new_node->next = NULL;
	return new_node;
}

/* ------------------------------------------------------------------------------------------------- */

void append_gt_pose(struct gt_pose_node** list, struct gt_pose_node* new_node) {
	if (*list == NULL) {
		*list = new_node;
	}
	else {
		struct gt_pose_node* iter;
		iter = *list;
		while (iter->next != NULL) {
			iter = iter->next;
		}
		iter->next = new_node;
	}
}

void set_gt_pose_list(const char* filepath, gt_pose_list* pose) {

	const size_t n = 1000;
	char line[n], * sp;

	FILE* fp = fopen(filepath, "r");
	if (fp == NULL) {
		printf("Trouble opening file ...");
		exit(0);
	}

	fgets(line, n, fp);   // skip the first line
	struct gt_pose_node* new_node;

	while (fgets(line, n, fp) != NULL) {
		new_node = create_new_gt_pose_node(line);
		append_gt_pose(&pose->POSE, new_node);
		(pose->num_ele)++;
	}
}

/* ------------------------------------------------------------------------------------------------- */

gt_pose_list* create_gt_pose_list(const char* filepath) {
	gt_pose_list* gt_pose;
	gt_pose = (gt_pose_list*)malloc(sizeof(gt_pose_list));
	init_gt_pose_list(gt_pose);
	set_gt_pose_list(filepath, gt_pose);
	return gt_pose;
}

/* ------------------------------------------------------------------------------------------------- */

void delete_gt_pose(struct gt_pose_node** list) {
	struct gt_pose_node* iter, * temp;
	iter = temp = *list;
	while (iter != NULL) {
		temp = iter->next;
		free(iter);
		iter = temp;
	}
	*list = NULL;
}

void delete_gt_pose_list(gt_pose_list* gt_pose) {
	delete_gt_pose(&gt_pose->POSE);
	free(gt_pose);
}

/* ------------------------------------------------------------------------------------------------- */

void print_nodes_gt_pose(struct gt_pose_node* list) {
	while (list != NULL) {
		printf("%10.3f", list->pose.timestamp_sec);
		printf("%10.3f", list->pose.px);
		printf("%10.3f", list->pose.py);
		printf("%10.3f", list->pose.pz);
		printf("%10.3f", list->pose.vx);
		printf("%10.3f", list->pose.vy);
		printf("%10.3f", list->pose.vz);
		printf("%10.3f", list->pose.ax);
		printf("%10.3f", list->pose.ay);
		printf("%10.3f", list->pose.az);
		printf("%10.3f", list->pose.q1);
		printf("%10.3f", list->pose.q2);
		printf("%10.3f", list->pose.q3);
		printf("%10.3f", list->pose.q4);
		printf("%10.3f", list->pose.rot_rate_x);
		printf("%10.3f", list->pose.rot_rate_y);
		printf("%10.3f", list->pose.rot_rate_z);
		printf("\n");
		list = list->next;
	}
}

/* ------------------------------------------------------------------------------------------------- */