# include <stdio.h>
# include <stdlib.h>
# include <string.h>

# include "loadcsv.h"

/* ------------------------------------------------------------------------------------------------- */

void init_gt_pose_list(gt_pose_list* pose) {
	pose->POSE = NULL;
	pose->num_ele = 0;
}

void init_imu_signal_list(imu_signal_list* imu) {
	imu->IMU = NULL;
	imu->num_ele = 0;
}

void init_zoe_veh_signal_list(zoe_veh_signal_list* zoe_veh) {
	zoe_veh->VEH = NULL;
	zoe_veh->num_ele = 0;
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

struct gt_pose_node* create_new_gt_pose_node(char* line) {
	struct gt_pose_node* new_node;
	new_node = (struct gt_pose_node*)malloc(sizeof(struct gt_pose_node));
	set_gt_pose_struct(line, &new_node->pose);
	new_node->next = NULL;
	return new_node;
}

struct imu_signal_node* create_new_imu_signal_node(char* line) {
	struct imu_signal_node* new_node;
	new_node = (struct imu_signal_node*)malloc(sizeof(struct imu_signal_node));
	set_imu_signal_struct(line, &new_node->imu);
	new_node->next = NULL;
	return new_node;
}

struct zoe_veh_signal_node* create_new_zoe_veh_signal_node(char* line) {
	struct zoe_veh_signal_node* new_node;
	new_node = (struct zoe_veh_signal_node*)malloc(sizeof(struct zoe_veh_signal_node));
	set_zoe_veh_signal_struct(line, &new_node->veh_signal);
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

gt_pose_list* create_gt_pose_list(const char* filepath) {
	gt_pose_list* gt_pose;
	gt_pose = (gt_pose_list*)malloc(sizeof(gt_pose_list));
	init_gt_pose_list(gt_pose);
	set_gt_pose_list(filepath, gt_pose);
	return gt_pose;
}


imu_signal_list* create_imu_signal_list(const char* filepath) {
	imu_signal_list* imu_signal;
	imu_signal = (imu_signal_list*)malloc(sizeof(imu_signal_list));
	init_imu_signal_list(imu_signal);
	set_imu_signal_list(filepath, imu_signal);
	return imu_signal;
}

zoe_veh_signal_list* create_zoe_veh_signal_list(const char* filepath) {
	zoe_veh_signal_list* zoe_veh_signal;
	zoe_veh_signal = (zoe_veh_signal_list*)malloc(sizeof(zoe_veh_signal_list));
	init_zoe_veh_signal_list(zoe_veh_signal);
	set_zoe_veh_signal_list(filepath, zoe_veh_signal);
	return zoe_veh_signal;
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