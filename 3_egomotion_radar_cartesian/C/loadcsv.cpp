# include <stdio.h>
# include <stdlib.h>
# include <string.h>

# include "loadcsv.h"
# include "constants.h"

/* ------------------------------------------------------------------------------------------------- */

void init_rad_meas_list(rad_meas_list* meas) {
	meas->MEAS = NULL;
	meas->num_ele = 0;
}

/* ------------------------------------------------------------------------------------------------- */

void set_rad_meas_array(
	char *line, 
	const uint16_t idx, 
	rad_meas* meas) {

	char* sp;
	sp = strtok(line, ","); meas[idx].px = (REAL)atof(sp);
	sp = strtok(NULL, ","); meas[idx].py = (REAL)atof(sp);
	sp = strtok(NULL, ","); meas[idx].vx = (REAL)atof(sp);
	sp = strtok(NULL, ","); meas[idx].vy = (REAL)atof(sp);
	sp = strtok(NULL, ","); meas[idx].vx_comp = (REAL)atof(sp);
	sp = strtok(NULL, ","); meas[idx].vy_comp = (REAL)atof(sp);
	sp = strtok(NULL, ","); meas[idx].px_rms = (REAL)atof(sp);
	sp = strtok(NULL, ","); meas[idx].py_rms = (REAL)atof(sp);
	sp = strtok(NULL, ","); meas[idx].vx_rms = (REAL)atof(sp);
	sp = strtok(NULL, ","); meas[idx].vy_rms = (REAL)atof(sp);
	sp = strtok(NULL, ","); meas[idx].rcs = (REAL)atof(sp);
	sp = strtok(NULL, ","); meas[idx].pdh0 = (REAL)false_alarm_probability_vals[atoi(sp)];
	sp = strtok(NULL, ","); meas[idx].dyn_prop = (uint8_t)atoi(sp);
	sp = strtok(NULL, ","); meas[idx].ambig_state = (uint8_t)atoi(sp);
	sp = strtok(NULL, ","); meas[idx].valid_state = (uint8_t)atoi(sp);
}

/* ------------------------------------------------------------------------------------------------- */

void set_rad_meas_struct(
	char* line_timestamps,
	uint16_t frame_idx,
	char* rad_meas_dir,
	rad_meas_struct_arr* meas) {

	// create the file path
	char rad_meas_file[max_string_len];
	const char* extension = ".csv";
	snprintf(rad_meas_file, sizeof(rad_meas_file), "%s/%d%s", rad_meas_dir, frame_idx, extension);
	//printf("%s\n", rad_meas_file);

	// open the file
	FILE* fp = fopen(rad_meas_file, "r");
	if (fp == NULL) {
		printf("Trouble opening radar frame %d file ...", frame_idx);
		exit(0);
	}
	char line[max_string_len];
	fgets(line, max_string_len, fp);   // skip the first line

	// set data
	uint16_t num_valid_meas = 0;
	while (fgets(line, max_string_len, fp) != NULL) {
		set_rad_meas_array(line, num_valid_meas, meas->rad_meas_arr);
		num_valid_meas++;
	}
	char* sp = strtok(line_timestamps, ","); 
	meas->timestamp_sec = (REAL)atof(sp);
	meas->num_valid_meas = num_valid_meas;
}

/* ------------------------------------------------------------------------------------------------- */

struct rad_meas_node* create_new_rad_meas_node(
	char* line,
	uint16_t frame_idx,
	char* radar_meas_dir) {

	struct rad_meas_node* new_node;
	new_node = (struct rad_meas_node*)malloc(sizeof(struct rad_meas_node));
	set_rad_meas_struct(line, frame_idx, radar_meas_dir, &new_node->rad_meas);
	new_node->next = NULL;
	return new_node;
}

/* ------------------------------------------------------------------------------------------------- */

void append_rad_meas(struct rad_meas_node** list, struct rad_meas_node* new_node) {
	if (*list == NULL) {
		*list = new_node;
	}
	else {
		struct rad_meas_node* iter;
		iter = *list;
		while (iter->next != NULL) {
			iter = iter->next;
		}
		iter->next = new_node;
	}
}

void set_rad_meas_list(
	char* timestamps_filepath,
	char* radar_meas_dir,
	rad_meas_list* rad_meas) {

	const size_t n = 1000;
	char line[n], * sp;

	FILE* fp = fopen(timestamps_filepath, "r");
	if (fp == NULL) {
		printf("Trouble opening timestamp file ...");
		exit(0);
	}

	fgets(line, n, fp);   // skip the first line
	struct rad_meas_node* new_node;
	uint16_t num_frames = 0;

	while (fgets(line, n, fp) != NULL) {
		num_frames++;
		new_node = create_new_rad_meas_node(line, num_frames, radar_meas_dir);
		append_rad_meas(&rad_meas->MEAS, new_node);
	}
	rad_meas->num_ele = num_frames;
}

/* ------------------------------------------------------------------------------------------------- */

rad_meas_list* create_rad_meas_list(
	char* timestamps_filepath,
	char* radar_meas_dir) {

	rad_meas_list* rad_meas;
	rad_meas = (rad_meas_list*)malloc(sizeof(rad_meas_list));
	init_rad_meas_list(rad_meas);
	set_rad_meas_list(timestamps_filepath, radar_meas_dir, rad_meas);
	return rad_meas;
}