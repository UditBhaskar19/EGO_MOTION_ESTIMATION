# include <stdio.h>
# include <stdlib.h>

# include "constants.h"
# include "loadcsv.h"
# include "egomotion.h"


const char gt_pose_file[] = "D:/github_desktop/EGO_MOTION_ESTIMATION/1_egomotion_wheel_speed/sensor_data/scene-1094/CAN/gt_pose.csv";
const char imu_signal_file[] = "D:/github_desktop/EGO_MOTION_ESTIMATION/1_egomotion_wheel_speed/sensor_data/scene-1094/CAN/imu_signal.csv";
const char zoe_veh_signal_file[] = "D:/github_desktop/EGO_MOTION_ESTIMATION/1_egomotion_wheel_speed/sensor_data/scene-1094/CAN/zoe_veh_signal.csv";

const char rad_mount_param_file[] = "D:/github_desktop/proj_copy/3_egomotion_radar_cartesian/sensor_data/scene-1094/radar/front_calib_extrinsic.csv";
const char meas_dir[] = "D:/github_desktop/proj_copy/3_egomotion_radar_cartesian/sensor_data/scene-1094/radar/front";
const char timestamp_file[] = "D:/github_desktop/proj_copy/3_egomotion_radar_cartesian/sensor_data/scene-1094/radar/front/time_stamps_sec.csv";


BOOL get_inputs(
    struct rad_meas_node** input_node,
	meas_aos* meas,
	uint16_t* frame_count) {

	uint16_t i, num_meas = 0;

	if (*input_node == NULL) {
		return FALSE;
	}
	else {
		num_meas = (*input_node)->rad_meas.num_valid_meas;
		meas->num_valid_meas = num_meas;
		meas->timestamp_sec = (*input_node)->rad_meas.timestamp_sec;

		for (i = 0; i < num_meas; i++) {
			meas->rad_meas_arr[i].px = (*input_node)->rad_meas.rad_meas_arr[i].px;
			meas->rad_meas_arr[i].py = (*input_node)->rad_meas.rad_meas_arr[i].py;
			meas->rad_meas_arr[i].vx = (*input_node)->rad_meas.rad_meas_arr[i].vx;
			meas->rad_meas_arr[i].vy = (*input_node)->rad_meas.rad_meas_arr[i].vy;
			meas->rad_meas_arr[i].px_rms = (*input_node)->rad_meas.rad_meas_arr[i].px_rms;
			meas->rad_meas_arr[i].py_rms = (*input_node)->rad_meas.rad_meas_arr[i].py_rms;
			meas->rad_meas_arr[i].vx_rms = (*input_node)->rad_meas.rad_meas_arr[i].vx_rms;
			meas->rad_meas_arr[i].vy_rms = (*input_node)->rad_meas.rad_meas_arr[i].vy_rms;
			meas->rad_meas_arr[i].dyn_prop = (*input_node)->rad_meas.rad_meas_arr[i].dyn_prop;
			meas->rad_meas_arr[i].valid_state = (*input_node)->rad_meas.rad_meas_arr[i].valid_state;
			meas->rad_meas_arr[i].ambig_state = (*input_node)->rad_meas.rad_meas_arr[i].ambig_state;
		}

		(*frame_count)++;
		*input_node = (*input_node)->next;
		return TRUE;
	}
}


void main_func(
	meas_aos* meas, 
	mount_param* rad_param, 
	estimated_ego_motion* est_egomotion, 
	gated_meas_ids* gated_meas, 
	BOOL* start) {

	coordinate_transformation_sf2vf(meas, rad_param);

	gate_measurements(est_egomotion, meas, gated_meas, start);

	compute_ego_motion_3dof_lse(est_egomotion, gated_meas, meas);
}


void main() {

	/* load the input signals */
	gt_pose_list* gt_pose = create_gt_pose_list(gt_pose_file);
	imu_signal_list* imu_signal = create_imu_signal_list(imu_signal_file);
	zoe_veh_signal_list* zoe_veh_signal = create_zoe_veh_signal_list(zoe_veh_signal_file);
	rad_meas_list* rad_meas = create_rad_meas_list(timestamp_file, meas_dir);
	mount_param_quaternion* mount_q = load_calib_parameters(rad_mount_param_file);

	/* initialized data */
	mount_param rad_param = init_struct_mounting_param(mount_q);
	meas_aos meas = init_meas_aos();
	gated_meas_ids gated_meas = init_gated_meas_ids();
	estimated_ego_motion est_egomotion = init_struct_estimated_ego_motion();

	/* run the filter */
	struct rad_meas_node* input_node = rad_meas->MEAS;
	uint16_t frame_count = -1;
	BOOL start = FALSE;

	while (get_inputs(&input_node, &meas, &frame_count)) {

		main_func(
			&meas,
			&rad_param,
			&est_egomotion,
			&gated_meas,
			&start);


		printf("timestamp:  %5.5f        ", est_egomotion.timestamp_sec);
		printf("vx:  %5.5f        ", est_egomotion.vx);
		printf("vy:  %5.5f        ", est_egomotion.vy);
		printf("yaw rate:  %5.5f\n", (double)rad2deg_const * est_egomotion.yaw_rate);

	} 

	delete_gt_pose_list(gt_pose);
	delete_imu_signal_list(imu_signal);
	delete_zoe_veh_signal_list(zoe_veh_signal);
	delete_rad_meas_list(rad_meas);

}