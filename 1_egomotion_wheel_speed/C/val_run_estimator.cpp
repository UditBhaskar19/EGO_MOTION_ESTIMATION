# include <stdio.h>
# include <stdlib.h>

# include "constants.h"
# include "loadcsv.h"
# include "egomotion.h"


const char gt_pose_file[] = "D:/github_desktop/EGO_MOTION_ESTIMATION/1_egomotion_wheel_speed/sensor_data/scene-1094/CAN/gt_pose.csv";
const char imu_signal_file[] = "D:/github_desktop/EGO_MOTION_ESTIMATION/1_egomotion_wheel_speed/sensor_data/scene-1094/CAN/imu_signal.csv";
const char zoe_veh_signal_file[] = "D:/github_desktop/EGO_MOTION_ESTIMATION/1_egomotion_wheel_speed/sensor_data/scene-1094/CAN/zoe_veh_signal.csv";


BOOL get_inputs(
	struct zoe_veh_signal_node** scene_data,
	veh_input_signals* input,
	uint16_t* frame_count) {

	if (*scene_data == NULL) {
		return FALSE;
	}
	else {
		input->timestamp_sec = (*scene_data)->veh_signal.timestamp_sec;
		input->FL_wheel_speed = (*scene_data)->veh_signal.FL_wheel_speed;
		input->FR_wheel_speed = (*scene_data)->veh_signal.FR_wheel_speed;
		input->RL_wheel_speed = (*scene_data)->veh_signal.RL_wheel_speed;
		input->RR_wheel_speed = (*scene_data)->veh_signal.RR_wheel_speed;
		input->steering_angle = (*scene_data)->veh_signal.steering_angle;
		(*frame_count)++;
		*scene_data = (*scene_data)->next;
		return TRUE;
	}
}


void main() {

	// load the input signals
	gt_pose_list* gt_pose = create_gt_pose_list(gt_pose_file);
	imu_signal_list* imu_signal = create_imu_signal_list(imu_signal_file);
	zoe_veh_signal_list* zoe_veh_signal = create_zoe_veh_signal_list(zoe_veh_signal_file);

	// initialized data
	noise_var_wheels noise_var_wheels_struct = init_struct_noise_var_wheels();
	zoe_spec zoe_spec_struct = init_struct_zoe_spec();
	wheel_locations wheel_loc_struct = init_wheel_locations();
	wheel_steer_angle wheel_steer_struct = init_wheel_steer_angle();
	veh_input_signals veh_input_signals_struct = init_struct_veh_input_signals();
	estimated_motion_param est_motion_param_struct = init_struct_est_motion_param();
	BOOL gated_wheel_idx[num_wheels_const] = {TRUE};

	// run the filter
	struct zoe_veh_signal_node* scene_data = zoe_veh_signal->VEH;
	uint16_t frame_count = -1;
	BOOL start = FALSE;

	while (get_inputs(&scene_data, &veh_input_signals_struct, &frame_count)) {

		convert_meas_unit(
			&veh_input_signals_struct, 
			&zoe_spec_struct);

		compute_wheel_steer_angles(
			&wheel_steer_struct, 
			&veh_input_signals_struct, 
			&zoe_spec_struct);

		gate_wheel_speed(
			&gated_wheel_idx[0],
			&start,
			&est_motion_param_struct,
			&veh_input_signals_struct,
			&wheel_steer_struct,
			&wheel_loc_struct,
			&zoe_spec_struct);

		compute_ego_motion_lse(
			&veh_input_signals_struct,
			&wheel_loc_struct,
			&wheel_steer_struct,
			&noise_var_wheels_struct,
			&gated_wheel_idx[0],
			&est_motion_param_struct);


		
		printf("frame %d success  \n", frame_count);
		printf("timestamp_sec : %f \n", est_motion_param_struct.timestamp_sec);
		printf("vx            : %f \n", est_motion_param_struct.vx);
		printf("vy            : %f \n", est_motion_param_struct.vy);
		printf("yaw rate      : %f \n", (double64_t)rad2deg_const * est_motion_param_struct.yaw_rate);

		/*
		printf("timestamp_sec : %f \n", veh_input_signals_struct.timestamp_sec);
		printf("FL_wheel_speed: %f \n", veh_input_signals_struct.FL_wheel_speed);
		printf("FR_wheel_speed: %f \n", veh_input_signals_struct.FR_wheel_speed);
		printf("RL_wheel_speed: %f \n", veh_input_signals_struct.RL_wheel_speed);
		printf("RR_wheel_speed: %f \n", veh_input_signals_struct.RR_wheel_speed);
		printf("steering_angle: %f \n", veh_input_signals_struct.steering_angle);
		*/
		printf("=======================================\n");


	}


	delete_gt_pose_list(gt_pose);
	delete_imu_signal_list(imu_signal);
	delete_zoe_veh_signal_list(zoe_veh_signal);

}