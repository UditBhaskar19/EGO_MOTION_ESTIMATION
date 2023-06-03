# ifndef _LOADCSV_H_
# define _LOADCSV_H_

# include "datatypes.h"

/* ---------------------------------------------------------------------
 *                 Structures to load the csv files
 * --------------------------------------------------------------------- */

typedef struct gt_pose {
    REAL timestamp_sec;
    REAL px, py, pz;
    REAL vx, vy, vz;
    REAL ax, ay, az;
    REAL q1, q2, q3, q4;
    REAL rot_rate_x, rot_rate_y, rot_rate_z;
} gt_pose;


typedef struct imu_signal {
    REAL timestamp_sec;
    REAL ax, ay, az;
    REAL q1, q2, q3, q4;
    REAL rot_rate_x, rot_rate_y, rot_rate_z;
} imu_signal;


typedef struct zoe_veh_signal {
    REAL timestamp_sec;
    REAL FL_wheel_speed, FR_wheel_speed, RR_wheel_speed, RL_wheel_speed;
    REAL steering_angle;
    REAL odom_speed, odom;
    REAL ax, ay;
} zoe_veh_signal;

/* ----------------------------------------------------------------------------------------------
 *               Load the csv file for the entire scene in the form of linked list
 *                                        Node structure
 * ---------------------------------------------------------------------------------------------- */

struct gt_pose_node {
    gt_pose pose;
    struct gt_pose_node* next;
};

struct imu_signal_node {
    imu_signal imu;
    struct imu_signal_node* next;
};

struct zoe_veh_signal_node {
    zoe_veh_signal veh_signal;
    struct zoe_veh_signal_node* next;
};

/* ----------------------------------------------------------------------------------------------
 *                                      Linked List structure
 * ---------------------------------------------------------------------------------------------- */

typedef struct gt_pose_list {
    struct gt_pose_node* POSE;
    INTEGER num_ele;
}gt_pose_list;

typedef struct imu_signal_list {
    struct imu_signal_node* IMU;
    INTEGER num_ele;
}imu_signal_list;

typedef struct zoe_veh_signal_list {
    struct zoe_veh_signal_node* VEH;
    INTEGER num_ele;
}zoe_veh_signal_list;



/* ----------------------------------------------------------------------------------------------
 *                                      Function prototypes
 * ---------------------------------------------------------------------------------------------- */

void init_gt_pose_list(gt_pose_list* pose);

void set_gt_pose_struct(char* line, gt_pose* pose);

struct gt_pose_node* create_new_gt_pose_node(char* line);

void set_gt_pose_list(const char* filepath, gt_pose_list* pose);

void append_gt_pose(struct gt_pose_node** list, struct gt_pose_node* new_node);

gt_pose_list* create_gt_pose_list(const char* filepath);

void delete_gt_pose(struct gt_pose_node** list);

void delete_gt_pose_list(gt_pose_list* gt_pose);

void print_nodes_gt_pose(struct gt_pose_node* list);




void init_imu_signal_list(imu_signal_list* imu);

void set_imu_signal_struct(char* line, imu_signal* imu);

struct imu_signal_node* create_new_imu_signal_node(char* line);

void set_imu_signal_list(const char* filepath, imu_signal_list* imu);

void append_imu_signal(struct imu_signal_node** list, struct imu_signal_node* new_node);

imu_signal_list* create_imu_signal_list(const char* filepath);

void delete_imu_signal(struct imu_signal_node** list);

void delete_imu_signal_list(imu_signal_list* imu_signal);

void print_nodes_imu_signal(struct imu_signal_node* list);




void init_zoe_veh_signal_list(zoe_veh_signal_list* zoe_veh);

void set_zoe_veh_signal_struct(char* line, zoe_veh_signal* zoe_veh);

struct zoe_veh_signal_node* create_new_zoe_veh_signal_node(char* line);

void set_zoe_veh_signal_list(const char* filepath, zoe_veh_signal_list* zoe_veh);

void append_zoe_veh_signal(struct zoe_veh_signal_node** list, struct zoe_veh_signal_node* new_node);

zoe_veh_signal_list* create_zoe_veh_signal_list(const char* filepath);

void delete_zoe_veh_signal(struct zoe_veh_signal_node** list);

void delete_zoe_veh_signal_list(zoe_veh_signal_list* zoe_veh);

void print_nodes_veh_signal(struct zoe_veh_signal_node* list);



# endif
