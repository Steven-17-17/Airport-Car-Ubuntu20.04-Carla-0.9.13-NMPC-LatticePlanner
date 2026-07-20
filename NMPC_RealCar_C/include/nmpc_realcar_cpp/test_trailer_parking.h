#ifndef NMPC_REALCAR_C_TEST_TRAILER_PARKING_H
#define NMPC_REALCAR_C_TEST_TRAILER_PARKING_H

#define MAX_PATH_POINTS 20000
#define MAX_REF_POINTS 40000
#define MAX_TRAILERS 4
#define MAX_NMPC_HORIZON 64

typedef struct {
    double x;
    double y;
    double yaw;
} PathPoint;

typedef struct {
    double x;
    double y;
    double yaw;
    double speed;
} VehicleState;

typedef struct {
    double x;
    double y;
    double yaw;
    int alive;
} TrailerState;

typedef struct {
    double throttle;
    double brake;
    double steer;
} ControlCommand;

typedef struct {
    int horizon;
    double dt;
    double wheelbase;
    double max_steer;
    double min_v;
    double max_v;
    int has_last_u;
    double last_delta_seq[MAX_NMPC_HORIZON];
    double last_v_seq[MAX_NMPC_HORIZON];
    double last_cost;
    int max_iterations;
    double ftol;
    int has_reference_index;
    int last_reference_index;
} NMPCController;

typedef struct {
    PathPoint reference[MAX_REF_POINTS];
    int reference_count;

    double parking_x;
    double parking_y;
    double alignment_window;
    double stop_approach_zone;
    double stop_speed_threshold;
    double stop_delay;

    int target_trailer_indices[MAX_TRAILERS];
    int target_trailer_count;
    int current_target_ptr;

    int all_tasks_done;
    int is_stopped;
    double stop_timer;
    int has_passed_parking[MAX_TRAILERS];
    int skip_until_depart[MAX_TRAILERS];

    NMPCController controller;
} ParkingRuntime;

typedef struct {
    ControlCommand command;
    int active_trailer_idx;
    int current_target_trailer_idx;
    int all_tasks_done;
    int is_stopped;
    int stop_event;
    int release_event;
    double target_speed;
    double distance_to_parking;
} ParkingStepResult;

void init_nmpc_controller(NMPCController *controller, int horizon, double dt, double wheelbase);
void init_parking_runtime(ParkingRuntime *runtime, const PathPoint *reference, int reference_count, double wheelbase);
int load_path_from_csv(const char *csv_path, PathPoint *points, int max_points);
int prepare_reference_path(const PathPoint *raw, int raw_count, PathPoint *output, int max_points);
void parking_runtime_step(
    ParkingRuntime *runtime,
    const VehicleState *vehicle,
    const TrailerState *trailers,
    int trailer_count,
    int confirm_release,
    double dt,
    ParkingStepResult *result);

#endif
