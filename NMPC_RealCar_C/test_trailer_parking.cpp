#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "nmpc_realcar_cpp/test_trailer_parking.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEFAULT_SMOOTH_WINDOW 5
#define DEFAULT_RESAMPLE_DS 0.1
#define DEFAULT_STOP_DELAY 1.0

static double clamp_double(double value, double lower, double upper) {
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static double normalize_angle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

static char *trim_left(char *text) {
    while (*text && isspace((unsigned char)*text)) {
        ++text;
    }
    return text;
}

static void trim_right(char *text) {
    size_t length = strlen(text);
    while (length > 0 && isspace((unsigned char)text[length - 1])) {
        text[length - 1] = '\0';
        --length;
    }
}

static int contains_x(const char *token) {
    for (const char *p = token; *p; ++p) {
        if (tolower((unsigned char)*p) == 'x') {
            return 1;
        }
    }
    return 0;
}

static int contains_y(const char *token) {
    for (const char *p = token; *p; ++p) {
        if (tolower((unsigned char)*p) == 'y') {
            return 1;
        }
    }
    return 0;
}

void init_nmpc_controller(NMPCController *controller, int horizon, double dt, double wheelbase) {
    controller->horizon = horizon;
    if (controller->horizon > MAX_NMPC_HORIZON) {
        controller->horizon = MAX_NMPC_HORIZON;
    }
    controller->dt = dt;
    controller->wheelbase = wheelbase;
    controller->max_steer = 30.0 * M_PI / 180.0;
    controller->min_v = 0.0;
    controller->max_v = 3.0;
    controller->has_last_u = 0;
    for (int i = 0; i < MAX_NMPC_HORIZON; ++i) {
        controller->last_delta_seq[i] = 0.0;
        controller->last_v_seq[i] = 0.0;
    }
    controller->last_cost = HUGE_VAL;
    controller->max_iterations = 50;
    controller->ftol = 1e-4;
    controller->has_reference_index = 0;
    controller->last_reference_index = 0;
}

void init_parking_runtime(ParkingRuntime *runtime, const PathPoint *reference, int reference_count, double wheelbase) {
    memset(runtime, 0, sizeof(*runtime));
    if (reference_count > MAX_REF_POINTS) {
        reference_count = MAX_REF_POINTS;
    }
    memcpy(runtime->reference, reference, sizeof(PathPoint) * (size_t)reference_count);
    runtime->reference_count = reference_count;

    runtime->parking_x = runtime->reference[reference_count - 1].x;
    runtime->parking_y = runtime->reference[reference_count - 1].y;
    runtime->alignment_window = 10.0;
    runtime->stop_approach_zone = 0.45;
    runtime->stop_speed_threshold = 0.1;
    runtime->stop_delay = DEFAULT_STOP_DELAY;

    runtime->target_trailer_count = MAX_TRAILERS;
    for (int i = 0; i < MAX_TRAILERS; ++i) {
        runtime->target_trailer_indices[i] = i + 1;
        runtime->has_passed_parking[i] = 0;
        runtime->skip_until_depart[i] = 0;
    }

    runtime->current_target_ptr = 0;
    runtime->all_tasks_done = 0;
    runtime->is_stopped = 0;
    runtime->stop_timer = 0.0;

    init_nmpc_controller(&runtime->controller, 25, 0.05, wheelbase);
}

int load_path_from_csv(const char *csv_path, PathPoint *points, int max_points) {
    FILE *file = fopen(csv_path, "r");
    if (file == NULL) {
        fprintf(stderr, "Failed to open CSV: %s\n", csv_path);
        return -1;
    }

    char line[4096];
    if (fgets(line, sizeof(line), file) == NULL) {
        fclose(file);
        fprintf(stderr, "CSV is empty: %s\n", csv_path);
        return -1;
    }

    int x_index = -1;
    int y_index = -1;
    int column_count = 0;

    char header_copy[4096];
    strncpy(header_copy, line, sizeof(header_copy) - 1);
    header_copy[sizeof(header_copy) - 1] = '\0';

    char *token = strtok(header_copy, ",\r\n");
    while (token != NULL) {
        char *clean = trim_left(token);
        trim_right(clean);
        if (x_index < 0 && contains_x(clean)) {
            x_index = column_count;
        }
        if (y_index < 0 && contains_y(clean)) {
            y_index = column_count;
        }
        ++column_count;
        token = strtok(NULL, ",\r\n");
    }

    if (x_index < 0 || y_index < 0) {
        x_index = 0;
        y_index = 1;
    }

    int point_count = 0;
    while (fgets(line, sizeof(line), file) != NULL) {
        if (point_count >= max_points) {
            break;
        }

        char row_copy[4096];
        strncpy(row_copy, line, sizeof(row_copy) - 1);
        row_copy[sizeof(row_copy) - 1] = '\0';

        int current_column = 0;
        double x_value = 0.0;
        double y_value = 0.0;
        int has_x = 0;
        int has_y = 0;

        token = strtok(row_copy, ",\r\n");
        while (token != NULL) {
            char *clean = trim_left(token);
            trim_right(clean);

            if (current_column == x_index) {
                x_value = strtod(clean, NULL);
                has_x = 1;
            }
            if (current_column == y_index) {
                y_value = strtod(clean, NULL);
                has_y = 1;
            }

            ++current_column;
            token = strtok(NULL, ",\r\n");
        }

        if (has_x && has_y) {
            points[point_count].x = x_value;
            points[point_count].y = y_value;
            points[point_count].yaw = 0.0;
            ++point_count;
        }
    }

    fclose(file);
    return point_count;
}

static void smooth_xy_path(const PathPoint *input, int count, int window_size, PathPoint *output) {
    if (count <= 0) {
        return;
    }

    if (window_size < 3 || count < window_size) {
        memcpy(output, input, sizeof(PathPoint) * (size_t)count);
        return;
    }

    int half_window = window_size / 2;
    for (int i = 0; i < count; ++i) {
        if (i < half_window || i >= count - half_window) {
            output[i] = input[i];
            continue;
        }

        double sum_x = 0.0;
        double sum_y = 0.0;
        for (int j = i - half_window; j <= i + half_window; ++j) {
            sum_x += input[j].x;
            sum_y += input[j].y;
        }

        output[i].x = sum_x / (double)window_size;
        output[i].y = sum_y / (double)window_size;
        output[i].yaw = 0.0;
    }
}

static void unwrap_angles(const double *wrapped, double *unwrapped, int count) {
    if (count <= 0) {
        return;
    }

    unwrapped[0] = wrapped[0];
    for (int i = 1; i < count; ++i) {
        double delta = wrapped[i] - wrapped[i - 1];
        while (delta > M_PI) {
            delta -= 2.0 * M_PI;
        }
        while (delta < -M_PI) {
            delta += 2.0 * M_PI;
        }
        unwrapped[i] = unwrapped[i - 1] + delta;
    }
}

static void smooth_scalar_sequence(const double *input, double *output, int count, int window_size) {
    if (count <= 0) {
        return;
    }

    if (window_size < 3 || count < window_size) {
        memcpy(output, input, sizeof(double) * (size_t)count);
        return;
    }

    int half_window = window_size / 2;
    for (int i = 0; i < count; ++i) {
        if (i < half_window || i >= count - half_window) {
            output[i] = input[i];
            continue;
        }

        double sum = 0.0;
        for (int j = i - half_window; j <= i + half_window; ++j) {
            sum += input[j];
        }
        output[i] = sum / (double)window_size;
    }
}

static int resample_path_linear(const PathPoint *input, int count, double ds, PathPoint *output, int max_points) {
    if (count < 2 || ds <= 0.0) {
        return 0;
    }

    double arc_length[MAX_PATH_POINTS];
    arc_length[0] = 0.0;
    for (int i = 1; i < count; ++i) {
        double dx = input[i].x - input[i - 1].x;
        double dy = input[i].y - input[i - 1].y;
        arc_length[i] = arc_length[i - 1] + hypot(dx, dy);
    }

    double total_length = arc_length[count - 1];
    if (total_length <= 0.0) {
        return 0;
    }

    int output_count = (int)floor(total_length / ds) + 1;
    if (output_count > max_points) {
        output_count = max_points;
    }

    int segment = 0;
    for (int i = 0; i < output_count; ++i) {
        double target_s = ds * (double)i;
        if (target_s > total_length) {
            target_s = total_length;
        }

        while (segment < count - 2 && arc_length[segment + 1] < target_s) {
            ++segment;
        }

        double seg_start = arc_length[segment];
        double seg_end = arc_length[segment + 1];
        double ratio = 0.0;
        if (seg_end > seg_start) {
            ratio = (target_s - seg_start) / (seg_end - seg_start);
        }

        output[i].x = input[segment].x + ratio * (input[segment + 1].x - input[segment].x);
        output[i].y = input[segment].y + ratio * (input[segment + 1].y - input[segment].y);
        output[i].yaw = 0.0;
    }

    if (output_count >= 2) {
        double raw_yaw[MAX_REF_POINTS];
        double unwrapped[MAX_REF_POINTS];
        double smoothed[MAX_REF_POINTS];

        for (int i = 0; i < output_count; ++i) {
            double dx = 0.0;
            double dy = 0.0;

            if (i == 0) {
                dx = output[1].x - output[0].x;
                dy = output[1].y - output[0].y;
            } else if (i == output_count - 1) {
                dx = output[output_count - 1].x - output[output_count - 2].x;
                dy = output[output_count - 1].y - output[output_count - 2].y;
            } else {
                dx = output[i + 1].x - output[i - 1].x;
                dy = output[i + 1].y - output[i - 1].y;
            }

            raw_yaw[i] = atan2(dy, dx);
        }

        unwrap_angles(raw_yaw, unwrapped, output_count);
        smooth_scalar_sequence(unwrapped, smoothed, output_count, 7);

        for (int i = 0; i < output_count; ++i) {
            output[i].yaw = normalize_angle(smoothed[i]);
        }
    }

    return output_count;
}

int prepare_reference_path(const PathPoint *raw, int raw_count, PathPoint *output, int max_points) {
    PathPoint smoothed[MAX_PATH_POINTS];
    if (raw_count <= 0) {
        return 0;
    }

    smooth_xy_path(raw, raw_count, DEFAULT_SMOOTH_WINDOW, smoothed);
    return resample_path_linear(smoothed, raw_count, DEFAULT_RESAMPLE_DS, output, max_points);
}

static int find_closest_reference_index(const PathPoint *reference, int reference_count, double x, double y) {
    int best_index = 0;
    double best_distance_sq = HUGE_VAL;
    for (int i = 0; i < reference_count; ++i) {
        double dx = reference[i].x - x;
        double dy = reference[i].y - y;
        double distance_sq = dx * dx + dy * dy;
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            best_index = i;
        }
    }
    return best_index;
}

static int find_tracking_reference_index(
    NMPCController *controller,
    const PathPoint *reference,
    int reference_count,
    double x,
    double y
) {
    if (!controller->has_reference_index) {
        int index = find_closest_reference_index(reference, reference_count, x, y);
        controller->has_reference_index = 1;
        controller->last_reference_index = index;
        return index;
    }

    int start_index = controller->last_reference_index - 20;
    int end_index = controller->last_reference_index + 250;
    if (start_index < 0) {
        start_index = 0;
    }
    if (end_index >= reference_count) {
        end_index = reference_count - 1;
    }

    int best_index = controller->last_reference_index;
    double best_distance_sq = HUGE_VAL;
    for (int i = start_index; i <= end_index; ++i) {
        double dx = reference[i].x - x;
        double dy = reference[i].y - y;
        double distance_sq = dx * dx + dy * dy;
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            best_index = i;
        }
    }

    if (best_distance_sq > 100.0) {
        best_index = find_closest_reference_index(reference, reference_count, x, y);
    }

    if (best_index > controller->last_reference_index) {
        controller->last_reference_index = best_index;
    }
    return controller->last_reference_index;
}

static double evaluate_sequence_cost(
    const NMPCController *controller,
    const VehicleState *state,
    const PathPoint *reference,
    int reference_count,
    int start_index,
    const double *delta_seq,
    const double *v_seq,
    double target_speed
) {
    VehicleState predicted = *state;
    double cost = 0.0;
    double accumulated_s = 0.0;
    const double reference_step = DEFAULT_RESAMPLE_DS;

    for (int i = 0; i < controller->horizon; ++i) {
        const double delta_cmd = delta_seq[i];
        const double v_cmd = v_seq[i];

        if (v_cmd < 0.1) {
            cost += 1000.0 * (0.1 - v_cmd) * (0.1 - v_cmd);
        }

        predicted.speed = v_cmd;
        predicted.x += predicted.speed * cos(predicted.yaw) * controller->dt;
        predicted.y += predicted.speed * sin(predicted.yaw) * controller->dt;
        predicted.yaw = normalize_angle(predicted.yaw + predicted.speed * tan(delta_cmd) / controller->wheelbase * controller->dt);

        accumulated_s += predicted.speed * controller->dt;
        int target_index = start_index + (int)lround(accumulated_s / reference_step);
        if (target_index < 0) {
            target_index = 0;
        }
        if (target_index >= reference_count) {
            target_index = reference_count - 1;
        }

        double front_x = predicted.x + controller->wheelbase * cos(predicted.yaw);
        double front_y = predicted.y + controller->wheelbase * sin(predicted.yaw);

        double dx = front_x - reference[target_index].x;
        double dy = front_y - reference[target_index].y;
        double cte = dx * (-sin(reference[target_index].yaw)) + dy * cos(reference[target_index].yaw);
        double yaw_error = normalize_angle(predicted.yaw - reference[target_index].yaw);

        cost += 10.0 * cte * cte;
        cost += 20.0 * yaw_error * yaw_error;
        cost += 50.0 * (v_cmd - target_speed) * (v_cmd - target_speed);

        if (i > 0) {
            double delta_diff = delta_seq[i] - delta_seq[i - 1];
            double v_diff = v_seq[i] - v_seq[i - 1];
            cost += 30.0 * delta_diff * delta_diff;
            cost += 10.0 * v_diff * v_diff;
        } else {
            cost += 50.0 * delta_cmd * delta_cmd;
        }
    }

    return cost;
}

static void clamp_control_sequence(
    const NMPCController *controller,
    double *delta_seq,
    double *v_seq,
    double v_min_bound
) {
    for (int i = 0; i < controller->horizon; ++i) {
        delta_seq[i] = clamp_double(delta_seq[i], -controller->max_steer, controller->max_steer);
        v_seq[i] = clamp_double(v_seq[i], v_min_bound, controller->max_v);
    }
}

static double optimize_control_sequence(
    NMPCController *controller,
    const VehicleState *state,
    const PathPoint *reference,
    int reference_count,
    int nearest_index,
    double target_speed,
    double v_min_bound,
    double *delta_seq,
    double *v_seq
) {
    clamp_control_sequence(controller, delta_seq, v_seq, v_min_bound);
    double best_cost = evaluate_sequence_cost(
        controller,
        state,
        reference,
        reference_count,
        nearest_index,
        delta_seq,
        v_seq,
        target_speed
    );

    double delta_step = controller->max_steer * 0.25;
    double v_step = 0.5;

    for (int iteration = 0; iteration < controller->max_iterations; ++iteration) {
        double iteration_start_cost = best_cost;
        int improved = 0;

        for (int i = 0; i < controller->horizon; ++i) {
            double best_delta_value = delta_seq[i];
            double best_v_value = v_seq[i];

            for (int sign = -1; sign <= 1; sign += 2) {
                delta_seq[i] = clamp_double(best_delta_value + (double)sign * delta_step, -controller->max_steer, controller->max_steer);
                double trial_cost = evaluate_sequence_cost(
                    controller,
                    state,
                    reference,
                    reference_count,
                    nearest_index,
                    delta_seq,
                    v_seq,
                    target_speed
                );
                if (trial_cost + controller->ftol < best_cost) {
                    best_cost = trial_cost;
                    best_delta_value = delta_seq[i];
                    improved = 1;
                } else {
                    delta_seq[i] = best_delta_value;
                }
            }

            for (int sign = -1; sign <= 1; sign += 2) {
                v_seq[i] = clamp_double(best_v_value + (double)sign * v_step, v_min_bound, controller->max_v);
                double trial_cost = evaluate_sequence_cost(
                    controller,
                    state,
                    reference,
                    reference_count,
                    nearest_index,
                    delta_seq,
                    v_seq,
                    target_speed
                );
                if (trial_cost + controller->ftol < best_cost) {
                    best_cost = trial_cost;
                    best_v_value = v_seq[i];
                    improved = 1;
                } else {
                    v_seq[i] = best_v_value;
                }
            }
        }

        if (!improved) {
            delta_step *= 0.5;
            v_step *= 0.5;
        }

        if (fabs(iteration_start_cost - best_cost) < controller->ftol && delta_step < 1e-4 && v_step < 1e-3) {
            break;
        }
    }

    return best_cost;
}

static ControlCommand solve_nmpc_step(
    NMPCController *controller,
    const VehicleState *state,
    const PathPoint *reference,
    int reference_count,
    double target_speed
) {
    ControlCommand command;
    command.throttle = 0.0;
    command.brake = 0.0;
    command.steer = 0.0;

    if (reference_count < 2) {
        return command;
    }

    int nearest_index = find_tracking_reference_index(
        controller,
        reference,
        reference_count,
        state->x + controller->wheelbase * cos(state->yaw),
        state->y + controller->wheelbase * sin(state->yaw)
    );

    double delta_seq[MAX_NMPC_HORIZON];
    double v_seq[MAX_NMPC_HORIZON];
    double v_target = target_speed;
    double v_min_bound = (v_target < 0.3) ? 0.0 : fmax(controller->min_v, 0.1);

    if (!controller->has_last_u) {
        for (int i = 0; i < controller->horizon; ++i) {
            delta_seq[i] = 0.0;
            v_seq[i] = fmax(v_target, 0.5);
        }
    } else {
        for (int i = 0; i < controller->horizon - 1; ++i) {
            delta_seq[i] = controller->last_delta_seq[i + 1];
            v_seq[i] = controller->last_v_seq[i + 1];
        }
        delta_seq[controller->horizon - 1] = controller->last_delta_seq[controller->horizon - 1];
        v_seq[controller->horizon - 1] = controller->last_v_seq[controller->horizon - 1];
    }

    double best_cost = optimize_control_sequence(
        controller,
        state,
        reference,
        reference_count,
        nearest_index,
        v_target,
        v_min_bound,
        delta_seq,
        v_seq
    );

    double best_steer = delta_seq[0];
    for (int i = 0; i < controller->horizon; ++i) {
        controller->last_delta_seq[i] = delta_seq[i];
        controller->last_v_seq[i] = v_seq[i];
    }
    controller->last_cost = best_cost;
    controller->has_last_u = 1;

    double steer_out = best_steer / controller->max_steer;
    steer_out = clamp_double(steer_out, -1.0, 1.0);

    double current_speed = state->speed;
    double speed_error = target_speed - current_speed;

    if (target_speed < 0.3) {
        command.throttle = 0.0;
        command.brake = 1.0;
    } else {
        if (speed_error > 0.0) {
            command.throttle = 0.3 + speed_error * 2.0;
            if (current_speed < 1.0 && target_speed > 1.5) {
                command.throttle = fmax(command.throttle, 0.7);
            }
            command.brake = 0.0;
        } else {
            command.throttle = 0.0;
            command.brake = -speed_error * 0.8;
        }

        command.throttle = clamp_double(command.throttle, 0.0, 1.0);
        command.brake = clamp_double(command.brake, 0.0, 1.0);
    }

    command.steer = steer_out;
    return command;
}

void parking_runtime_step(
    ParkingRuntime *runtime,
    const VehicleState *vehicle,
    const TrailerState *trailers,
    int trailer_count,
    int confirm_release,
    double dt,
    ParkingStepResult *result
) {
    memset(result, 0, sizeof(*result));
    result->distance_to_parking = HUGE_VAL;

    if (runtime->current_target_ptr >= runtime->target_trailer_count) {
        runtime->all_tasks_done = 1;
    }

    int active_trailer_idx = 0;
    int current_target_trailer_idx = -1;

    if (!runtime->all_tasks_done && runtime->current_target_ptr < runtime->target_trailer_count) {
        current_target_trailer_idx = runtime->target_trailer_indices[runtime->current_target_ptr];
        int trailer_array_index = current_target_trailer_idx - 1;
        if (trailer_array_index >= 0 && trailer_array_index < trailer_count && trailers[trailer_array_index].alive) {
            double dx = trailers[trailer_array_index].x - runtime->parking_x;
            double dy = trailers[trailer_array_index].y - runtime->parking_y;
            result->distance_to_parking = hypot(dx, dy);
            if (result->distance_to_parking < runtime->alignment_window) {
                active_trailer_idx = current_target_trailer_idx;
            }
        }
    }

    result->active_trailer_idx = active_trailer_idx;
    result->current_target_trailer_idx = current_target_trailer_idx;
    result->all_tasks_done = runtime->all_tasks_done;
    result->is_stopped = runtime->is_stopped;

    if (runtime->all_tasks_done) {
        result->target_speed = 2.5;
        result->command = solve_nmpc_step(&runtime->controller, vehicle, runtime->reference, runtime->reference_count, result->target_speed);
        return;
    }

    if (runtime->is_stopped) {
        result->command.throttle = 0.0;
        result->command.brake = 1.0;
        result->command.steer = 0.0;
        result->target_speed = 0.0;

        runtime->stop_timer += dt;
        if (runtime->stop_timer >= runtime->stop_delay && confirm_release) {
            result->release_event = 1;
            runtime->stop_timer = 0.0;
            runtime->is_stopped = 0;
            runtime->controller.has_last_u = 0;

            if (runtime->current_target_ptr < runtime->target_trailer_count - 1) {
                ++runtime->current_target_ptr;
                int next_trailer_idx = runtime->target_trailer_indices[runtime->current_target_ptr];
                int next_array_index = next_trailer_idx - 1;
                if (next_array_index >= 0 && next_array_index < trailer_count && trailers[next_array_index].alive) {
                    double dx = trailers[next_array_index].x - runtime->parking_x;
                    double dy = trailers[next_array_index].y - runtime->parking_y;
                    double next_distance = hypot(dx, dy);
                    if (next_distance < runtime->stop_approach_zone) {
                        runtime->has_passed_parking[runtime->current_target_ptr] = 1;
                        runtime->skip_until_depart[runtime->current_target_ptr] = 1;
                    }
                }
            } else {
                runtime->all_tasks_done = 1;
            }
        }
        return;
    }

    double target_speed = 2.5;
    int current_index = runtime->current_target_ptr;

    if (current_index >= 0 && current_index < runtime->target_trailer_count) {
        if (runtime->skip_until_depart[current_index]) {
            if (result->distance_to_parking > runtime->stop_approach_zone * 2.0) {
                runtime->skip_until_depart[current_index] = 0;
                runtime->has_passed_parking[current_index] = 0;
            }
            target_speed = 1.39;
        } else if (!runtime->has_passed_parking[current_index] && result->distance_to_parking < runtime->stop_approach_zone) {
            target_speed = 0.0;
            if (vehicle->speed < runtime->stop_speed_threshold) {
                runtime->has_passed_parking[current_index] = 1;
                runtime->is_stopped = 1;
                runtime->stop_timer = 0.0;
                result->stop_event = 1;
            }
        } else {
            target_speed = (active_trailer_idx > 0) ? 1.39 : 2.5;
        }
    }

    result->target_speed = target_speed;
    result->command = solve_nmpc_step(&runtime->controller, vehicle, runtime->reference, runtime->reference_count, target_speed);
    result->all_tasks_done = runtime->all_tasks_done;
    result->is_stopped = runtime->is_stopped;
}

#ifndef NMPC_REALCAR_C_LIBRARY_ONLY
static void print_usage(const char *program_name) {
    printf("Usage: %s [path_csv]\n", program_name);
    printf("This C++ file is the controller core for ROS1/CARLA integration.\n");
    printf("It loads and preprocesses the waypoint CSV, then exposes a state-machine step function\n");
    printf("that you can call from a ROS1 bridge or a real-vehicle adapter.\n");
}

int main(int argc, char **argv) {
    const char *csv_path = NULL;
    if (argc > 1) {
        csv_path = argv[1];
    } else {
        csv_path = "user_waypoints.csv";
    }

    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        print_usage(argv[0]);
        return 0;
    }

    PathPoint raw_path[MAX_PATH_POINTS];
    PathPoint reference_path[MAX_REF_POINTS];

    int raw_count = load_path_from_csv(csv_path, raw_path, MAX_PATH_POINTS);
    if (raw_count <= 0) {
        fprintf(stderr, "Failed to load path from %s\n", csv_path);
        return 1;
    }

    int reference_count = prepare_reference_path(raw_path, raw_count, reference_path, MAX_REF_POINTS);
    if (reference_count <= 0) {
        fprintf(stderr, "Failed to prepare reference path\n");
        return 1;
    }

    double first_x = reference_path[0].x;
    double first_y = reference_path[0].y;
    double last_x = reference_path[reference_count - 1].x;
    double last_y = reference_path[reference_count - 1].y;

    printf("Loaded %d raw points from %s\n", raw_count, csv_path);
    printf("Prepared %d reference points after smoothing and resampling\n", reference_count);
    printf("First point: (%.6f, %.6f)\n", first_x, first_y);
    printf("Last point : (%.6f, %.6f)\n", last_x, last_y);
    printf("\n");
    print_usage(argv[0]);
    printf("\n");
    printf("To use this with ROS1 + CARLA, embed parking_runtime_step() inside your adapter loop.\n");
    printf("Subscribe vehicle state, trailer poses, and a manual release/advance flag, then publish ControlCommand.\n");

    return 0;
}
#endif
