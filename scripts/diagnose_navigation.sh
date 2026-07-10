#!/usr/bin/env bash
# Diagnoza pipeline'u mapowania (point cloud -> octomap) i unikania przeszkód
# (planner -> ścieżka -> regulator PID -> alokacja ciągu -> silniki). Zbiera
# echo z najważniejszych tematów do plików, zrzuca parametry węzłów i stan
# mapy na start/koniec, a po chwili wysyła cel nawigacji jako wyzwalacz.
#
# Użycie (w środowisku z już zesourcowanym ROS2 i uruchomionym autonomy.launch.py):
#   ./diagnose_navigation.sh [czas_zbierania_w_sekundach] [x] [y] [z]
#
# Przykład:
#   ./diagnose_navigation.sh 30 14.0 0.0 -1.0

set -uo pipefail

DURATION="${1:-30}"
GOAL_X="${2:-14.0}"
GOAL_Y="${3:-0.0}"
GOAL_Z="${4:--1.0}"

LOG_DIR="./rov_diag_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

PIDS=()

start_echo() {
    local topic="$1"
    local outfile="$2"
    shift 2
    ros2 topic echo "$topic" "$@" > "$LOG_DIR/$outfile" 2>&1 &
    PIDS+=($!)
}

start_hz() {
    local topic="$1"
    local outfile="$2"
    ros2 topic hz "$topic" > "$LOG_DIR/$outfile" 2>&1 &
    PIDS+=($!)
}

dump_params() {
    local node="$1"
    ros2 param dump "$node" > "$LOG_DIR/params_$(echo "$node" | tr -d '/').yaml" 2>&1
}

save_octomap() {
    local label="$1"
    if command -v ros2 >/dev/null 2>&1 && ros2 pkg executables octomap_server 2>/dev/null | grep -q octomap_saver; then
        timeout 5 ros2 run octomap_server octomap_saver -f "$LOG_DIR/octomap_${label}.bt" --ros-args -p full:=true \
            > "$LOG_DIR/octomap_saver_${label}.log" 2>&1
    else
        echo "octomap_saver niedostępny w tym środowisku - pomijam zapis pliku .bt" > "$LOG_DIR/octomap_saver_${label}.log"
    fi
}

summarize() {
    echo ""
    echo "=== Szybkie podsumowanie ==="

    echo "--- Częstotliwość /marines_rov/point_cloud (ostatnie próbki) ---"
    tail -n 5 "$LOG_DIR/point_cloud_hz.log" 2>/dev/null || echo "(brak danych - temat nigdy nie opublikował ani jednej wiadomości)"

    echo ""
    echo "--- Częstotliwość /occupied_cells_vis_array (ostatnie próbki) ---"
    tail -n 5 "$LOG_DIR/occupied_cells_hz.log" 2>/dev/null || echo "(brak danych - octomap nic nie publikuje)"

    echo ""
    echo "--- Częstotliwość /projected_map (2D rzut mapy, ostatnie próbki) ---"
    tail -n 5 "$LOG_DIR/projected_map_hz.log" 2>/dev/null || echo "(brak danych)"

    echo ""
    echo "--- Ostatnia znana pozycja (odom -> base_link) ---"
    tail -n 8 "$LOG_DIR/tf_odom_base_link.log" 2>/dev/null

    echo ""
    echo "--- Ostatnie wartości silników (/thrusters/STONEFISH, komenda z alokatora) ---"
    tail -n 12 "$LOG_DIR/thrusters_cmd_echo.log" 2>/dev/null

    echo ""
    echo "--- Ostatnie cmd_wrench (siła/moment z regulatora PID) ---"
    tail -n 12 "$LOG_DIR/cmd_wrench_echo.log" 2>/dev/null

    echo ""
    echo "--- Ostrzeżenia/błędy w /rosout (unikalne, max 30) ---"
    grep -iE "warn|error|brak trasy|niedostępny|could not transform|dropping message|kolizj" \
        "$LOG_DIR/rosout_echo.log" 2>/dev/null | sort -u | head -30

    echo ""
    echo "Pełne logi zapisane w: $LOG_DIR"
}

cleanup() {
    echo ""
    echo "Zatrzymuję zbieranie danych..."
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" >/dev/null 2>&1
    done
    wait 2>/dev/null

    echo "Zapisuję stan mapy na koniec przebiegu..."
    save_octomap "end"

    summarize
}
trap cleanup EXIT INT TERM

echo "Logi lądują w: $LOG_DIR"
echo "Zbieranie danych przez ${DURATION}s..."

# --- Migawka parametrów węzłów (co faktycznie działa, nie co jest w plikach źródłowych) ---
for node in /octomap_server /planner_node /pid_controller_node /ekf_filter_node /thruster_allocator_node; do
    dump_params "$node"
done

echo "Zapisuję stan mapy na start przebiegu..."
save_octomap "start"

# --- Mapowanie: point cloud (Stonefish/multibeam) -> octomap -> rzut 2D ---
start_hz   /marines_rov/point_cloud       point_cloud_hz.log
start_hz   /occupied_cells_vis_array      occupied_cells_hz.log
start_hz   /projected_map                 projected_map_hz.log
start_echo /marines_rov/point_cloud       point_cloud_echo.log --no-arr
start_echo /occupied_cells_vis_array      occupied_cells_echo.log --no-arr
start_echo /projected_map                 projected_map_echo.log --no-arr

# --- Lokalizacja: surowa odometria, po konwersji NED->ENU, IMU, DVL, TF ---
start_echo /marines_rov/odometry          odometry_raw_echo.log
start_echo /marines_rov/odometry_enu      odometry_enu_echo.log
start_echo /marines_rov/imu               imu_echo.log
start_echo /marines_rov/dvl               dvl_echo.log
start_echo /marines_rov/dvl_raw           dvl_raw_echo.log
start_echo /marines_rov/altitude          altitude_echo.log
ros2 run tf2_ros tf2_echo odom base_link            > "$LOG_DIR/tf_odom_base_link.log" 2>&1 &
PIDS+=($!)
ros2 run tf2_ros tf2_echo base_link ROV/multibeam_3d > "$LOG_DIR/tf_base_link_multibeam.log" 2>&1 &
PIDS+=($!)

# --- Nawigacja: cel -> trasa RRT* -> zakończenie trasy -> siła na wyjściu PID ---
start_echo /marines_rov/goal_pose         goal_pose_echo.log
start_echo /marines_rov/global_path       global_path_echo.log --no-arr
start_echo /marines_rov/path_complete     path_complete_echo.log
start_echo /marines_rov/cmd_vel           cmd_vel_echo.log
start_echo /marines_rov/cmd_wrench        cmd_wrench_echo.log

# --- Silniki: komenda z alokatora ciągu i stan zwrotny ze Stonefisha ---
start_echo /thrusters/STONEFISH           thrusters_cmd_echo.log
start_echo /marines_rov/thruster_state    thruster_state_echo.log

# --- Logi węzłów (odpowiednik rqt_console; NIE obejmuje "print()" w kodzie -
# te trafiają tylko do terminala/pliku logu launch, nie na /rosout) ---
start_echo /rosout                        rosout_echo.log --no-arr

sleep 2  # czas na podłączenie się subskrybentów, zanim wyślemy wyzwalacz

echo "Wysyłam cel nawigacji (wyzwalacz): x=$GOAL_X y=$GOAL_Y z=$GOAL_Z"
ros2 topic pub --once /marines_rov/goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: $GOAL_Z}}
}" > "$LOG_DIR/goal_pose_publish.log" 2>&1

sleep "$DURATION"
