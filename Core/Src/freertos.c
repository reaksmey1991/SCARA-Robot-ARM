/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS Task definitions for Dual RMD Motor (M1 & M2)
  *                      Limit-Switch Homing + Trapezoidal Motion Profile (Motion.c)
  *                      + RMD Built-in Position Control (0xA4) over FDCAN.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "RMD_Motor.h"
#include "MKS_Servo.h"
#include "Motion.h"
#include "CanFilter.h"
#include "fdcan.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    MOTOR_STATE_HOMING_SEARCH = 0, // Rotating slowly in reverse until Limit Switch == 0
    MOTOR_STATE_HOMING_SETTLE,     // Stopped at switch, reading multi-turn angle to latch 0 deg
    MOTOR_STATE_POSITION_READY     // Homing complete, tracking target angle via Motion.c + RMD 0xA4
} Motor_Homing_State_t;

typedef struct {
    FDCAN_HandleTypeDef    *hfdcan;             // FDCAN handle (&hfdcan1 or &hfdcan2)
    CAN_Bus_t               bus;                // CAN_BUS_I or CAN_BUS_II
    uint16_t                can_id;             // RMD CAN ID (0x141 for M1, 0x142 for M2)
    uint8_t                 fb_index;           // Feedback index (0 for 0x141, 1 for 0x142)
    GPIO_TypeDef           *limit_port;         // GPIO port for Limit Switch (GPIOD)
    uint16_t                limit_pin;          // GPIO pin for Limit Switch (GPIO_PIN_6 / GPIO_PIN_7)
    Motor_Homing_State_t    state;              // Current homing / control state
    uint32_t                settle_start_tick;  // Tick when limit switch was hit
    float                   zero_offset_deg;    // Motor internal multi-turn angle at Limit Switch (0 deg reference)
    float                   last_target_deg;    // Previous target angle to detect target changes
    uint8_t                 profile_planned;    // Flag indicating motion profile is initialized
    uint8_t                 has_moved;          // 1 after user commands first non-zero target
    uint32_t                move_start_tick;    // Tick when current trapezoidal move started
    struct motion_profile   motion;             // Trapezoidal motion profile instance (Motion.c)
    motion_profile_state_t  profile_state;      // Current trapezoidal profile phase
    float                   pos_sp_deg;         // Instantaneous position setpoint relative to 0 (deg)
    float                   vel_sp_dps;         // Instantaneous velocity setpoint (deg/s)
    float                   acc_sp_dps2;        // Instantaneous acceleration setpoint (deg/s^2)
} RMD_Axis_t;

typedef enum {
    SCARA_STATE_INIT_CALIBRATE = 0, // 1. Startup calibration of M1, M2, and MKS #1
    SCARA_STATE_IDLE_READY,         // 2. Ready: press state_start=1 to free motors & enter recording
    SCARA_STATE_TEACH_RECORD,       // 3. Free motor mode: press state_record=1 to save Position 1, 2, ..., (n+1)
    SCARA_STATE_WAIT_START_HOLD,    // 4. state_stop_record=1 pressed: hold state_start=1 for 1s to start playback
    SCARA_STATE_PLAY_CALIBRATE,     // 5. Calibrate all motors (M1, M2, MKS #1) first before playback
    SCARA_STATE_PLAY_MOVE_POS,      // 6. Move M1, M2, and MKS (5000*n) to Position (n+1) and wait until DONE
    SCARA_STATE_PLAY_ROTATE_MSK2    // 7. Stop at Position (n+1) and wait until MSK2 rotates to 3600 or 0 deg
} Scara_Process_State_t;

typedef struct {
    float m1_deg;                   // Recorded M1 angle (deg) for Position (n+1)
    float m2_deg;                   // Recorded M2 angle (deg) for Position (n+1)
    float mks_deg;                  // MKS_pos_sp_deg = 5000 * n for Position (n+1)
} Scara_Waypoint_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RMD_MOTOR_M1_ID      (RMD_MOTOR_ID_BASE + 0) // 0x141 (Motor M1)
#define RMD_MOTOR_M2_ID      (RMD_MOTOR_ID_BASE + 1) // 0x142 (Motor M2 on FDCAN1)

#define MKS_LIMIT_PORT       GPIOD
#define MKS_LIMIT_PIN        GPIO_PIN_5              // D5: Normal = 1, Hit = 0

#define M1_LIMIT_PORT        GPIOD
#define M1_LIMIT_PIN         GPIO_PIN_6              // D6: Normal = 1, Hit = 0

#define M2_LIMIT_PORT        GPIOD
#define M2_LIMIT_PIN         GPIO_PIN_7              // D7: Normal = 1, Hit = 0

#define CONTROL_PERIOD_MS    10                      // 10ms control loop (100 Hz)
#define HOMING_STOP_MS       50                      // First 50ms in settle: stop motor
#define HOMING_SETTLE_MS     150                     // Total settle time before latching 0 deg

#define SCARA_MAX_WAYPOINTS  32                      // Maximum number of recorded SCARA positions
#define SCARA_MKS_STEP_DEG   11000.0f                 // Position (n+1) => MKS_pos_sp_deg = 5000 * n
#define SCARA_MSK2_ALT_DEG   36000.0f                 // MSK2 alternates 0 -> 3600 -> 0 deg after each position
#define SCARA_HOLD_START_MS  1000u                   // Hold state_start = 1 for 1000ms (1s) to start playback

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* ==================== User Control Targets (Live Expressions) ==================== */
volatile float    M1_target             = 0.0f;   // Target angle for Motor M1 in deg (clamped to [M1_min_deg, M1_max_deg])
volatile float    M2_target             = 0.0f;   // Target angle for Motor M2 in deg (clamped to [M2_min_deg, M2_max_deg])

/* ==================== Input Position Limits (deg) ==================== */
volatile float    M1_min_deg            = 0.0f;   // Minimum allowed target for M1 (0.0 = Limit Switch D6)
volatile float    M1_max_deg            = 2080.0f; // Maximum allowed target for M1 (deg)
volatile float    M2_min_deg            = 0.0f;   // Minimum allowed target for M2 (0.0 = Limit Switch D7)
volatile float    M2_max_deg            = 1050.0f; // Maximum allowed target for M2 (deg)

/* ==================== Direction Inversion (0 = Default, 1 = Inverted for Mechanical Robot) ==================== */
volatile uint8_t  M1_invert_dir         = 0;      // 0 = default, 1 = invert M1 homing & positive target direction
volatile uint8_t  M2_invert_dir         = 0;      // 0 = default, 1 = invert M2 homing & positive target direction

/*
 * Set free_rotate_mode = 1 after homing to release motor torque and rotate M1/M2/MKS by hand
 * to read M1_current_angle_deg / M2_current_angle_deg / MKS_current_angle_deg
 * (and peak M1_max_measured_deg / M2_max_measured_deg / MKS_max_measured_deg)
 * in Live Expressions to find your mechanical travel limits.
 * Set free_rotate_mode = 0 for normal closed-loop position control.
 */
volatile uint8_t  free_rotate_mode      = 0;
volatile float    M1_max_measured_deg   = 0.0f;   // Peak angle measured from 0 deg on M1
volatile float    M2_max_measured_deg   = 0.0f;   // Peak angle measured from 0 deg on M2
volatile float    MKS_max_measured_deg  = 0.0f;   // Peak angle measured from 0 deg on MKS
volatile float    MSK2_max_measured_deg = 0.0f;   // Peak angle measured from 0 deg on MSK2_stepper

/* ==================== Homing & Motion Profile Parameters ==================== */
volatile float    M1_homing_speed_dps   = 120.0f;  // M1 slow reverse speed during homing (deg/s)
volatile float    M2_homing_speed_dps   = 120.0f;  // M2 slow reverse speed during homing (deg/s)
volatile uint16_t MKS_homing_speed_rpm  = 80;     // MKS slow speed during homing toward D4 (RPM)
volatile float    profile_max_speed_dps = 900.0f; // Trapezoidal max speed (deg/s)
volatile float    profile_max_acc_dps2  = 1000.0f; // Trapezoidal max acceleration (deg/s^2)
volatile float    profile_max_dec_dps2  = 1000.0f; // Trapezoidal max deceleration (deg/s^2)
volatile uint16_t pos_max_speed_dps     = 1500;    // Hardware speed limit sent to RMD 0xA4 (dps)
volatile uint8_t  rehome_trigger        = 0;      // Set to 1 in Live Expressions to re-home M1, M2 & MKS
volatile uint8_t  MKS_rehome_trigger    = 0;      // Set to 1 in Live Expressions to re-home ONLY MKS

/*
 * Set M2_use_fdcan2 = 1 if you cannot change M2's ID and instead connect M2 (ID 1 / 0x141)
 * to FDCAN2 (PB12/PB13) while M1 (ID 1 / 0x141) is on FDCAN1 (PD0/PB9).
 * Leave M2_use_fdcan2 = 0 if both M1 (0x141) and M2 (0x142) are on FDCAN1.
 */
volatile uint8_t  M2_use_fdcan2         = 0;

/* ==================== Motor ID Programming (Live Expressions) ==================== */
volatile uint8_t  set_new_motor_id      = 0;      // Set to 2 (with ONLY M2 connected) to program M2's ID to 2
volatile uint8_t  id_write_done         = 0;      // Becomes 1 after new ID is written (halts bus until power-cycle)

/* ==================== Limit Switch & Homing Status (Live Expressions) ==================== */
volatile uint8_t  D5_state              = 1;      // MKS limit switch pin state (PD4: 1 = normal, 0 = hit)
volatile uint8_t  D6_state              = 1;      // M1 limit switch pin state  (PD6: 1 = normal, 0 = hit)
volatile uint8_t  D7_state              = 1;      // M2 limit switch pin state  (PD7: 1 = normal, 0 = hit)
volatile uint8_t  M1_homed              = 0;      // 1 when M1 has hit D6 and latched 0 deg
volatile uint8_t  M2_homed              = 0;      // 1 when M2 has hit D7 and latched 0 deg
volatile uint8_t  MKS_homed             = 0;      // 1 when MKS has hit D4 and latched 0 deg

/* ==================== Live Setpoints & Feedback (Live Expressions) ==================== */
volatile float    M1_pos_sp_deg          = 0.0f;   // M1 current profile position setpoint (0 = switch D6)
volatile float    M2_pos_sp_deg          = 0.0f;   // M2 current profile position setpoint (0 = switch D7)
volatile float    MKS_pos_sp_deg         = 0.0f;   // MKS current position setpoint (0 = switch D5)
volatile float    MSK2_pos_sp_deg        = 0.0f;   // MSK2_stepper current position setpoint (deg)
volatile float    MKS2_pos_sp_deg        = 0.0f;   // Alias for MSK2_pos_sp_deg (always 0 <-> 3600 deg)
volatile float    M1_current_angle_deg   = 0.0f;   // M1 actual angle relative to 0 switch (deg)
volatile float    M2_current_angle_deg   = 0.0f;   // M2 actual angle relative to 0 switch (deg)
volatile float    MKS_current_angle_deg  = 0.0f;   // MKS actual angle relative to 0 switch (deg)
volatile float    MSK2_current_angle_deg = 0.0f;   // MSK2_stepper actual angle relative to power-on 0 (deg)
volatile int16_t  M1_speed_dps           = 0;      // M1 speed feedback (dps)
volatile int16_t  M2_speed_dps           = 0;      // M2 speed feedback (dps)

/* ==================== MKS #1 Control on FDCAN2 (Homing on PD5 + Direct Angle Target) ==================== */
volatile float        MKS_target_deg     = 0.0f;            // Target angle for MKS #1 in deg (clamped to [MKS_min_deg, MKS_max_deg])
volatile float        MKS_min_deg        = 0.0f;            // Minimum allowed MKS #1 target angle (0.0 = Limit Switch D5)
volatile float        MKS_max_deg        = 25000.0f;         // Maximum allowed MKS #1 target angle (deg)
volatile uint16_t     MKS_speed_rpm      = 300;             // MKS #1 rotation speed in RPM (1..3000)
volatile uint8_t      MKS_acc            = 10;              // MKS #1 acceleration (1..255; note: in 0xFD, acc=0 stops motor!)
volatile uint32_t     MKS_pulses_per_rev = 3200;            // 3200 pulses/rev at default MStep=16 (200 steps * 16)
volatile uint8_t      MKS_pos_mode       = 0;               // 0 = 0xFD (V1.0.0 Rel Pulses), 1 = 0xFE (Abs Pulses), 2 = 0xF5 (Abs Axis)
volatile uint8_t      MKS_invert_dir     = 0;               // 0 = Home CCW (dir=1) & +Target CW (dir=0); 1 = Home CW (dir=0) & +Target CCW (dir=1)
volatile uint8_t      MKS_set_zero       = 0;               // Set to 1 in Live Expressions to manually zero current MKS #1 position
volatile uint16_t     mks_can_id         = MKS_DEFAULT_ID;  // MKS #1 CAN ID on FDCAN2 (default 0x01)
volatile uint16_t     MKS_active_kbps    = 500;             // Locked at 500 kbps on FDCAN2 (PB12/PB13)
volatile uint8_t      poll_enable        = 1;               // 1 = poll MKS encoders (0x31) every 100 ms
static Motor_Homing_State_t mks_state            = MOTOR_STATE_HOMING_SEARCH;
static uint32_t             mks_settle_tick      = 0;
static uint8_t              mks_zero_sent        = 0;
static float                mks_zero_offset      = 0.0f;
static float                mks_last_target      = 0.0f;
static float                mks_pending_delta    = 0.0f;
static uint8_t              mks_move_pending     = 0;
static uint32_t             mks_move_cmd_tick    = 0;
static uint32_t             mks_move_start_tick  = 0;
static uint32_t             mks_expected_move_ms = 0;
static uint32_t             mks_home_cmd_tick    = 0;
static uint8_t              mks_last_inv_dir     = 0;
static uint16_t             mks_last_home_rpm    = 0;
static uint8_t              mks_was_free_rot     = 0;

/* ==================== MSK2_stepper Control on FDCAN2 (No Calibration Needed, Direct Angle Target) ==================== */
volatile float        MSK2_stepper        = 0.0f;             // Type target angle in deg (e.g. 360.0) to rotate MSK2_stepper
volatile float        MSK2_min_deg        = -50000.0f;        // Minimum allowed MSK2_stepper target angle (deg)
volatile float        MSK2_max_deg        = 50000.0f;         // Maximum allowed MSK2_stepper target angle (deg)
volatile uint16_t     MSK2_speed_rpm      = 3000;              // MSK2_stepper rotation speed in RPM (1..3000)
volatile int16_t      MSK2_acc            = 1000;               // MSK2_stepper acceleration (1..255)
volatile uint32_t     MSK2_pulses_per_rev = 3200;             // 3200 pulses/rev at default MStep=16 (200 steps * 16)
volatile uint8_t      MSK2_pos_mode       = 0;                // 0 = 0xFD (Rel Pulses), 1 = 0xFE (Abs Pulses), 2 = 0xF5 (Abs Axis)
volatile uint8_t      MSK2_invert_dir     = 0;                // 0 = positive deg is CW (dir=0), 1 = positive deg is CCW (dir=1)
volatile uint8_t      MSK2_set_zero       = 0;                // Set to 1 in Live Expressions to zero current MSK2_stepper position
volatile uint16_t     msk2_can_id         = MSK2_DEFAULT_ID;  // MSK2_stepper CAN ID on FDCAN2 (default 0x02)
static float          msk2_zero_offset      = 0.0f;
static float          msk2_last_target      = 0.0f;
static float          msk2_pending_delta    = 0.0f;
static uint8_t        msk2_move_pending     = 0;
static uint32_t       msk2_move_cmd_tick    = 0;
static uint32_t       msk2_move_start_tick  = 0;
static uint32_t       msk2_expected_move_ms = 0;
static uint8_t        msk2_was_free_rot     = 0;

/* FDCAN2 Bus Diagnostics */
volatile uint32_t     fdcan2_lec         = 0;
volatile uint32_t     fdcan2_bus_off     = 0;
volatile uint32_t     fdcan2_err_passive = 0;
volatile uint32_t     fdcan2_tx_err_cnt  = 0;
volatile uint32_t     fdcan2_rx_err_cnt  = 0;

/*
 * ==================== SCARA ROBOT TEACH / RECORD & PLAYBACK STATE MACHINE ====================
 * 1. First Start: Calibrate M1, M2, and MKS #1 (wait until all homed).
 * 2. Press state_start (PE11): Free motors (free_rotate_mode = 1) to move robot for recording.
 *    - 1st press of state_record (PE9)  -> Record Position 1 (n=0: MKS_pos_sp_deg = 0)
 *    - 2nd press of state_record (PE9)  -> Record Position 2 (n=1: MKS_pos_sp_deg = 5000)
 *    - (n+1)th press of state_record    -> Record Position (n+1) (MKS_pos_sp_deg = 5000 * n)
 * 3. Press state_stop_record (PE10 / PC0): Exit free-rotate recording mode.
 * 4. Hold state_start = 1 for 1 second (1000 ms):
 *    - Calibrate all motors (M1, M2, MKS #1) to 0 deg first.
 *    - Move to Position 1 (M1, M2, MKS_pos_sp_deg = 0) and wait until ALL are DONE.
 *    - Stop and rotate MSK2_stepper to 3600 deg (MKS2_pos_sp_deg = 3600), wait until DONE.
 *    - Move to Position 2 (M1, M2, MKS_pos_sp_deg = 5000) and wait until ALL are DONE.
 *    - Stop and rotate MSK2_stepper to 0 deg (MKS2_pos_sp_deg = 0), wait until DONE.
 *    - Repeat for all recorded positions (Position n+1 => MKS = 5000*n, MSK2 toggles 0 <-> 3600).
 */
volatile uint8_t               state_start              = 0; // PE11: no press = 0, pressed = 1 (hold 1s after stop_record to play)
volatile uint8_t               state_record             = 0; // PE9:  no record = 0, press = 1 (records Position 1, 2, ..., n+1)
volatile uint8_t               state_stop_record        = 0; // PE10 / PC0: press = 1 to finish recording waypoints
volatile uint8_t               state_playback           = 0; // PC0:  alias / indicator for stop-record & playback readiness
volatile Scara_Process_State_t scara_state              = SCARA_STATE_INIT_CALIBRATE;
volatile Scara_Waypoint_t      scara_waypoints[SCARA_MAX_WAYPOINTS];
volatile uint8_t               recorded_positions_count = 0; // Total taught positions recorded (1..SCARA_MAX_WAYPOINTS)
volatile uint8_t               current_position_index   = 0; // 1-based Position number currently executing (1 = Pos 1, 2 = Pos 2, ...)
volatile uint32_t              start_hold_elapsed_ms    = 0; // Live view of how long state_start has been held (0..1000 ms)

uint32_t now_tick = 0;


/* Internal Axis Structures */
static RMD_Axis_t axis_M1;
static RMD_Axis_t axis_M2;

/* USER CODE END Variables */

/* Definitions for RMD_Task */
osThreadId_t RMD_TaskHandle;
const osThreadAttr_t RMD_Task_attributes = {
  .name = "RMD_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for Status_Task */
osThreadId_t Status_TaskHandle;
const osThreadAttr_t Status_Task_attributes = {
  .name = "Status_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void RMD_Control_Task(void *argument);
void Status_Heartbeat_Task(void *argument);
static void RMD_Axis_Init(RMD_Axis_t *ax, FDCAN_HandleTypeDef *hfdcan, CAN_Bus_t bus, uint16_t can_id, GPIO_TypeDef *port, uint16_t pin);
static void RMD_Axis_Update(RMD_Axis_t *ax, volatile float *user_target_deg, float min_deg, float max_deg, float home_speed_dps, uint8_t invert_dir, uint8_t *homed_flag, uint32_t now_tick);
static void RMD_ProgramMotorID_Sequence(FDCAN_HandleTypeDef *hfdcan, uint8_t new_id);
static void MKS_Servo_Init(void);
static void MKS_Servo_Update(uint32_t now_tick);
static void MSK2_Stepper_Init(void);
static void MSK2_Stepper_Update(uint32_t now_tick);
static uint8_t RMD_Axis_Move_Is_Done(RMD_Axis_t *ax, float target_deg, float current_angle_deg, uint32_t now_tick);
static uint8_t MKS_Move_Is_Done(uint32_t now_tick);
static uint8_t MSK2_Move_Is_Done(uint32_t now_tick);
static void SCARA_Process_Update(uint32_t now_tick);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes / Functions needed when configGENERATE_RUN_TIME_STATS is 1 */
__weak void configureTimerForRunTimeStats(void) { }
__weak unsigned long getRunTimeCounterValue(void) { return HAL_GetTick(); }

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* Configure FDCAN Filters for RMD CAN IDs (0x141 - 0x148 and 0x241 - 0x248) */
  CAN_FilterConfig();

  /* Initialize M1 (FDCAN1, ID 0x141, Limit Switch D6), M2 (Limit Switch D7), and MKS (FDCAN2, Limit Switch D4) */
  RMD_Axis_Init(&axis_M1, &hfdcan1, CAN_BUS_I, RMD_MOTOR_M1_ID, M1_LIMIT_PORT, M1_LIMIT_PIN);
  if (M2_use_fdcan2) {
    RMD_Axis_Init(&axis_M2, &hfdcan2, CAN_BUS_II, RMD_MOTOR_M1_ID, M2_LIMIT_PORT, M2_LIMIT_PIN);
  } else {
    RMD_Axis_Init(&axis_M2, &hfdcan1, CAN_BUS_I, RMD_MOTOR_M2_ID, M2_LIMIT_PORT, M2_LIMIT_PIN);
  }
  MKS_Servo_Init();
  MSK2_Stepper_Init();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create Thread(s) */
  RMD_TaskHandle = osThreadNew(RMD_Control_Task, NULL, &RMD_Task_attributes);
  Status_TaskHandle = osThreadNew(Status_Heartbeat_Task, NULL, &Status_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_RMD_Control_Task */
/**
  * @brief  Main RMD Motor Homing & Position Control Task (M1 & M2)
  * @param  argument: Not used
  */
/* USER CODE END Header_RMD_Control_Task */
void RMD_Control_Task(void *argument)
{
  /* USER CODE BEGIN RMD_Control_Task */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t last_m2_fdcan2_mode = M2_use_fdcan2;

  /* Enable and release brakes for both M1 and M2 on startup */
  RMD_EnableMotor(axis_M1.hfdcan, axis_M1.can_id);
  osDelay(10);
  RMD_EnableMotor(axis_M2.hfdcan, axis_M2.can_id);
  osDelay(40);

  RMD_ReleaseBrake(axis_M1.hfdcan, axis_M1.can_id);
  osDelay(10);
  RMD_ReleaseBrake(axis_M2.hfdcan, axis_M2.can_id);
  osDelay(40);

  /* Enable MKS #1 (ID 0x01) and MSK2_stepper (ID 0x02) on FDCAN2 (PB12/PB13 @ 500 kbps) at startup */
  MKS_ReleaseProtect(&hfdcan2, mks_can_id);
  osDelay(10);
  MKS_Enable(&hfdcan2, mks_can_id, 1);
  osDelay(10);

  MKS_ReleaseProtect(&hfdcan2, msk2_can_id);
  osDelay(10);
  MKS_Enable(&hfdcan2, msk2_can_id, 1);
  osDelay(10);
  MKS_SetZero(&hfdcan2, msk2_can_id);
  osDelay(20);

  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
   now_tick = xTaskGetTickCount();

    /* Read D4 (MKS), D6 (M1), and D7 (M2) limit switch states (Normal = 1, Hit = 0) */
    D5_state = (HAL_GPIO_ReadPin(MKS_LIMIT_PORT, MKS_LIMIT_PIN) == GPIO_PIN_SET) ? 1 : 0;
    D6_state = (HAL_GPIO_ReadPin(M1_LIMIT_PORT, M1_LIMIT_PIN) == GPIO_PIN_SET) ? 1 : 0;
    D7_state = (HAL_GPIO_ReadPin(M2_LIMIT_PORT, M2_LIMIT_PIN) == GPIO_PIN_SET) ? 1 : 0;

    /* One-shot Motor ID Programming (Connect ONLY M2, set set_new_motor_id = 2) */
    if (set_new_motor_id >= 1 && set_new_motor_id <= 32) {
      uint8_t target_id = set_new_motor_id;
      set_new_motor_id = 0;
      RMD_ProgramMotorID_Sequence(&hfdcan1, target_id);
      id_write_done = 1;
    }

    /* While id_write_done == 1, keep CAN bus completely silent until user power-cycles motor */
    if (id_write_done) {
      osDelay(100);
      xLastWakeTime = xTaskGetTickCount();
      continue;
    }

    /* Check if user toggled M2_use_fdcan2 or rehome_trigger in Live Expressions */
    if (rehome_trigger || (M2_use_fdcan2 != last_m2_fdcan2_mode)) {
      rehome_trigger = 0;
      last_m2_fdcan2_mode = M2_use_fdcan2;
      free_rotate_mode = 0;
      M1_homed = 0;
      M2_homed = 0;
      M1_target = 0.0f;
      M2_target = 0.0f;
      RMD_Axis_Init(&axis_M1, &hfdcan1, CAN_BUS_I, RMD_MOTOR_M1_ID, M1_LIMIT_PORT, M1_LIMIT_PIN);
      if (M2_use_fdcan2) {
        RMD_Axis_Init(&axis_M2, &hfdcan2, CAN_BUS_II, RMD_MOTOR_M1_ID, M2_LIMIT_PORT, M2_LIMIT_PIN);
      } else {
        RMD_Axis_Init(&axis_M2, &hfdcan1, CAN_BUS_I, RMD_MOTOR_M2_ID, M2_LIMIT_PORT, M2_LIMIT_PIN);
      }
      MKS_Servo_Init();
      RMD_EnableMotor(axis_M1.hfdcan, axis_M1.can_id);
      osDelay(10);
      RMD_EnableMotor(axis_M2.hfdcan, axis_M2.can_id);
      osDelay(20);
      RMD_ReleaseBrake(axis_M1.hfdcan, axis_M1.can_id);
      osDelay(10);
      RMD_ReleaseBrake(axis_M2.hfdcan, axis_M2.can_id);
      osDelay(20);
      MKS_ReleaseProtect(&hfdcan2, mks_can_id);
      osDelay(10);
      MKS_Enable(&hfdcan2, mks_can_id, 1);
      osDelay(10);
      MKS_ReleaseProtect(&hfdcan2, msk2_can_id);
      osDelay(10);
      MKS_Enable(&hfdcan2, msk2_can_id, 1);
      osDelay(20);
      xLastWakeTime = xTaskGetTickCount();
      now_tick = xLastWakeTime;
    }

    /* Update SCARA Teach/Record & Playback State Machine */
    SCARA_Process_Update(now_tick);

    /* Update M1 and M2 state machines with 2ms spacing so CAN frames never collide */
    RMD_Axis_Update(&axis_M1, &M1_target, M1_min_deg, M1_max_deg, M1_homing_speed_dps, M1_invert_dir, (uint8_t *)&M1_homed, now_tick);
    osDelay(2);
    RMD_Axis_Update(&axis_M2, &M2_target, M2_min_deg, M2_max_deg, M2_homing_speed_dps, M2_invert_dir, (uint8_t *)&M2_homed, now_tick);

    /* Update MKS #1 (with Limit-Switch Homing) and MSK2_stepper (Direct Control, No Calibration) on FDCAN2 */
    MKS_Servo_Update(now_tick);
    osDelay(2);
    MSK2_Stepper_Update(now_tick);

    /* Update Live Expression diagnostic variables */
    M1_pos_sp_deg   = axis_M1.pos_sp_deg;
    M2_pos_sp_deg   = axis_M2.pos_sp_deg;
    MKS2_pos_sp_deg = MSK2_pos_sp_deg;

    RMD_Feedback_t *fb1 = RMD_GetFeedback(axis_M1.bus, axis_M1.fb_index);
    if (fb1 != NULL) {
      M1_speed_dps = fb1->feedback_speed;
      if (M1_homed) {
        float dir_sign1 = M1_invert_dir ? -1.0f : 1.0f;
        M1_current_angle_deg = (fb1->multi_turn_angle_deg - axis_M1.zero_offset_deg) * dir_sign1;
        if (fabsf(M1_current_angle_deg) > fabsf(M1_max_measured_deg)) {
          M1_max_measured_deg = M1_current_angle_deg;
        }
      }
    }

    RMD_Feedback_t *fb2 = RMD_GetFeedback(axis_M2.bus, axis_M2.fb_index);
    if (fb2 != NULL) {
      M2_speed_dps = fb2->feedback_speed;
      if (M2_homed) {
        float dir_sign2 = M2_invert_dir ? -1.0f : 1.0f;
        M2_current_angle_deg = (fb2->multi_turn_angle_deg - axis_M2.zero_offset_deg) * dir_sign2;
        if (fabsf(M2_current_angle_deg) > fabsf(M2_max_measured_deg)) {
          M2_max_measured_deg = M2_current_angle_deg;
        }
      }
    }

    /* Wait for next cycle (10ms / 100 Hz) */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
  /* USER CODE END RMD_Control_Task */
}

/* USER CODE BEGIN Header_Status_Heartbeat_Task */
/**
  * @brief  Heartbeat Task to indicate System Alive & Poll Physical Buttons
  * @param  argument: Not used
  */
/* USER CODE END Header_Status_Heartbeat_Task */
void Status_Heartbeat_Task(void *argument)
{
  /* USER CODE BEGIN Status_Heartbeat_Task */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t heatbeat = 0;
  uint8_t last_pin_start  = 0;
  uint8_t last_pin_record = 0;
  uint8_t last_pin_stop   = 0;

  for (;;)
  {
    /* Toggle LED (PA15) every 200ms to indicate system alive */
    if (xTaskGetTickCount() - heatbeat >= 200) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
      heatbeat = xTaskGetTickCount();
    }

    /*
     * Read physical button pins (configured with Pull-Down: no press = 0, pressed = 1).
     * Only overwrite state_* when the physical pin state changes or is held high,
     * so the user can ALSO trigger state_start / state_record / state_stop_record in Live Expressions!
     */
    uint8_t pin_start  = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t pin_record = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)  == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t pin_stop   = ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) == GPIO_PIN_SET) ||
                          (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)  == GPIO_PIN_SET) ||
                          (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)  == GPIO_PIN_SET)) ? 1u : 0u;

    if (pin_start || (pin_start != last_pin_start)) {
      state_start = pin_start;
    }
    if (pin_record || (pin_record != last_pin_record)) {
      state_record = pin_record;
    }
    if (pin_stop || (pin_stop != last_pin_stop)) {
      state_stop_record = pin_stop;
      state_playback    = pin_stop;
    }

    last_pin_start  = pin_start;
    last_pin_record = pin_record;
    last_pin_stop   = pin_stop;

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
  }
  /* USER CODE END Status_Heartbeat_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief Comprehensive sequence to write a new Motor ID (1..32) to RMD ROM
  */
static void RMD_ProgramMotorID_Sequence(FDCAN_HandleTypeDef *hfdcan, uint8_t new_id)
{
  /* 1. Shutdown (0x80) and Stop (0x81) motor so it exits closed-loop control */
  RMD_DisableMotor(hfdcan, RMD_MOTOR_ID_BASE);
  osDelay(50);
  RMD_StopMotor(hfdcan, RMD_MOTOR_ID_BASE);
  osDelay(100);

  /* 2. Standard RMD V3 0x79 command (new_id in Byte 7) to 0x300, 0x141, and 0x7FF */
  uint8_t cmd_b7[8] = {0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, new_id};
  FDCAN_TxHeaderTypeDef txHeader = {
    .Identifier = 0x300,
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_8,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .MessageMarker = 0
  };

  txHeader.Identifier = 0x300;
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, cmd_b7);
  osDelay(100);

  txHeader.Identifier = 0x141;
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, cmd_b7);
  osDelay(100);

  txHeader.Identifier = 0x7FF;
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, cmd_b7);
  osDelay(100);

  /* 3. Alternate RMD 0x79 format (uint16_t Little-Endian in Byte 6) */
  uint8_t cmd_b6[8] = {0x79, 0x00, 0x00, 0x00, 0x00, 0x00, new_id, 0x00};
  txHeader.Identifier = 0x300;
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, cmd_b6);
  osDelay(100);

  txHeader.Identifier = 0x141;
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, cmd_b6);
  osDelay(600);
}

/**
  * @brief Initialize an RMD motor axis structure for homing and motion profiling
  */
static void RMD_Axis_Init(RMD_Axis_t *ax, FDCAN_HandleTypeDef *hfdcan, CAN_Bus_t bus, uint16_t can_id, GPIO_TypeDef *port, uint16_t pin)
{
  ax->hfdcan            = hfdcan;
  ax->bus               = bus;
  ax->can_id            = can_id;
  ax->fb_index          = (uint8_t)(can_id - RMD_MOTOR_ID_BASE);
  ax->limit_port        = port;
  ax->limit_pin         = pin;
  ax->state             = MOTOR_STATE_HOMING_SEARCH;
  ax->settle_start_tick = 0;
  ax->zero_offset_deg   = 0.0f;
  ax->last_target_deg   = 0.0f;
  ax->profile_planned   = 0;
  ax->has_moved         = 0;
  ax->move_start_tick   = 0;
  ax->profile_state     = MOTION_PROFILE_UNINITIALIZED;
  ax->pos_sp_deg        = 0.0f;
  ax->vel_sp_dps        = 0.0f;
  ax->acc_sp_dps2       = 0.0f;

  RMD_Feedback_t *fb = RMD_GetFeedback(ax->bus, ax->fb_index);
  if (fb != NULL) {
    fb->angle_valid = 0;
  }

  motion_profile_init(&ax->motion,
                      profile_max_acc_dps2,
                      profile_max_speed_dps,
                      profile_max_dec_dps2);
}

/**
  * @brief Execute per-axis homing and trapezoidal position control with input clamping
  */
static void RMD_Axis_Update(RMD_Axis_t *ax, volatile float *user_target_deg, float min_deg, float max_deg, float home_speed_dps, uint8_t invert_dir, uint8_t *homed_flag, uint32_t now_tick)
{
  /* Read limit switch: Normal = 1 (GPIO_PIN_SET), Hit = 0 (GPIO_PIN_RESET) */
  uint8_t limit_hit = (HAL_GPIO_ReadPin(ax->limit_port, ax->limit_pin) == GPIO_PIN_RESET) ? 1 : 0;
  float dir_sign = invert_dir ? -1.0f : 1.0f;

  switch (ax->state)
  {
    case MOTOR_STATE_HOMING_SEARCH:
    {
      if (limit_hit) {
        /* Limit switch hit (pin == 0) -> Stop motor immediately and enter settle state */
        RMD_SetSpeed(ax->hfdcan, ax->can_id, 0);
        ax->settle_start_tick = now_tick;
        ax->state             = MOTOR_STATE_HOMING_SETTLE;
      } else {
        /* Not hit yet (pin == 1) -> Rotate slowly toward limit switch (reverse of positive target direction) */
        int32_t rev_speed_0_01dps = (int32_t)(-dir_sign * fabsf(home_speed_dps) * 100.0f);
        RMD_SetSpeed(ax->hfdcan, ax->can_id, rev_speed_0_01dps);
      }
      break;
    }

    case MOTOR_STATE_HOMING_SETTLE:
    {
      uint32_t elapsed = now_tick - ax->settle_start_tick;
      RMD_Feedback_t *fb = RMD_GetFeedback(ax->bus, ax->fb_index);

      if (elapsed < HOMING_STOP_MS) {
        /* First 50ms: send speed = 0 to ensure motor is completely stopped on switch */
        RMD_SetSpeed(ax->hfdcan, ax->can_id, 0);
      } else {
        /* Next: request multi-turn angle (0x92) to latch current position as 0 deg */
        RMD_RequestMultiTurnAngle(ax->hfdcan, ax->can_id);
      }

      if (elapsed >= HOMING_SETTLE_MS && fb != NULL && fb->angle_valid) {
        ax->zero_offset_deg = fb->multi_turn_angle_deg;
        ax->pos_sp_deg      = 0.0f;
        ax->vel_sp_dps      = 0.0f;
        ax->acc_sp_dps2     = 0.0f;
        ax->last_target_deg = 0.0f;
        *user_target_deg    = 0.0f;
        ax->profile_planned = 1;
        ax->has_moved       = 0;
        *homed_flag         = 1;
        ax->state           = MOTOR_STATE_POSITION_READY;
      }
      break;
    }

    case MOTOR_STATE_POSITION_READY:
    default:
    {
      /* Free-Rotate / Teach Mode: zero torque (free shaft) and read multi-turn angle (0x92) */
      if (free_rotate_mode) {
        if ((now_tick / CONTROL_PERIOD_MS) & 0x01) {
          RMD_SetTorque(ax->hfdcan, ax->can_id, 0);
        } else {
          RMD_RequestMultiTurnAngle(ax->hfdcan, ax->can_id);
        }
        RMD_Feedback_t *fb = RMD_GetFeedback(ax->bus, ax->fb_index);
        if (fb != NULL && fb->angle_valid) {
          float cur_rel_deg   = (fb->multi_turn_angle_deg - ax->zero_offset_deg) * dir_sign;
          ax->pos_sp_deg      = cur_rel_deg;
          ax->last_target_deg = cur_rel_deg;
          *user_target_deg    = cur_rel_deg;
        }
        ax->has_moved = 0;
        break;
      }

      /* Clamp user input target to [min_deg, max_deg] and write back so Live Expressions shows clamped value */
      float target_deg = *user_target_deg;
      if (target_deg < min_deg) {
        target_deg = min_deg;
        *user_target_deg = min_deg;
      } else if (target_deg > max_deg) {
        target_deg = max_deg;
        *user_target_deg = max_deg;
      }

      /* Re-plan trapezoidal move whenever user changes target (e.g. M1_target = 360) */
      if (!ax->profile_planned || fabsf(target_deg - ax->last_target_deg) > 0.001f) {
        motion_profile_init(&ax->motion,
                            profile_max_acc_dps2,
                            profile_max_speed_dps,
                            profile_max_dec_dps2);
        motion_profile_plan_move(&ax->motion,
                                 ax->pos_sp_deg,
                                 0.0f,
                                 target_deg,
                                 0.0f);
        ax->move_start_tick = now_tick;
        ax->last_target_deg = target_deg;
        ax->profile_planned = 1;
        ax->has_moved       = 1;
      }

      /* Before the first move is commanded (target == 0 at switch), keep motor stopped at 0 */
      if (!ax->has_moved) {
        RMD_SetSpeed(ax->hfdcan, ax->can_id, 0);
        break;
      }

      /* Compute instantaneous setpoint from trapezoidal profile (Motion.c) */
      float elapsed_s = (float)(now_tick - ax->move_start_tick) * 0.001f;
      float pos_sp = ax->pos_sp_deg;
      float vel_sp = 0.0f;
      float acc_sp = 0.0f;

      ax->profile_state = motion_profile_get_pva(&ax->motion, elapsed_s, &pos_sp, &vel_sp, &acc_sp);
      if (ax->profile_state != MOTION_PROFILE_INVALID) {
        ax->pos_sp_deg  = pos_sp;
        ax->vel_sp_dps  = vel_sp;
        ax->acc_sp_dps2 = acc_sp;
      }

      /* Hardware speed cap for RMD 0xA4 command */
      uint16_t rmd_speed_limit = (uint16_t)fabsf(profile_max_speed_dps);
      if (pos_max_speed_dps > rmd_speed_limit) {
        rmd_speed_limit = pos_max_speed_dps;
      }
      if (rmd_speed_limit == 0) {
        rmd_speed_limit = 100;
      }

      /* Convert relative angle (0 = limit switch) to RMD absolute multi-turn command (0.01 deg) */
      float absolute_cmd_deg = ax->zero_offset_deg + (dir_sign * ax->pos_sp_deg);
      RMD_SetPosition(ax->hfdcan,
                      ax->can_id,
                      rmd_speed_limit,
                      (int32_t)(absolute_cmd_deg * 100.0f));
      break;
    }
  }
}

/**
  * @brief Initialize MKS SERVO57D homing state machine for Limit Switch PD5
  */
static void MKS_Servo_Init(void)
{
  mks_state            = MOTOR_STATE_HOMING_SEARCH;
  mks_settle_tick      = 0;
  mks_zero_sent        = 0;
  mks_zero_offset      = 0.0f;
  mks_last_target      = 0.0f;
  mks_pending_delta    = 0.0f;
  mks_move_pending     = 0;
  mks_move_cmd_tick    = 0;
  mks_move_start_tick  = 0;
  mks_expected_move_ms = 0;
  mks_home_cmd_tick    = 0;
  mks_last_inv_dir     = MKS_invert_dir;
  mks_last_home_rpm    = MKS_homing_speed_rpm;
  mks_was_free_rot     = 0;
  mks_move_status      = MKS_MOVE_IDLE;
  MKS_homed            = 0;
  MKS_target_deg       = 0.0f;
  MKS_pos_sp_deg       = 0.0f;
  MKS_current_angle_deg = 0.0f;
}

/**
  * @brief Process MKS SERVO57D V1.3 PD5 Limit-Switch Homing + Direct Target Angle Control on FDCAN2 (PB12/PB13 @ 500 kbps)
  */
static void MKS_Servo_Update(uint32_t now_tick)
{
  static uint32_t last_poll_tick = 0;

  MKS_active_kbps = 500; /* Permanently locked at 500 kbps on FDCAN2 (PB12/PB13) */

  /* 1. FDCAN2 Bus-Off & Error Diagnostics (proper HAL Stop/Start recovery) */
  FDCAN_ProtocolStatusTypeDef ps;
  FDCAN_ErrorCountersTypeDef  ec;
  if (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &ps) == HAL_OK) {
    fdcan2_lec         = ps.LastErrorCode;
    fdcan2_bus_off     = ps.BusOff;
    fdcan2_err_passive = ps.ErrorPassive;
    if (ps.BusOff) {
      HAL_FDCAN_Stop(&hfdcan2);
      HAL_FDCAN_Start(&hfdcan2);
    }
  }
  if (HAL_FDCAN_GetErrorCounters(&hfdcan2, &ec) == HAL_OK) {
    fdcan2_tx_err_cnt = ec.TxErrorCnt;
    fdcan2_rx_err_cnt = ec.RxErrorCnt;
  }

  /* 2. Check MKS-only re-home trigger (MKS_rehome_trigger = 1 in Live Expressions) */
  if (MKS_rehome_trigger) {
    MKS_rehome_trigger = 0;
    MKS_Servo_Init();
    MKS_Enable(&hfdcan2, mks_can_id, 1);
    return;
  }

  /* Read PD5 limit switch: Normal = 1 (GPIO_PIN_SET), Hit = 0 (GPIO_PIN_RESET) */
  uint8_t limit_hit = (HAL_GPIO_ReadPin(MKS_LIMIT_PORT, MKS_LIMIT_PIN) == GPIO_PIN_RESET) ? 1 : 0;
  float   dir_sign  = MKS_invert_dir ? -1.0f : 1.0f;

  switch (mks_state)
  {
    case MOTOR_STATE_HOMING_SEARCH:
    {
      if (limit_hit) {
        /* PD5 Limit Switch hit (pin == 0) -> Stop MKS immediately (acc = 0) and enter settle state */
        MKS_StopSpeed(&hfdcan2, mks_can_id, 0);
        mks_settle_tick = now_tick;
        mks_zero_sent   = 0;
        mks_state       = MOTOR_STATE_HOMING_SETTLE;
      } else {
        /*
         * Not hit yet (PD5 == 1) -> Rotate slowly toward PD5 switch in Speed Mode (0xF6).
         * When MKS_invert_dir == 0: positive angle is dir=0 (CW), so homing toward 0 switch is dir=1 (CCW).
         * When MKS_invert_dir == 1: positive angle is dir=1 (CCW), so homing toward 0 switch is dir=0 (CW).
         */
        uint8_t  home_dir = MKS_invert_dir ? 0u : 1u;
        uint16_t home_rpm = (MKS_homing_speed_rpm == 0) ? 40u : MKS_homing_speed_rpm;
        uint8_t  safe_acc = (MKS_acc == 0) ? 10u : MKS_acc;

        if ((mks_home_cmd_tick == 0) ||
            (now_tick - mks_home_cmd_tick >= 50u) ||
            (MKS_invert_dir != mks_last_inv_dir) ||
            (home_rpm != mks_last_home_rpm))
        {
          mks_home_cmd_tick = now_tick;
          mks_last_inv_dir  = MKS_invert_dir;
          mks_last_home_rpm = home_rpm;
          MKS_SetSpeed(&hfdcan2, mks_can_id, home_dir, home_rpm, safe_acc);
        }
      }
      break;
    }

    case MOTOR_STATE_HOMING_SETTLE:
    {
      uint32_t elapsed = now_tick - mks_settle_tick;

      if (elapsed < HOMING_STOP_MS) {
        /* First 50ms: send immediate speed stop (0xF6 speed=0, acc=0) to ensure motor is stopped on PD5 */
        if ((elapsed % 20u) == 0u) {
          MKS_StopSpeed(&hfdcan2, mks_can_id, 0);
        }
      } else if (!mks_zero_sent) {
        /* At 50ms: send Set-Zero command (0x92) to zero MKS internal pulse & encoder counters at PD5 */
        mks_zero_sent = 1;
        mks_enc_value = 0;
        mks_enc_deg   = 0.0f;
        MKS_SetZero(&hfdcan2, mks_can_id);
      } else if ((elapsed % 30u) == 0u) {
        /* Poll encoder (0x31) to confirm 0 deg reference */
        MKS_ReadEncoder(&hfdcan2, mks_can_id);
      }

      if (elapsed >= HOMING_SETTLE_MS) {
        mks_zero_offset       = mks_enc_deg;
        MKS_target_deg        = 0.0f;
        mks_last_target       = 0.0f;
        mks_pending_delta     = 0.0f;
        mks_move_pending      = 0;
        mks_expected_move_ms  = 0;
        MKS_pos_sp_deg        = 0.0f;
        MKS_current_angle_deg = 0.0f;
        MKS_homed             = 1;
        last_poll_tick        = now_tick;
        mks_state             = MOTOR_STATE_POSITION_READY;
      }
      break;
    }

    case MOTOR_STATE_POSITION_READY:
    default:
    {
      /* Update MKS_current_angle_deg & MKS_max_measured_deg from encoder feedback */
      MKS_current_angle_deg = (mks_enc_deg - mks_zero_offset) * dir_sign;
      if (fabsf(MKS_current_angle_deg) > fabsf(MKS_max_measured_deg)) {
        MKS_max_measured_deg = MKS_current_angle_deg;
      }

      /* Free-Rotate / Teach Mode: disable MKS holding torque so user can rotate shaft by hand */
      if (free_rotate_mode) {
        if (!mks_was_free_rot) {
          mks_was_free_rot = 1;
          MKS_Enable(&hfdcan2, mks_can_id, 0);
        }
        if (now_tick - last_poll_tick >= 50u) {
          last_poll_tick = now_tick;
          MKS_ReadEncoder(&hfdcan2, mks_can_id);
        }
        MKS_pos_sp_deg       = MKS_current_angle_deg;
        mks_last_target      = MKS_current_angle_deg;
        MKS_target_deg       = MKS_current_angle_deg;
        mks_pending_delta    = 0.0f;
        mks_move_pending     = 0;
        mks_expected_move_ms = 0;
        return;
      } else if (mks_was_free_rot) {
        mks_was_free_rot = 0;
        MKS_Enable(&hfdcan2, mks_can_id, 1);
      }

      /* Manual Set-Zero trigger from Live Expressions (MKS_set_zero = 1) */
      if (MKS_set_zero) {
        MKS_set_zero          = 0;
        mks_zero_offset       = 0.0f;
        mks_enc_value         = 0;
        mks_enc_deg           = 0.0f;
        MKS_target_deg        = 0.0f;
        mks_last_target       = 0.0f;
        mks_pending_delta     = 0.0f;
        mks_move_pending      = 0;
        mks_expected_move_ms  = 0;
        MKS_pos_sp_deg        = 0.0f;
        MKS_current_angle_deg = 0.0f;
        MKS_SetZero(&hfdcan2, mks_can_id);
        return;
      }

      /* Clamp MKS_target_deg to [MKS_min_deg, MKS_max_deg] (0.0 = Limit Switch D5) */
      float clamped_mks_target = MKS_target_deg;
      if (clamped_mks_target < MKS_min_deg) {
        clamped_mks_target = MKS_min_deg;
        MKS_target_deg     = MKS_min_deg;
      } else if (clamped_mks_target > MKS_max_deg) {
        clamped_mks_target = MKS_max_deg;
        MKS_target_deg     = MKS_max_deg;
      }
      MKS_pos_sp_deg = clamped_mks_target;

      /* Detect target change -> Step 1: send Enable (0xF3) on FDCAN2 */
      if (fabsf(clamped_mks_target - mks_last_target) > 0.001f) {
        mks_pending_delta += (clamped_mks_target - mks_last_target);
        mks_last_target    = clamped_mks_target;
        mks_move_status    = MKS_MOVE_IDLE;
        mks_move_pending   = 1;
        mks_move_cmd_tick  = now_tick;
        mks_move_start_tick = now_tick;

        uint16_t rpm = (MKS_speed_rpm == 0) ? 1u : MKS_speed_rpm;
        uint32_t cruise_ms = (uint32_t)((fabsf(mks_pending_delta) * 1000.0f) / ((float)rpm * 6.0f));
        mks_expected_move_ms = (cruise_ms > 0u) ? (cruise_ms + 300u) : 0u;

        MKS_Enable(&hfdcan2, mks_can_id, 1);
      }

      /* Step 2 (20ms later): send Move command on FDCAN2 */
      if (mks_move_pending && (now_tick - mks_move_cmd_tick >= 20u)) {
        mks_move_pending = 0;
        mks_move_start_tick = now_tick;
        uint8_t safe_acc = (MKS_acc == 0) ? 10 : MKS_acc; /* In MKS V1.0.0 0xFD, acc=0 means STOP, so ensure acc >= 1 */

        if (MKS_pos_mode == 0) {
          /*
           * MKS SERVO57D_CAN User Manual — Position Mode 1 (0xFD: Relative Motion by Pulses)
           * Frame: [0xFD, (dir<<7)|(speed>>8), speed&0xFF, acc, pulses[23:16], pulses[15:8], pulses[7:0], CRC]
           */
          float delta_deg = mks_pending_delta;
          mks_pending_delta = 0.0f;
          uint8_t dir = (delta_deg >= 0.0f) ? 0u : 1u;
          if (MKS_invert_dir) {
            dir ^= 1u;
          }
          uint32_t pulses = (uint32_t)((fabsf(delta_deg) * (float)MKS_pulses_per_rev / 360.0f) + 0.5f);
          if (pulses > 0u) {
            MKS_MoveRelPulses(&hfdcan2, mks_can_id, dir, MKS_speed_rpm, safe_acc, pulses);
          } else {
            mks_expected_move_ms = 0;
          }
        } else if (MKS_pos_mode == 1) {
          /* 0xFE: Absolute move by pulses (respects MKS_invert_dir) */
          mks_pending_delta = 0.0f;
          float signed_target = dir_sign * clamped_mks_target;
          int32_t target_pulses = (int32_t)(signed_target * ((float)MKS_pulses_per_rev / 360.0f));
          MKS_MoveAbsPulses(&hfdcan2, mks_can_id, MKS_speed_rpm, safe_acc, target_pulses);
        } else {
          /* 0xF5: Absolute move by encoder axis degrees (16384 counts = 360 deg, respects MKS_invert_dir) */
          mks_pending_delta = 0.0f;
          float signed_target = dir_sign * clamped_mks_target;
          MKS_MoveAbsDeg(&hfdcan2, mks_can_id, MKS_speed_rpm, safe_acc, signed_target);
        }
        last_poll_tick = now_tick; /* Give driver 100ms before next 0x31 poll */
        return;
      }

      /* Periodic encoder polling (0x31 -> updates mks_enc_deg) every 100 ms on FDCAN2 */
      if (poll_enable && (now_tick - last_poll_tick >= 100u)) {
        last_poll_tick = now_tick;
        MKS_ReadEncoder(&hfdcan2, mks_can_id);
      }
      break;
    }
  }
}

/**
  * @brief Initialize MSK2_stepper (No Calibration Needed — ready immediately at 0 deg)
  */
static void MSK2_Stepper_Init(void)
{
  msk2_zero_offset       = 0.0f;
  msk2_last_target       = 0.0f;
  msk2_pending_delta     = 0.0f;
  msk2_move_pending      = 0;
  msk2_move_cmd_tick     = 0;
  msk2_move_start_tick   = 0;
  msk2_expected_move_ms  = 0;
  msk2_was_free_rot      = 0;
  msk2_move_status       = MKS_MOVE_IDLE;
  MSK2_stepper           = 0.0f;
  MSK2_pos_sp_deg        = 0.0f;
  MKS2_pos_sp_deg        = 0.0f;
  MSK2_current_angle_deg = 0.0f;
}

/**
  * @brief Process MSK2_stepper direct target angle control (MSK2_stepper) on FDCAN2 (No Calibration)
  */
static void MSK2_Stepper_Update(uint32_t now_tick)
{
  static uint32_t last_msk2_poll_tick = 50; /* Offset by 50ms from MKS #1 polling */

  float dir_sign = MSK2_invert_dir ? -1.0f : 1.0f;

  /* 1. Update MSK2_current_angle_deg & MSK2_max_measured_deg from encoder feedback */
  MSK2_current_angle_deg = (msk2_enc_deg - msk2_zero_offset) * dir_sign;
  if (fabsf(MSK2_current_angle_deg) > fabsf(MSK2_max_measured_deg)) {
    MSK2_max_measured_deg = MSK2_current_angle_deg;
  }

  /* 2. Free-Rotate / Teach Mode: disable MSK2 holding torque so user can rotate shaft by hand */
  if (free_rotate_mode) {
    if (!msk2_was_free_rot) {
      msk2_was_free_rot = 1;
      MKS_Enable(&hfdcan2, msk2_can_id, 0);
    }
    if (now_tick - last_msk2_poll_tick >= 50u) {
      last_msk2_poll_tick = now_tick;
      MKS_ReadEncoder(&hfdcan2, msk2_can_id);
    }
    MSK2_pos_sp_deg       = MSK2_current_angle_deg;
    MKS2_pos_sp_deg       = MSK2_current_angle_deg;
    msk2_last_target      = MSK2_current_angle_deg;
    MSK2_stepper          = MSK2_current_angle_deg;
    msk2_pending_delta    = 0.0f;
    msk2_move_pending     = 0;
    msk2_expected_move_ms = 0;
    return;
  } else if (msk2_was_free_rot) {
    msk2_was_free_rot = 0;
    MKS_Enable(&hfdcan2, msk2_can_id, 1);
  }

  /* 3. Manual Set-Zero trigger from Live Expressions (MSK2_set_zero = 1) */
  if (MSK2_set_zero) {
    MSK2_set_zero          = 0;
    msk2_zero_offset       = 0.0f;
    msk2_enc_value         = 0;
    msk2_enc_deg           = 0.0f;
    MSK2_stepper           = 0.0f;
    msk2_last_target       = 0.0f;
    msk2_pending_delta     = 0.0f;
    msk2_move_pending      = 0;
    msk2_expected_move_ms  = 0;
    MSK2_pos_sp_deg        = 0.0f;
    MKS2_pos_sp_deg        = 0.0f;
    MSK2_current_angle_deg = 0.0f;
    MKS_SetZero(&hfdcan2, msk2_can_id);
    return;
  }

  /* 4. Clamp MSK2_stepper target to [MSK2_min_deg, MSK2_max_deg] */
  float clamped_msk2_target = MSK2_stepper;
  if (clamped_msk2_target < MSK2_min_deg) {
    clamped_msk2_target = MSK2_min_deg;
    MSK2_stepper        = MSK2_min_deg;
  } else if (clamped_msk2_target > MSK2_max_deg) {
    clamped_msk2_target = MSK2_max_deg;
    MSK2_stepper        = MSK2_max_deg;
  }
  MSK2_pos_sp_deg = clamped_msk2_target;
  MKS2_pos_sp_deg = clamped_msk2_target;

  /* 5. Detect target change -> Step 1: send Enable (0xF3) on FDCAN2 */
  if (fabsf(clamped_msk2_target - msk2_last_target) > 0.001f) {
    msk2_pending_delta += (clamped_msk2_target - msk2_last_target);
    msk2_last_target    = clamped_msk2_target;
    msk2_move_status    = MKS_MOVE_IDLE;
    msk2_move_pending   = 1;
    msk2_move_cmd_tick  = now_tick;
    msk2_move_start_tick = now_tick;

    uint16_t rpm = (MSK2_speed_rpm == 0) ? 1u : MSK2_speed_rpm;
    uint32_t cruise_ms = (uint32_t)((fabsf(msk2_pending_delta) * 1000.0f) / ((float)rpm * 6.0f));
    msk2_expected_move_ms = (cruise_ms > 0u) ? (cruise_ms + 300u) : 0u;

    MKS_Enable(&hfdcan2, msk2_can_id, 1);
  }

  /* Step 2 (20ms later): send Move command on FDCAN2 */
  if (msk2_move_pending && (now_tick - msk2_move_cmd_tick >= 20u)) {
    msk2_move_pending = 0;
    msk2_move_start_tick = now_tick;
    uint8_t safe_acc = (MSK2_acc <= 0) ? 10u : ((MSK2_acc > 255) ? 255u : (uint8_t)MSK2_acc);

    if (MSK2_pos_mode == 0) {
      /* 0xFD: Relative move by pulses (respects MSK2_invert_dir) */
      float delta_deg = msk2_pending_delta;
      msk2_pending_delta = 0.0f;
      uint8_t dir = (delta_deg >= 0.0f) ? 0u : 1u;
      if (MSK2_invert_dir) {
        dir ^= 1u;
      }
      uint32_t pulses = (uint32_t)((fabsf(delta_deg) * (float)MSK2_pulses_per_rev / 360.0f) + 0.5f);
      if (pulses > 0u) {
        MKS_MoveRelPulses(&hfdcan2, msk2_can_id, dir, MSK2_speed_rpm, safe_acc, pulses);
      } else {
        msk2_expected_move_ms = 0;
      }
    } else if (MSK2_pos_mode == 1) {
      /* 0xFE: Absolute move by pulses (respects MSK2_invert_dir) */
      msk2_pending_delta = 0.0f;
      float signed_target = dir_sign * clamped_msk2_target;
      int32_t target_pulses = (int32_t)(signed_target * ((float)MSK2_pulses_per_rev / 360.0f));
      MKS_MoveAbsPulses(&hfdcan2, msk2_can_id, MSK2_speed_rpm, safe_acc, target_pulses);
    } else {
      /* 0xF5: Absolute move by encoder axis degrees (respects MSK2_invert_dir) */
      msk2_pending_delta = 0.0f;
      float signed_target = dir_sign * clamped_msk2_target;
      MKS_MoveAbsDeg(&hfdcan2, msk2_can_id, MSK2_speed_rpm, safe_acc, signed_target);
    }
    last_msk2_poll_tick = now_tick;
    return;
  }

  /* 6. Periodic encoder polling (0x31 -> updates msk2_enc_deg) every 100 ms on FDCAN2 */
  if (poll_enable && (now_tick - last_msk2_poll_tick >= 100u)) {
    last_msk2_poll_tick = now_tick;
    MKS_ReadEncoder(&hfdcan2, msk2_can_id);
  }
}

/**
  * @brief Check if an RMD motor axis (M1 or M2) has completed its trapezoidal motion to target_deg
  */
static uint8_t RMD_Axis_Move_Is_Done(RMD_Axis_t *ax, float target_deg, float current_angle_deg, uint32_t now_tick)
{
  if (ax->state != MOTOR_STATE_POSITION_READY) {
    return 0;
  }

  /* If target is 0.0 deg at home switch and axis has not needed to move yet */
  if (!ax->has_moved) {
    return (fabsf(target_deg - ax->pos_sp_deg) <= 0.05f) ? 1u : 0u;
  }

  /* Ensure RMD_Axis_Update has planned the move for the current target_deg */
  if (!ax->profile_planned || (fabsf(ax->last_target_deg - target_deg) > 0.05f)) {
    return 0;
  }

  /* Trapezoidal trajectory must be in COMPLETED state */
  if (ax->profile_state != MOTION_PROFILE_COMPLETED) {
    return 0;
  }

  /* Wait until physical motor settles within 5 deg of target OR 300ms after profile completion */
  uint32_t profile_ms = (uint32_t)(ax->motion.total_time * 1000.0f);
  uint32_t elapsed_ms = now_tick - ax->move_start_tick;
  if ((fabsf(current_angle_deg - ax->pos_sp_deg) <= 5.0f) || (elapsed_ms >= (profile_ms + 300u))) {
    return 1u;
  }

  return 0;
}

/**
  * @brief Check if MKS #1 (MKS_pos_sp_deg = 5000 * n) has finished its rotation
  */
static uint8_t MKS_Move_Is_Done(uint32_t now_tick)
{
  if (!MKS_homed || (mks_state != MOTOR_STATE_POSITION_READY)) {
    return 0;
  }

  /* If target change hasn't been latched or move command is still waiting for 20ms delay */
  if (mks_move_pending || (fabsf(MKS_target_deg - mks_last_target) > 0.05f)) {
    return 0;
  }

  /* If 0 pulses were needed (e.g., Position 1 where MKS_pos_sp_deg = 0 and already at 0 deg) */
  if (mks_expected_move_ms == 0u) {
    return 1u;
  }

  /* 1. Driver explicitly reported move completion (0xFD/0xFE/0xF5 status = 2) */
  if (mks_move_status == MKS_MOVE_DONE) {
    return 1u;
  }

  /* 2. Encoder position converged to MKS_pos_sp_deg after expected travel time, or safety timeout elapsed */
  uint32_t elapsed_ms = now_tick - mks_move_start_tick;
  if ((elapsed_ms >= mks_expected_move_ms) &&
      ((fabsf(MKS_current_angle_deg - MKS_pos_sp_deg) <= 25.0f) ||
       (elapsed_ms >= (mks_expected_move_ms + 800u))))
  {
    return 1u;
  }

  return 0;
}

/**
  * @brief Check if MSK2_stepper (MKS2_pos_sp_deg = 3600 or 0) has finished its rotation
  */
static uint8_t MSK2_Move_Is_Done(uint32_t now_tick)
{
  /* If target change hasn't been latched or move command is still waiting for 20ms delay */
  if (msk2_move_pending || (fabsf(MSK2_stepper - msk2_last_target) > 0.05f)) {
    return 0;
  }

  /* If 0 pulses were needed (already at target angle) */
  if (msk2_expected_move_ms == 0u) {
    return 1u;
  }

  /* 1. Driver explicitly reported move completion (0xFD/0xFE/0xF5 status = 2) */
  if (msk2_move_status == MKS_MOVE_DONE) {
    return 1u;
  }

  /* 2. Encoder position converged to MSK2_pos_sp_deg after expected travel time, or safety timeout elapsed */
  uint32_t elapsed_ms = now_tick - msk2_move_start_tick;
  if ((elapsed_ms >= msk2_expected_move_ms) &&
      ((fabsf(MSK2_current_angle_deg - MSK2_pos_sp_deg) <= 25.0f) ||
       (elapsed_ms >= (msk2_expected_move_ms + 800u))))
  {
    return 1u;
  }

  return 0;
}

/**
  * @brief SCARA Robot Teach/Record & Sequential Playback State Machine
  *
  * Sequence:
  *   1. Startup Calibrate: Wait until M1, M2, and MKS #1 finish calibration to 0 deg.
  *   2. Press state_start = 1: Free motors (free_rotate_mode = 1) to teach/record positions.
  *   3. Press state_record = 1:
  *        - 1st press -> Position 1 (n = 0): record (M1, M2), MKS_pos_sp_deg = 0
  *        - 2nd press -> Position 2 (n = 1): record (M1, M2), MKS_pos_sp_deg = 5000
  *        - (n+1)th   -> Position (n+1):     record (M1, M2), MKS_pos_sp_deg = 5000 * n
  *   4. Press state_stop_record = 1: Lock motors (free_rotate_mode = 0) and wait for state_start hold.
  *   5. Hold state_start = 1 for 1 second (1000 ms):
  *        - All motors go to Calibrate first (M1, M2, MKS #1 -> 0 deg; MSK2 -> 0 deg).
  *        - Move to Position 1 (M1, M2, MKS_pos_sp_deg = 0). Wait until ALL are DONE.
  *        - Stop at Position 1 and rotate MSK2 to 3600 deg (MKS2_pos_sp_deg = 3600). Wait until DONE.
  *        - Move to Position 2 (M1, M2, MKS_pos_sp_deg = 5000). Wait until ALL are DONE.
  *        - Stop at Position 2 and rotate MSK2 to 0 deg (MKS2_pos_sp_deg = 0). Wait until DONE.
  *        - Continue for all recorded positions (Position n+1 => MKS = 5000 * n, MSK2 alternates 3600 <-> 0).
  */
static void SCARA_Process_Update(uint32_t now_tick)
{
  static uint8_t  prev_state_start       = 0;
  static uint8_t  prev_state_record      = 0;
  static uint8_t  prev_state_stop_record = 0;
  static uint32_t last_record_edge_tick  = 0;
  static uint32_t start_hold_start_tick  = 0;
  static uint8_t  start_hold_timing      = 0;
  static uint32_t state_step_enter_tick  = 0;
  static uint8_t  play_idx               = 0;
  static uint8_t  has_played_once        = 0;

  /* Read physical pins to know if state_* was set by button or Live Expressions */
  uint8_t pin_start  = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_SET) ? 1u : 0u;
  uint8_t pin_record = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)  == GPIO_PIN_SET) ? 1u : 0u;
  uint8_t pin_stop   = ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) == GPIO_PIN_SET) ||
                        (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)  == GPIO_PIN_SET) ||
                        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)  == GPIO_PIN_SET)) ? 1u : 0u;

  uint8_t stop_req   = (state_stop_record != 0u) || (state_playback != 0u);

  uint8_t start_rising  = (state_start != 0u)  && (prev_state_start == 0u);
  uint8_t record_rising = (state_record != 0u) && (prev_state_record == 0u);
  uint8_t stop_rising   = (stop_req != 0u)     && (prev_state_stop_record == 0u);

  prev_state_start       = (state_start != 0u) ? 1u : 0u;
  prev_state_record      = (state_record != 0u) ? 1u : 0u;
  prev_state_stop_record = (stop_req != 0u) ? 1u : 0u;

  switch (scara_state)
  {
    case SCARA_STATE_INIT_CALIBRATE:
    {
      /* 1. First Start is Calibrate: wait until M1, M2, and MKS #1 are all calibrated */
      if (M1_homed && M2_homed && MKS_homed) {
        scara_state = SCARA_STATE_IDLE_READY;
      }
      break;
    }

    case SCARA_STATE_IDLE_READY:
    {
      /* 2. When state_start is pressed, free motors for recording the First Motion */
      if (start_rising || (state_start != 0u)) {
        recorded_positions_count = 0;
        current_position_index   = 0;
        has_played_once          = 0;
        free_rotate_mode         = 1;
        start_hold_timing        = 0;
        start_hold_elapsed_ms    = 0;

        if (!pin_start) {
          /* Auto-clear if triggered from Live Expressions so it doesn't stay stuck at 1 */
          state_start      = 0;
          prev_state_start = 0;
        }
        scara_state = SCARA_STATE_TEACH_RECORD;
      }
      break;
    }

    case SCARA_STATE_TEACH_RECORD:
    {
      /* Keep motors free so user can move robot arm to Position 1, Position 2, ..., Position (n+1) */
      free_rotate_mode = 1;

      /* On each state_record = 1 rising edge (debounced 150ms), save Position (n+1) */
      if (record_rising && ((now_tick - last_record_edge_tick) >= 150u)) {
        last_record_edge_tick = now_tick;

        if (recorded_positions_count < SCARA_MAX_WAYPOINTS) {
          uint8_t n = recorded_positions_count;

          /* Clamp recorded M1 & M2 angles to valid travel limits */
          float rec_m1 = M1_current_angle_deg;
          if (rec_m1 < M1_min_deg) rec_m1 = M1_min_deg;
          if (rec_m1 > M1_max_deg) rec_m1 = M1_max_deg;

          float rec_m2 = M2_current_angle_deg;
          if (rec_m2 < M2_min_deg) rec_m2 = M2_min_deg;
          if (rec_m2 > M2_max_deg) rec_m2 = M2_max_deg;

          /* Position (n+1) => MKS_pos_sp_deg = 5000 * n (Pos 1 = 0, Pos 2 = 5000, ...) */
          float rec_mks = SCARA_MKS_STEP_DEG * (float)n;
          if (rec_mks > MKS_max_deg) {
            MKS_max_deg = rec_mks;
          }

          scara_waypoints[n].m1_deg  = rec_m1;
          scara_waypoints[n].m2_deg  = rec_m2;
          scara_waypoints[n].mks_deg = rec_mks;

          recorded_positions_count = n + 1u;
          current_position_index   = recorded_positions_count;
        }

        if (!pin_record) {
          /* Auto-clear Live Expression write so user can set state_record = 1 again for next Position */
          state_record      = 0;
          prev_state_record = 0;
        }
      }

      /* Until state_stop_record = 1 (press) -> exit free-rotate recording mode */
      if (stop_rising || stop_req) {
        free_rotate_mode      = 0;
        start_hold_timing     = 0;
        start_hold_elapsed_ms = 0;

        if (!pin_stop) {
          state_stop_record      = 0;
          state_playback         = 0;
          prev_state_stop_record = 0;
        }
        scara_state = SCARA_STATE_WAIT_START_HOLD;
      }
      break;
    }

    case SCARA_STATE_WAIT_START_HOLD:
    {
      free_rotate_mode = 0;

      /*
       * Hold state_start = 1 for 1s (1000 ms) -> All motors calibrate first, then start playback!
       * (If playback was already completed once and user does a short press < 800ms on state_start,
       *  re-enter Teach/Record mode to record a new sequence.)
       */
      if (state_start != 0u) {
        if (!start_hold_timing) {
          start_hold_timing     = 1;
          start_hold_start_tick = now_tick;
          start_hold_elapsed_ms = 0;
        } else {
          start_hold_elapsed_ms = now_tick - start_hold_start_tick;
          if (start_hold_elapsed_ms >= SCARA_HOLD_START_MS) {
            start_hold_timing     = 0;
            start_hold_elapsed_ms = SCARA_HOLD_START_MS;

            if (!pin_start) {
              state_start      = 0;
              prev_state_start = 0;
            }

            if (recorded_positions_count > 0u) {
              /* Trigger calibration on all calibrated motors (M1, M2, MKS #1) & return MSK2 to 0 deg */
              MSK2_stepper          = 0.0f;
              rehome_trigger        = 1;
              state_step_enter_tick = now_tick;
              scara_state           = SCARA_STATE_PLAY_CALIBRATE;
            } else {
              /* No waypoints recorded yet -> enter Teach/Record mode */
              free_rotate_mode = 1;
              scara_state      = SCARA_STATE_TEACH_RECORD;
            }
          }
        }
      } else {
        if (start_hold_timing) {
          uint32_t held_ms = now_tick - start_hold_start_tick;
          start_hold_timing     = 0;
          start_hold_elapsed_ms = 0;

          /* After a playback has finished, a short press (< 800ms) on state_start starts new recording */
          if (has_played_once && (held_ms >= 30u) && (held_ms < 800u)) {
            recorded_positions_count = 0;
            current_position_index   = 0;
            has_played_once          = 0;
            free_rotate_mode         = 1;
            scara_state              = SCARA_STATE_TEACH_RECORD;
            break;
          }
        }
      }
      break;
    }

    case SCARA_STATE_PLAY_CALIBRATE:
    {
      /* Wait until all motors (M1, M2, MKS #1) have finished calibration and MSK2 is at 0 deg */
      if ((rehome_trigger == 0u) &&
          M1_homed && M2_homed && MKS_homed &&
          MSK2_Move_Is_Done(now_tick))
      {
        /* Zero MSK2 reference cleanly at the start of playback so 0 <-> 3600 is exact */
        play_idx               = 0;
        current_position_index = 1; /* Position 1 */

        /* Command Position 1: M1, M2, and MKS_pos_sp_deg = 5000 * 0 = 0 */
        M1_target             = scara_waypoints[0].m1_deg;
        M2_target             = scara_waypoints[0].m2_deg;
        MKS_target_deg        = scara_waypoints[0].mks_deg; /* 0.0f for Position 1 */
        state_step_enter_tick = now_tick;
        scara_state           = SCARA_STATE_PLAY_MOVE_POS;
      }
      break;
    }

    case SCARA_STATE_PLAY_MOVE_POS:
    {
      /* Keep commanding current Position (play_idx + 1) targets */
      M1_target      = scara_waypoints[play_idx].m1_deg;
      M2_target      = scara_waypoints[play_idx].m2_deg;
      MKS_target_deg = scara_waypoints[play_idx].mks_deg; /* 5000 * play_idx */

      /* Allow at least 50ms for RMD_Axis_Update & MKS_Servo_Update to latch targets */
      if ((now_tick - state_step_enter_tick) < 50u) {
        break;
      }

      /* Wait until M1, M2, AND MKS_pos_sp_deg (5000 * n) are ALL DONE */
      if (RMD_Axis_Move_Is_Done(&axis_M1, M1_target, M1_current_angle_deg, now_tick) &&
          RMD_Axis_Move_Is_Done(&axis_M2, M2_target, M2_current_angle_deg, now_tick) &&
          MKS_Move_Is_Done(now_tick))
      {
        /*
         * Position (play_idx + 1) is DONE!
         * Stop M1, M2, MKS at current position and command MSK2_stepper:
         *   - After Position 1 (play_idx = 0): MSK2 rotates 0 -> 3600 deg
         *   - After Position 2 (play_idx = 1): MSK2 rotates 3600 -> 0 deg
         *   - Alternates 3600 <-> 0 on every subsequent Position (n + 1)
         */
        float next_msk2_deg   = ((play_idx % 2u) == 0u) ? SCARA_MSK2_ALT_DEG : 0.0f;
        MSK2_stepper          = next_msk2_deg;
        state_step_enter_tick = now_tick;
        scara_state           = SCARA_STATE_PLAY_ROTATE_MSK2;
      }
      break;
    }

    case SCARA_STATE_PLAY_ROTATE_MSK2:
    {
      /* Allow at least 50ms for MSK2_Stepper_Update to latch target and send CAN move command */
      if ((now_tick - state_step_enter_tick) < 50u) {
        break;
      }

      /* Wait until MSK2_stepper finishes rotating to 3600 deg (or 0 deg) */
      if (MSK2_Move_Is_Done(now_tick)) {
        play_idx++;

        if (play_idx < recorded_positions_count) {
          /* Advance to next recorded Position (play_idx + 1) */
          current_position_index = play_idx + 1u;
          M1_target              = scara_waypoints[play_idx].m1_deg;
          M2_target              = scara_waypoints[play_idx].m2_deg;
          MKS_target_deg         = scara_waypoints[play_idx].mks_deg; /* 5000 * play_idx */
          state_step_enter_tick  = now_tick;
          scara_state            = SCARA_STATE_PLAY_MOVE_POS;
        } else {
          /* All recorded positions completed! Ready to replay (hold state_start 1s) or re-record */
          has_played_once = 1;
          scara_state     = SCARA_STATE_WAIT_START_HOLD;
        }
      }
      break;
    }

    default:
      scara_state = SCARA_STATE_INIT_CALIBRATE;
      break;
  }
}

/* USER CODE END Application */
