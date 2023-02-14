#include "main.h"

#define GTERM_IMPL
#include "pal/gterm.h"

typedef enum {
  kDeviceTypeNoSensor = 0,
  kDeviceTypeMotorSensor = 2,
  kDeviceTypeAbsEncSensor = 4,
} V5_DeviceType;

int32_t vexDeviceGetStatus(V5_DeviceType *buffer);

/////////////////////////

int motor_port = -1;
int sensor_port = -1;

void display_error(const char *text);

void RemoveProsWarningScreen() {
  display_error("test");

  lv_obj_t *child = NULL;
  while (child = lv_obj_get_child(lv_scr_act(), child)) {
    if (lv_obj_get_hidden(child)) continue;

    display_error("");

    if (lv_obj_get_hidden(child)) {
      lv_obj_set_pos(child, 1000, 1000);
      break;
    }

    display_error("test");
  }
}

void WaitPortChange(int maximum_wait_ms) {
  V5_DeviceType type_set1[32];
  vexDeviceGetStatus(type_set1);

  uint32_t start_time = millis();

  while (maximum_wait_ms == 0 || millis() - start_time < maximum_wait_ms) {
    V5_DeviceType type_set2[32];
    vexDeviceGetStatus(type_set2);

    if (memcmp(type_set1, type_set2, sizeof(type_set1)) != 0) {
      break;
    }
    delay(10);
  }
}

int FindPort(V5_DeviceType wanted_type) {
  V5_DeviceType type_set1[32];
  vexDeviceGetStatus(type_set1);

  for (int zero_port = 0; zero_port < 21; zero_port++) {
    if (type_set1[zero_port] == wanted_type) {
      return zero_port + 1;
    }
  }

  return -1;
}

void initialize() {
  RemoveProsWarningScreen();

  gterm_init(NULL);

BEGIN:

  while ((sensor_port = FindPort(kDeviceTypeAbsEncSensor)) == -1 ||
         (motor_port = FindPort(kDeviceTypeMotorSensor)) == -1) {
    gterm_print("Please connect a motor and a sensor to the V5 Brain");
    WaitPortChange(0);
  }

  gterm_print("Using motor port %d and sensor port %d", motor_port, sensor_port);
  delay(1000);
  gterm_print("Running self-test sequence");

  for (int is_reversed = 0; is_reversed < 2; is_reversed++) {
    /* Configure the motor for green, coast */
    motor_set_brake_mode(motor_port, E_MOTOR_BRAKE_COAST);
    motor_set_gearing(motor_port, E_MOTOR_GEAR_GREEN);
    motor_set_zero_position(motor_port, 0.0);
    motor_set_encoder_units(motor_port, E_MOTOR_ENCODER_ROTATIONS);
    motor_set_reversed(motor_port, false);

    /* Reset the sensor */
    rotation_reset_position(sensor_port);

    if (!is_reversed) {
      /* TEST 1 - Run motor forward for 3 seconds, track position counters and disconnect */
      gterm_print("TEST 1 - #0000ff FORWARD# performance");
    } else {
      /* TEST 2 - Run motor backward for 3 seconds, track position counters and disconnect */
      gterm_print("TEST 2 - #0000ff REVERSE# performance");
    }

    int cnt_poserr = 0;
    double position_track = 0.0;
    double position_sense = 0.0;
    double speed_track = 0.0;
    double speed_sense = 0.0;

    for (int i = 0; i < (3000 / 10); i++) {
      motor_move_voltage(motor_port, is_reversed ? -12000 : 12000);

      /* Read data from motor and sensor */
      speed_track = motor_get_actual_velocity(motor_port);
      speed_sense = (rotation_get_velocity(sensor_port) * 1.0) * (1.0 / 360.0) * (60.0);
      position_track = motor_get_position(motor_port);
      position_sense = (rotation_get_position(sensor_port) * 1.0) * (1.0 / 36000.0);

      /* If sensed position is different from tracked position, report it */
      if (fabs(position_track - position_sense) > 0.2) {
        cnt_poserr++;
      }

      delay(10);
    }

    /* Stop the motor after the test */
    motor_move(motor_port, 0);

    /* Strings for pass/fail */
    static const char *str_pf[] = {"FAIL", "PASS"};
    static const char *str_pf_col[] = {"#ff0000 FAIL#", "#00ff00 PASS#"};

    gterm_print("REPORT:");
    /* Can reach 200rpm (green cartridge)? */
    bool speed_reached = is_reversed ? (speed_sense < -200.0) : (speed_sense > 200.0);
    gterm_print("Motor reached speed of %f > 200? %s", speed_sense, str_pf_col[speed_reached]);

    /* No position errors during test */
    bool pos_pass = (cnt_poserr == 0);
    gterm_print("Motor tracked position matched? %s", str_pf_col[pos_pass]);
  }

  gterm_print("Unplugged the motor or start again in 10 seconds");
  WaitPortChange(10000);

  goto BEGIN;
}
