#ifndef CONF_H
#define CONF_H

/* Used Pins */
#define PIN_TB6600_EN     8
#define dirPin            12
#define stepPin           13
#define PIN_BUTTON_RESET  10
#define PIN_BUZZER        9

#define MIN_ROT_PER_MINUTE 1
#define MAX_ROT_PER_MINUTE 50

#define TB6600_STEPS_PER_REV  3200
#define TB6600_PULSE_WIDTH_US 5

#define TIM_MS_INT_PRINT    (200U)
#define TIM_MS_INT_GUI      (100U) // To read Nex and check holding
#define TIM_MS_INT_CONTROL  (1U)

#define PRESET_TOTAL_TIME       (10 * 60) // 10 minutes = 600 seconds
#define PRESET_FIRST_CYCLE_TIME (30)      // 30 seconds
#define PRESET_PAUSE_TIME       (50)      // 50 seconds
#define PRESET_REP_CYCLE_TIME   (10)      // 10 seconds

#define PRESET_ANGLE_FW (185)
#define PRESET_ANGLE_BW (80)
#define PRESET_SPEED    (85)

/* Tone */
#define TONE_FREQ_HZ	1000
#define TONE_START_TIME_S 20 // seconds before the end
#define TONE_END_TIME_S   15 // seconds before the end. It should be always less than start time
#define TONE_INTERVAL_MS 1000


#endif // CONF_H