// delay between motor changing directions. to reduce strain on motors.
#define MOTOR_CHANGE_DIRECTION_DELAY_MILLIS 500
// if no speed is specified
#define MOTOR_DEFAULT_SPEED 150 
// gear ratio determined through expermentation. if an incorrect turret position is being reported, this value could be responsible.
#define TURRET_GEAR_RATIO 24000
// this calibration delay is so that the calibration is not triggered multiple times. probably safe to change.
#define TURRET_CALIBRATION_DELAY_MILLIS 2000