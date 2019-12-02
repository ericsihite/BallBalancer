/*  ============================================================================
	Dyno Cape Header
	----------------------------------------------------------------------------
	Header file for the motor dyno using Beaglebone Black and Robotics Cape.	
	
	Eric Nauli Sihite
	Spring/Summer 2018
============================================================================  */

#ifndef DYNO_CAPE_H
#define DYNO_CAPE_H

// Included libraries
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <math.h> // for M_PI
#include <roboticscape.h> 		// includes ALL robotics cape subsystems
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/*==============================================================================
	PARAMETER DECLARATIONS
==============================================================================*/

// Debug and startup settings --------------------------------------------------

// Debug
#define DISABLE_MOTORS			0	// disable the motors
#define DISABLE_EKF				0	// disable EKF for debugging motors
#define DISABLE_LIN_KF			0	// Skip linear KF.

// Console printing, disable to prevent crash if turned on for too long.
#define DISABLE_PRINTF_THREAD	1	// 1 to disable the console printf thread

// Print data to text file setting. 
// 1 = auto start, 0 = press pause button to start.
// THE ROBOT WILL BE ON STANDBY (NOT RUNNING) IF NOT RECORDING DATA.
#define START_FPRINTF			0	// 0 = press pause to start the robot.

// Experimental parameters -----------------------------------------------------

// Controller type
#define CONTROLLER_TYPE			2	// 1 = EKF, 2 = KF
#define USE_DMP_THETA			1	// Replaces theta estimates with DMP


// Starting and tipover conditions ---------------------------------------------

#define TIP_ANGLE		50		// Tipover angle (degree)
#define START_ANGLE		10		// Starting angle (degree)
#define START_DELAY		0.5		// Start delay before arming the motor(second)
#define INIT_IDLE_TIME	5		// Idle time when the program just started (sec)

// Control parameters and settings ---------------------------------------------

// Driving control command / drive rates
#define BT_DRIVE_RATE		6.2832		// default driving rate (rad/s)
#define BT_SPIN_RATE		6.2832		// default spinning rate (rad/s)

#define N_GAINSCH	5	// Number of gain scheduled values based on yaw rate

// Phi dot estimate low pass filter coefficients.
// wc = 10 rad/s = -0.9512, 15 rad/s = -0.9277, 20 rad/s = -0.9048
static const double PHIDOT_LP_WC[N_GAINSCH] = {	-0.9512,
												-0.9512,
												-0.9512,
												-0.9277,
												-0.9048};

// Theta reference SLC outer loop controller
#define SLC_COEF_1 			-0.000985 
static const double SLC_GAIN[N_GAINSCH]   = {	0.20, 
												0.20, 
												0.15, 
												0.10, 
												0.10};
									
static const double SLC_COEF_2[N_GAINSCH] = {	0.9704, 	// 6 rad/s
												0.9675, 	// 6.6 rad/s
												0.9634, 	// 7.5 rad/s
												0.9598, 	// 8.2 rad/s
												0.9512};	// 10 rad/s

// 

// Friction parameters ---------------------------------------------------------

// EKF friction model (EKF model)
#define VISCOUS_FC		0.001156*1		// viscous friction	coefficient
#define COULOMB_FC		0.0526*0		// coulomb friction coefficient xy
#define COULOMB_FC_Z	0.0526*0		// coulomb friction coefficient z
#define COULOMB_FC_R	0.1				// friction deadzone 

// Friction compensation in the controller (Controller compensation)
// Compensation values with respect to the controller command
#define COULOMB_FCOMP_X		0.30*0.99	// Coulomb friction constant (duty x)
#define COULOMB_FCOMP_Y		0.30*0.99	// Coulomb friction constant (duty y)
#define COULOMB_FCOMP_Z		0.30*0.99	// Coulomb friction constant (duty z)

// Friction and its compensation settings
#define FRIC_EXP			1.95	// Compensation smoothing rate (exp)
#define FRIC_Z_MULT_RIGHT	0.99	// z friction multiplier (right spin)







// Successive loop closure controller gains ------------------------------------

#define ENABLE_SLC_SIM		1	// Enable SLC theta reference




// State feedback controller gains ---------------------------------------------





// Math function parameters ----------------------------------------------------








// Unsorted and older commands -------------------------------------------------







// #define YAW_RATE_TRESHOLD_HI	2		// rad/s
// #define YAW_RATE_TRESHOLD_LO	0.5		// rad/s, must be different than hi
// EKF Coulomb friction scaler [0,1], 1 = the identified coefficient
// // Phi scaler for the no friction model KFs
// #define PHIREF_BOOST	1//1
// #define PHI_SCALER		1 //1 	// Scale phi in the controller 
// #define PHI_SCALER_M	1//0.3906 	// Scale phi in the model
// // #define PHI_MULTIPLIER		1.5





// #define BT_SPIN_RATE_L		6.2832*0.25
// #define BT_SPIN_RATE_R		6.2832*0.25
// #define BT_SPIN_RATE_LF		6.2832*0.5
// #define BT_SPIN_RATE_RF		6.2832*0.5



// #define REF_SCALER_SPIN		0.50		// Drive rate scaler when spinning


// // Coulomb friction parameters (model)
// #define COULOMB_FC_M		0.3936*0
// #define COULOMB_FC_SCALER	0.75*0


int flag_body_frame_drive;
int spinning_flag, spinning_flag_old;
double  offset_phi_x, offset_phi_y;

int slc_sim_enabled;			// SLC sim flag
double phi_e_x_old, phi_e_y_old, slc_theta_x, slc_theta_y;
double slc_theta_x_old, slc_theta_y_old, slc_thetaD_x, slc_thetaD_y;
double phiD_e_x_old, phiD_e_y_old;

double yaw_magnet, yaw_magnet_offset, yaw_magnet_old;
double phi_in_x, phi_in_y, phidot_in_x, phidot_in_y;
double phiRef_in_x, phiRef_in_y, phiRefdot_in_x, phiRefdot_in_y;

double theta_x_atan2, theta_y_atan2;


double offset_yaw;

double phi_ref_switch_x, phi_ref_switch_y, phi_ref_x, phi_ref_y;



// #define SLC_SIM_GAIN	0.25*1.5*0.4

// #define SLC_SIM_GAIN_SPIN	0.15*2.5
// #define SLC_SIM_SPD_GAIN		0.15*2.0










// // #define COULOMB_FCOMP_X	0.3936*0.75	// Coulomb friction constant (duty xy) 0.75
// // #define COULOMB_FCOMP_Y	0.3936*0.75	// Coulomb friction constant (duty xy) 0.75
// // #define COULOMB_FCOMP_Z	0.3936*0.00	// Coulomb friction constant (duty xy) 0.75

// //0.05706




// // #define WC_ACCEL	0.7408	// cutoff = 60 rad/s
// // #define WC_ACCEL	0.8187	// cutoff = 40 rad/s
// #define WC_ACCEL	0.9048	// cutoff = 20 rad/s
// double acc_1, acc_2, acc_3;


// #define SLC_COEF_1		-0.2
// #define SLC_COEF_2		0.2
// #define SLC_COEF_3		0.9704455335 // 6 rad/s

// 0.0009851488817



double slc_spd_coef_2;
double yaw_0;


// #define SLC_SIM_SPD_GAIN	0.15






double slc_pos_gain, slc_spd_gain, slc_target_gain;
int spin_count;

// #define SPIN_CT_THRESH 200	// 1 second


// #define SLC_SIM_GAIN_SPIN	0.15*1.5


// #define USE_PHI_ENCODER			0
#define USE_PHI_RESET			0

#define USE_PHI_ERROR_LIMIT		0
#define YAW_RATE_PHI_LIMIT		5
#define PHI_ERROR_LIMIT			2.5
#define PHIDOT_ERROR_LIMIT		20




#define USE_SS_SLC				0
#define USE_INPUT_ROTATION		0
#define USE_INPUT_FIX			0

#define USE_LP_PHIDOT			1
#define USE_POS_SLC				0
double lp_wc, lp_wc_target;




// Spin stop enabled
#define USE_SPIN_FAILURE_CHECK		1
#define SPIN_FAIL_THETA_THRESHOLD	0.30
#define SPIN_FAIL_PHI_THRESHOLD		15
#define SPIN_FAIL_PHID_THRESHOLD	15

#define SPIN_SAFE_THETA_THRESHOLD	0.075
#define SPIN_SAFE_PHI_THRESHOLD		5
#define SPIN_SAFE_PHID_THRESHOLD	2.5



#define N_MOV_AVG					100		// 0.5 second MA

double rc_z2, phid_error_ma;
int flag_spin_imbalance;


#define R_C		0.985	// Remote control input LP frequency (3 rad/s)
#define R_CZ	0.985	// Remote control input LP frequency (3 rad/s)
#define R_SLC	0.9704
#define R_CZ2	0.985	// 6 rad/s
#define R_K		0.9704	// Gain scheduling for drive/spin (6 rad/s)



#define USE_YAWRATE_CORRECTION		0
#define YAWRATE_CORRECTION_COEF		0.01



#define USE_SPIN_PHID_CONTROL_ONLY	0
#define PHIE_SCALER		2
#define PHIE_LIM_1		1		
#define PHIE_LIM_2		100.6
#define PHIE_LIM_3		100.4
#define PHIE_LIM_4		100.2


// pos slc
#define SLC_SIM_GAIN			0.225*1.95		//0.15*1.5*1.25 unused
#define SLC_COEF_POS_1			-0.2
#define SLC_COEF_POS_2			0.2
#define SLC_SIM_POS_GAIN		0.20 //0.225*1.75 // 0.20
#define SLC_SIM_POS_GAIN_S2		0.60 
#define SLC_SIM_POS_GAIN_S3		0.50
#define SLC_SIM_POS_GAIN_S4		0.40

// Q(1,50), R(100)
// #define K1_SS_SLC		-0.0998*1.0
// #define K2_SS_SLC		-0.8352*1.0

// Q(1,50), R(50) best?
// #define K1_SS_SLC		-0.1410*1.0
// #define K2_SS_SLC		-1.1298*1.0

// Q(1,50), R(20)
// #define K1_SS_SLC		-0.2227*1.0
// #define K2_SS_SLC		-1.7100*1.0

// Q(1,50), R(10)
// #define K1_SS_SLC		-0.3144*1.0
// #define K2_SS_SLC		-2.3601*1.0

// Q(10,20), R(10)
#define K1_SS_SLC		-0.9950*1.0
#define K2_SS_SLC		-1.9925*1.0

// Q(1,50), R(5)
// #define K1_SS_SLC		-0.4435*0.5
// #define K2_SS_SLC		-3.2747*0.5

// 2nd order low pass filter coefficients, wc = 10 rad/s
// #define LP_NUM_1	0.001209104274
// #define LP_NUM_2	0.00116946476
// #define LP_DEN_1	-1.902458849
// #define LP_DEN_2	0.904837418

#define LP_NUM_1	0.0488
#define LP_NUM_2	0
#define LP_DEN_1	-0.9512
#define LP_DEN_2	0.0


// 2nd order low pass filter coefficients, wc = 20 rad/s
// #define LP_NUM_1	0.00467884016
// #define LP_NUM_2	0.004377076846
// #define LP_DEN_1	-1.809674836
// #define LP_DEN_2	0.8187307531

// 2nd order low pass filter coefficients, wc = 40 rad/s
// #define LP_NUM_1	0.01752309631
// #define LP_NUM_2	0.01533544357
// #define LP_DEN_1	-1.637461506
// #define LP_DEN_2	0.670320046

// 2nd order low pass filter coefficients, wc = 60 rad/s
// #define LP_NUM_1	0.03693631311
// #define LP_NUM_2	0.03023888162
// #define LP_DEN_1	-1.481636441
// #define LP_DEN_2	0.5488116361



double phide_raw_x, phide_raw_y;



#define SLC_SS_B0_CONST		210.566 //213
#define SLC_SS_B0_SPD		3.4642 //4.14
#define SLC_SS_B1			7.2453 //8.90
#define SLC_SS_B2			-3.7736 //-4.76

double x1_slc, x2_slc, x3_slc, x4_slc, b0, b1, b2;

// 1st order LP filter (wc = 10 rad/s)
#define LP1_N0	 0
#define LP1_N1	 0.0488
#define LP1_N2	 0
#define LP1_D1	-0.9512
#define LP1_D2	 0
#define LP1_D1_S2	-0.9512		// 10 rad/s = -0.9512, 40 = -0.8187
#define LP1_D1_S3	-0.9512		// 15 rad/s = -0.9277, 45 = -0.6376
#define LP1_D1_S4	-0.9277		// 20 rad/s = -0.9048, 60 = -0.7408
#define LP1_D1_S5	-0.9048		// 30 rad/s = -0.8607, 90 = -0.6376

// #define LP1_D1_S2	-0.8607		// 10 rad/s = -0.9512, 40 = -0.8187
// #define LP1_D1_S3	-0.7985		// 20 rad/s = -0.9048, 30 = -0.8607
// #define LP1_D1_S4	-0.7408		// 15 rad/s = -0.9277, 60 = -0.7408
// #define LP1_D1_S5	-0.6376		// 20 rad/s = -0.9048, 30 = -0.8607

// #define LP1_D1_S2	-0.9512		// 10 rad/s = -0.9512, 40 = -0.8187
// #define LP1_D1_S3	-0.9277		// 15 rad/s = -0.9277, 60 = -0.7408
// #define LP1_D1_S4	-0.9048		// 20 rad/s = -0.9048, 30 = -0.8607

// 45 rad/s 0.6376
// 90 rad/s 0.6376

// Cap the resulting SLC theta ref
#define THETA_REF_CAP		0.1*3.0		//0.3
#define THETAD_REF_CAP		0.5*5.0	//1
#define SLC_PRESCALER		0.99


// spd slc
#define SLC_SIM_SPD_GAIN		0.20 //0.225*1.75 // 0.20
#define SLC_SIM_SPD_GAIN_S2		0.20
#define SLC_SIM_SPD_GAIN_S3		0.15
#define SLC_SIM_SPD_GAIN_S4		0.10
#define SLC_SIM_SPD_GAIN_S5		0.10

#define SLC_COEF_SPD_1			-0.000985   // -0.00147	old with 0.3
#define SLC_COEF_SPD_2			0.9704 		// 0.9704455335		// 6 rad/s
#define SLC_COEF_SPD_2_S2		0.9675		// 6.6 rad/s
#define SLC_COEF_SPD_2_S3		0.9634		// 7.5 rad/s
#define SLC_COEF_SPD_2_S4		0.9598		// 8.2 rad/s
#define SLC_COEF_SPD_2_S5		0.9512		// 10 rad/s

// 8 = 0.9608,  9 = 0.9560, 10 = 0.9512
//12 rad/s= 0.9417645336



// #define LP_WC				0.985	// 3 rad/s = 1 sec rise time
#define LP_WC				0.97	// 6 rad/s = 0.5 sec rise time
#define LP_Q				0.97	// 6 rad/s = 0.5 sec rise time


#define K_SCALING	1
// #define R_K			0.97	// Gain scheduling for drive/spin (6 rad/s)


// not spinning controller
#define CONT_K1			10 		//8.5		// 
#define CONT_K2			0.4 	//0.3		// 
#define CONT_K3			0.5 	//0.4		// 0.6 max (some jitter)
#define CONT_K4			0.075 	//0.075		// 0.5 max no lp, 1.0 max with lp


#define USE_VAR_GAINS	1
// #define USE_VAR_EST		0


// 0.5 Hz yaw rate // good but overshoot
#define CONT_K1_S2		12			//8.0
#define CONT_K2_S2 		0.4		//0.1
#define CONT_K3_S2		0.5			//0.4
#define CONT_K4_S2		0.10		//0.05
#define CONT_M_K1_S2	0.667*0
#define CONT_M_K2_S2	0.048*0
#define CONT_M_K3_S2	0.026*0
#define CONT_M_K4_S2	0.031*0



// 0.75 Hz yaw rate // overshoot? more spd? less theta gain?
#define CONT_K1_S3		14
#define CONT_K2_S3 		0.4	//0.3
#define CONT_K3_S3		0.6
#define CONT_K4_S3		0.10
#define CONT_M_K1_S3	1.334*0
#define CONT_M_K2_S3	0.221*0
#define CONT_M_K3_S3	0.119*0
#define CONT_M_K4_S3	0.144*0


// 1.0 Hz yaw rate
#define CONT_K1_S4		14
#define CONT_K2_S4 		0.5		//0.3
#define CONT_K3_S4		0.6
#define CONT_K4_S4		0.085
#define CONT_M_K1_S4	2.267*0
#define CONT_M_K2_S4	0.462*0
#define CONT_M_K3_S4	0.229*0
#define CONT_M_K4_S4	0.289*0

// 1.5 Hz yaw rate
#define CONT_K1_S5		12
#define CONT_K2_S5 		0.5		//0.3
#define CONT_K3_S5		0.45
#define CONT_K4_S5		0.050
#define CONT_M_K1_S5	2.267*0
#define CONT_M_K2_S5	0.462*0
#define CONT_M_K3_S5	0.229*0
#define CONT_M_K4_S5	0.289*0


// Yaw controller
#define CONT_KZ1		0.4*0.95		// theta z		0.4
#define CONT_KZ2		1.2*0.95		// theta dot z	1.2

// Friction
// #define USE_FRICTION_COMP	0		// Enable friction compensation
// #define FRICTION_COMP_TYPE	1		// Comp type: 1 = boost, 2 = speed














// #define WC_DDIFF	0.7304		// Dirty differentiator cutoff (10 Hz)
// #define GAIN_DDIFF	62.832	// Dirty differentiator gain (10 Hz)
// #define WC_DDIFF	0.8546		// Dirty differentiator cutoff (5 Hz)
// #define GAIN_DDIFF	31.416		// Dirty differentiator gain (5 Hz)
#define WC_DDIFF	0.9691		// Dirty differentiator cutoff (1 Hz)
#define GAIN_DDIFF	6.2832	// Dirty differentiator gain (1 Hz)



// #define R_CZ	0.97	// Remote control input LP frequency (6 rad/s)




// Controller gains


// // Theta reference adjustment gains (not needed)
// #define SLC_GAIN_I		-0.00001*0	// theta_ref adjustment to phi_dot 
// #define SLC_GAIN_D		-0.002*0	// theta_ref adjustment to phi_dot 
// #define SLC_I_SAT		0.5				// integrator saturation lim





// // Braking helper
// #define USE_BRAKE		0




// Bluetooth and data recording settings ---------------------------------------

// Bluetooth options
#define UART5_BUS	5		// Bluetooth bus
#define BUF_SIZE	32		// Bluetooth buffer length
#define BAUDRATE 	115200	// Baud rate
#define TIMEOUT_S 	1		// Time out wait time (seconds)

// fprintf parameters
#define FILENAME 			"mbbr_data.txt"		// fprintf filename
#define RECORD_TIME_END		5		// End record time (minutes)
#define DATA_COL_LENGTH		100		// Data column length
#define SKIP_REC_STEP		1		// Data skip n steps (0,1,3 only please)

// Thread and sampling frequencies ---------------------------------------------

#define SAMPLING_RATE_HZ	200		// Sampling rate (Hz)
#define PRINTF_RATE_HZ		50		// Console printf rate (Hz)
#define FPRINTF_RATE_HZ		100		// Text fprintf rate (Hz)
#define SETPOINT_RATE_HZ	50		// Setpoint thread frequency (Hz)
#define	DT_S 				0.005	// Sampling period (second)

// I/O parameters --------------------------------------------------------------

#define ENC_CPR		1080		// Encoder counts per revolution

// Drive/coast compensation ----------------------------------------------------

// Drive/coast compensation setup
#define USE_DRIVE_COAST_COMPENSATION 	1	// Enable drive/coast compensation
#define PWM_MODEL	2		// 0 = low PWM, 1 = high PWM, 2 = standard

// PWM setting
#define PWM_FREQ	20000		// Motor PWM frequency (Hz)

// Newton raphson parameters
#define NR_N_MAX			10		// Maximum Newton-Raphson iteration
#define NR_TOL				1e-6	// Newton-Raphson convergence tolerance


// Motor model parameters
#define MTR_MODEL_V		8.00			// Battery Voltage
#define MTR_MODEL_K		0.0742			// Torque gain
#define MTR_MODEL_R		3.3854			// Motor internal resistance
#define MTR_MODEL_L		0.00117*0.5		// Motor inductance
#define WR_MAX			107 			// maximum motor speed rad/s

// Experimental?
#define USE_MOTOR_BOOST		1

// Vector sizes and Kalman Filter setup ----------------------------------------

// 3D model Extended Kalman Filter vectors length
#define NX		13	// State vector length
#define NY		8	// Measurement vector length
#define NU		3	// Input vector length

// Linear model Kalman Filter vectors length
#define NX_L	8	// State vector length
#define NY_L	6	// Measurement vector length
#define NU_L	2	// Input vector length

// Controller vectors length
#define NX_C	10  // Controller states
#define NU_C	3	// Controller commands

// Tap filter size and polynomial power
#define N_TAP	5	// 5, 10, 20 only pls
#define P_TAP	1

// Initial KF P matrix value (assume diagonal)
#define P0					0.00001 	

// Override matrix inversion tolerance for James' library
#define USE_CUSTOM_ZERO_TOLERANCE	1		// Custom zero tolerance
#define MY_ZERO_TOLERANCE			1e-50	// need to be small enough

// Offset values ---------------------------------------------------------------

// Theta xy
#define THETA_OFFSET_X	0	// Theta offset x
#define THETA_OFFSET_Y	0	// Theta offset y

// Gyro xyz
#define GYRO_OFFSET_X	0	// Gyro offset x
#define GYRO_OFFSET_Y	0	// Gyro offset y
#define GYRO_OFFSET_Z	0	// Gyro offset z

// Accelerometer tilt/orientation angles
#define ACCEL_ANGLE_X	0*0.0175	// accel tilt angle about x-axis (rad)
#define ACCEL_ANGLE_Y	0*0.0175	// accel tilt angle about y-axis (rad)
#define ACCEL_ANGLE_Z	0*0.0175	// accel tilt angle about z-axis (rad)

// Accelerometer scaling
#define ACCEL_G_SCALE			1	// Accelerometer measurement scaler

// Transformation matrix
#define T_MATRIX_YAW	10	// Transformation matrix yaw correction (degrees)

// Pin setups ------------------------------------------------------------------

// Motor channels
#define MOTOR_CH_1		1	// Motor channel 1
#define MOTOR_CH_2		3	// Motor channel 2
#define MOTOR_CH_3		4	// Motor channel 3

// Encoder channels
#define ENC_CH_1	1	// Encoder channel 1
#define ENC_CH_2	2	// Encoder channel 2
#define ENC_CH_3	3	// Encoder channel 3

// I/O Polarities --------------------------------------------------------------

// Motor polarities
#define MTR_SGN_1	1	// Motor polarity 1
#define MTR_SGN_2	-1	// Motor polarity 2
#define MTR_SGN_3	1	// Motor polarity 3

// Encoder polarities
#define ENC_SGN_1	-1	// Encoder polarity 1
#define ENC_SGN_2	1	// Encoder polarity 2
#define ENC_SGN_3	1	// Encoder polarity 3

// IMU polarities
#define TX_SGN		1	// Theta DMP polarity x
#define TY_SGN		1	// Theta DMP polarity y
#define TZ_SGN		1	// Theta DMP polarity z
#define WX_SGN		1	// Gyro polarity x
#define WY_SGN		1	// Gyro polarity y
#define WZ_SGN		1	// Gyro polarity z
#define AX_SGN		-1 	// Accel polarity x
#define AY_SGN		1	// Accel polarity y
#define AZ_SGN		-1	// Accel polarity z

// Extras ----------------------------------------------------------------------

// Dimension parameter
#define R_WHEEL		12.5	// mm
#define R_BALL		32.0	// mm

// Math constants
#define PI			3.14159
#define TWO_PI		6.2831853

/*==============================================================================
	STRUCT AND ENUMERATES
==============================================================================*/

typedef struct mbbr_states{
	// Kalman filter states and measurement variables.
	
	// Vectors
	rc_vector_t x;		// Estimated states
	rc_vector_t f;		// Predicted states f(x,u,t)
	rc_vector_t y;		// Measurements
	rc_vector_t h;		// Predicted measurements h(x,u,t)
	rc_vector_t u;		// Controller values [x,y,z]
	
	// Matrices
	rc_matrix_t F;		// Jacobian of for f(x,u,t)
	rc_matrix_t H;		// Jacobian of for h(x,u,t)
	
	rc_matrix_t Q;		// Process noise covariance matrix
	rc_matrix_t R;		// Measurement noise covariance matrix
	rc_matrix_t Pi;		// Initial KF covariance matrix
	
	// Scalars
	int Nx;				// State vector length
	int Ny;				// Measurement vector length
	int Nu;				// Input vector length
	double P_init;		// Initial KF covariance scaler
	
} mbbr_states;

typedef struct io_data{
	// Raw sensor measurements and wheel controller values
	
	// Vectors, all length of 3.
	rc_vector_t theta;	// IMU DMP estimates
	rc_vector_t gyro;	// IMU raw gyrometer measurements
	rc_vector_t accel;	// IMU raw accelerometer measurements
	rc_vector_t magnet;	// IMU magnetometer measurements
	
	rc_vector_t phi;		// Ball spin
	rc_vector_t phi_old;	// Ball spin previous step
	rc_vector_t phid;		// Ball spin rate (dirty differentiator est)
	
	rc_vector_t phi_w;		// Wheel spin
	rc_vector_t phi_w_old;	// Wheel spin previous step
	rc_vector_t phid_w;		// Wheel spin rate (dirty differentiator est)
		
	rc_vector_t xc;		// Controller states
	rc_vector_t xr;		// States references
	rc_vector_t xe;		// States tracking error
	rc_vector_t u;		// Controller values [x,y,z]
	rc_vector_t uw;		// Wheel Controller values [w1,w2,w3]
	rc_vector_t uw_dc;	// Wheel Controller values [w1,w2,w3] drive-coast
	
	
	
	
	
	// Matrix (constants)
	rc_matrix_t T_b2w;	// Transformation matrix body -> wheel
	rc_matrix_t T_w2b;	// Transformation matrix wheel -> body
	rc_matrix_t C;		// Drive-coast surface, input: linear model u, omega_r
	rc_matrix_t R_accel;	// Accelerometer rotation
	
	// Matrix non constants
	rc_matrix_t K_target;	// Controller gain target
	rc_matrix_t K;			// Controller gain (x,y,z)
	
	// Tap filter variables
	rc_matrix_t A_tap;	// Pseudoinverse of the matrix A
	rc_vector_t tr_x;	// Theta reference values x
	rc_vector_t tr_y;	// Theta reference values y
	rc_vector_t w_1;	// Wheel angles 1
	rc_vector_t w_2;	// Wheel angles 2
	rc_vector_t w_3;	// Wheel angles 3	
	
	// Experimental
	rc_matrix_t Ru;
	rc_matrix_t Tu;
	rc_vector_t phid_e_ma;
	
	
	
	// Scalar
	double w_max;		// Maximum wheel speed
	double phid_i_x, phid_i_y;	// Inertial wheel speed.
	
} io_data;

typedef struct lp2_filter{
	rc_vector_t yk;
	rc_vector_t uk;
	double num0, num1, num2, den1, den2;
} lp2_filter;

// Driving states
enum drv_state {STOP,DRIVING};
enum spin_state {NOT_SPINNING,SPINNING};
enum estimator_state {KF,EKF,RAW};

// Motor enabled or disabled
enum mtr_state {ARMED, DISARMED};

/*==============================================================================
	VARIABLES DECLARATIONS
==============================================================================*/

// Struct variable declarations
mbbr_states 	s, sl;			// EKF States structs
io_data			uy;				// Input / output variables
rc_kalman_t		ekf, ekfl;		// Cape KF structs
rc_mpu_data_t 	mpu_data;		// Cape IMU data structs

lp2_filter	x1_ss, x2_ss, x3_ss, x4_ss;
lp2_filter 	phid_fx, phid_fy, phid_in_fx, phid_in_fy;



// Enumerate declarations
enum drv_state 			driving_state;
enum spin_state			spinning_state, spinning_state_old;
enum mtr_state 			motor_state;
enum estimator_state	est_st, est_st_old;

// Bluetooth variables
int 	uart2_filestream;		// UART2 file descriptor
uint8_t rx_buffer[BUF_SIZE];	// RX Buffer
double 	bt_x, bt_y;				// Joystick data
int 	bt_button;				// Button data
int 	bt_start;				// Start/select button data
int 	uart_bus;
int 	read_count;

// Time stamp variables 
double 	t_ms, t_mode, t_start, t_end, t_func;	// Time stamps
struct 	timespec t0, t1; 			// clock_gettime struct

// fprintf variables 
extern char rec_filename[];	// filename variable
int data_row, data_col;		// fprintf row and column length
int rec_count, rec_left, n_skip;	// Counters
double *data;				// Data array pointer (1D array)
int fprintf_not_done;		// Ending flag

// Other variables
double v_battery;
rc_vector_t temp_vec_1, temp_vec_2, temp_vec_3;
double theta_ref_i_x, theta_ref_i_y;
double temp_d_1, temp_d_2, temp_d_3, temp_d_4, temp_d_5, temp_d_6;
double yaw_forward, theta_z_in;

double rc_x, rc_y, rc_z, phi_hold_x, phi_hold_y;
int start_fprintf;

int temp_i_1;

double vx, vy;

double slc_target_wc;

double spin_rate, drive_rate_x, drive_rate_y;
double phi_if_x, phi_if_y, phid_if_x, phid_if_y;
int count_test_1, count_test_2;

int drv_st_new, drv_st_old;




/*==============================================================================
	FUNCTION DECLARATIONS
==============================================================================*/

// Thread function declarations
void mbbr_imu_interrupt_loop();
void* setpoint_loop(void* ptr);
void* printf_loop(void* ptr);	
void* fprintf_loop(void* ptr);

// Initialization functions
void init_mbbr_states(mbbr_states* s, int Nx, int Ny, int Nu, double Pi);
void init_input_output_data(io_data* uy);

// Reset functions
void reset_variables();
void reset_mbbr_states(mbbr_states* s);
void reset_io_data(io_data* uy);

// Exit functions
void exit_mbbr_states(mbbr_states* s);
void exit_io_data(io_data* uy);

// Sensor and motor functions
void update_measurements();
void update_controller();
void disarm_controller();
void arm_controller();

// Motor functions and drive-coast compensation functions
double drive_coast_comp(double u, double w, double Vb, int model, double L);
void coast_model(double* f, double* df, 
					double v, double wr, double it, double tau_r, double is);
// double interpolate_mesh(double x, double y, rc_matrix_t A); // unused

void set_controller_gain(rc_matrix_t* K, int spin_flag);


// Bluetooth functions
void init_bluetooth();
void read_bluetooth();

// Math functions
double angle_difference(double x1, double x2);
double abs_d(double x);
double sign_d(double x);
double saturate_d(double x, double min, double max);
double theta_xy_mag(rc_vector_t theta);
double sign_d_friction(double x, double r);
double friction_comp_exp(double x, double a, double c);

// Button functions
void on_mode_release();
void on_pause_release();

// sspace_data.c functions
void init_noise_data_3d(mbbr_states* s);
void init_noise_data_lin(mbbr_states* s);
void init_transformation_matrix(rc_matrix_t* T_b2w, rc_matrix_t* T_w2b);
//void init_drive_coast_surface(rc_matrix_t* C, int NC);	// unused
void mbbr_model_3d(mbbr_states* s);
void mbbr_model_lin(mbbr_states* s);


void init_tap_filter_matrix(rc_matrix_t* A, int n, int p);
double tap_filter_diff(rc_matrix_t A, rc_vector_t* f, double fk);

double filter_2nd(lp2_filter* lp, double u);
void init_filter_2nd(lp2_filter* lp, 
			double n0, double n1, double n2, double d1, double d2);
void reset_filter_2nd(lp2_filter* lp);
void exit_filter_2nd(lp2_filter* lp);

void input_transformation_matrix(rc_matrix_t* Ru, double x1, double x2);
void init_phi_transformation(rc_matrix_t* T);
void update_ekf_noise_covariance(mbbr_states* s, int spin_flag);

void filter_2nd_coef_update(lp2_filter* lp, double wc);
double moving_average_update(rc_vector_t* ma, double x, double y);

#endif















