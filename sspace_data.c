/*==============================================================================
	CONTAINS ALL THE DATA FOR THE STATE SPACE VARIABLES
	---------------------------------------------------
	Variable lists:
		Noise covariance data matrices: Q, R
		Controller gain
		Transformation matrix
		Discrete time f(x,u) and it's Jacobian
		Discrete time h(x,u) and it's Jacobian
==============================================================================*/

#include "mbbr_ekf.h"

/*==============================================================================
	KALMAN FILTER NOISE COVARIANCE DATA
==============================================================================*/



void init_noise_data_3d(mbbr_states* s){
	/*==========================================================================
		Initialize all noise covariance matrix. Constant value.
		Q = state noise covariance
		R = measurement noise covariance
		
		Calculate this from  Matlab.
	==========================================================================*/
	
	// More Q = trust measurement more
	// More R = trust model more
	
	// Dynamic Kalman Filter process noise data	
	s->Q.d[0][0]  	= 0.00001;	// Theta x 	0.000001
	s->Q.d[1][1]  	= 0.00001;	// Theta y	0.000001
	s->Q.d[2][2]  	= 0.0001;		// Theta z	0.00000001
	s->Q.d[3][3]  	= 0.0001;		// Phi x	0.1
	s->Q.d[4][4]  	= 0.0001;		// Phi y	0.1
	s->Q.d[5][5]  	= 0.1*1;		// Theta dot x	0.1
	s->Q.d[6][6]  	= 0.1*1;		// Theta dot y	0.1
	s->Q.d[7][7]  	= 0.1*1; 		// Theta dot z 	2.0
	s->Q.d[8][8]  	= 0.001*1;		// Phi dot x   	10
	s->Q.d[9][9]	= 0.001*1;		// Phi dot y	10
	s->Q.d[10][10]	= 0.001*1;	// enc phi x aug
	s->Q.d[11][11]	= 0.001*1;	// enc phi y aug
	s->Q.d[12][12]	= 0.001*1;	// enc phi z aug
	
	// Measurement Noise
	s->R.d[0][0]	= 0.0000020764*1;	//0.0000020764*1;			// wx
	s->R.d[1][1]	= 0.0000020764*1;	//0.0000020764*1;			// wy
	s->R.d[2][2]	= 0.0000020764*1;	//0.0000020764*1;			// wz
	s->R.d[3][3]	= 0.00007615435495*1;	//0.00007615435495*1;		// phix
	s->R.d[4][4]	= 0.00007615435495*1;	//0.00007615435495*1;		// phiy
	s->R.d[5][5]	= 0.00007615435495*1;	//0.00007615435495*1;		// phiz
	s->R.d[6][6]	= 0.07039*20*1;	//0.00070390*1000;		// ax   40
	s->R.d[7][7]	= 0.07039*20*1;	//0.00070390*1000;		// ay
	// s->R.d[8][8]	= 0.015; // Magnetometer
	
	return;
}



void init_noise_data_lin(mbbr_states* s){
	/*==========================================================================
		Initialize all noise covariance matrix. Constant value.
		Q = state noise covariance
		R = measurement noise covariance
		
		Calculate this from  Matlab.
	==========================================================================*/
	
	// Dynamic Kalman Filter process noise data	
	s->Q.d[0][0]  	= 0.00001;	// Theta x 0.00000001
	s->Q.d[1][1]  	= 0.00001;	// Theta y
	s->Q.d[2][2]  	= 0.0001;	// Phi x
	s->Q.d[3][3]  	= 0.0001;	// Phi y
	s->Q.d[4][4]  	= 0.1*1;		// Theta dot x
	s->Q.d[5][5]  	= 0.1*1;		// Theta dot y
	s->Q.d[6][6]  	= 0.001*1;	// Phi dot x
	s->Q.d[7][7]  	= 0.001*1;	// Phi dot y
	
	// Measurement Noise
	s->R.d[0][0]  	= 0.0000020764*1;	// Gyro x 1
	s->R.d[1][1]  	= 0.0000020764*1;	// Gyro y
	s->R.d[2][2]  	= 0.00007615435495*1;	// Encoder x
	s->R.d[3][3]  	= 0.00007615435495*1;	// Encoder y
	s->R.d[4][4]  	= 0.07039*20;	// Accel x 1000
	s->R.d[5][5]  	= 0.07039*20;	// Accel y
	
	return;
}

void update_ekf_noise_covariance(mbbr_states* s, int spin_flag){
	// Target noise
	rc_vector_t qk = rc_vector_empty();
	rc_vector_t rk = rc_vector_empty();
	rc_vector_zeros(&qk, NX);
	rc_vector_zeros(&rk, NY);
	
	if (spin_flag == 2){
		// Spinning fast noise
		qk.d[0] = 0.00001;	// Theta x
		qk.d[1] = 0.00001;	// Theta y
		qk.d[2] = 0.00001;	// Theta z
		qk.d[3] = 0.1;	// Phi x
		qk.d[4] = 0.1;	// Phi y
		qk.d[5] = 0.1*1;	// Theta dot x
		qk.d[6] = 0.1*1;	// Theta dot y
		qk.d[7] = 0.1*20;	// Theta dot z
		qk.d[8] = 0.0001;	// Phi dot x
		qk.d[9] = 0.0001;	// Phi dot y
		qk.d[10] = 0.001;	// Enc phi aug x or enc phi x
		qk.d[11] = 0.001;	// Enc phi aug y or enc phi y
		qk.d[12] = 0.001;	// Enc phi aug z or enc phi z		
		
		rk.d[0] = 0.0000020764;	// Gyro x
		rk.d[1] = 0.0000020764;	// Gyro y
		rk.d[2] = 0.0000020764;	// Gyro z
		rk.d[3] = 0.00007615435495;	// Enc x
		rk.d[4] = 0.00007615435495;	// Enc y
		rk.d[5] = 0.00007615435495;	// Enc z
		rk.d[6] = 0.07039*20;	// Accel x
		rk.d[7] = 0.07039*20;	// Accel y
	}
	if (spin_flag == 3){
		// Spinning fast noise
		qk.d[0] = 0.00001;	// Theta x
		qk.d[1] = 0.00001;	// Theta y
		qk.d[2] = 0.00001;	// Theta z
		qk.d[3] = 0.1*1;	// Phi x
		qk.d[4] = 0.1*1;	// Phi y
		qk.d[5] = 0.1*1;	// Theta dot x
		qk.d[6] = 0.1*1;	// Theta dot y
		qk.d[7] = 0.1*20;	// Theta dot z
		qk.d[8] = 0.0001;	// Phi dot x
		qk.d[9] = 0.0001;	// Phi dot y
		qk.d[10] = 0.001;	// Enc phi aug x or enc phi x
		qk.d[11] = 0.001;	// Enc phi aug y or enc phi y
		qk.d[12] = 0.001;	// Enc phi aug z or enc phi z		
		
		rk.d[0] = 0.0000020764;	// Gyro x
		rk.d[1] = 0.0000020764;	// Gyro y
		rk.d[2] = 0.0000020764;	// Gyro z
		rk.d[3] = 0.00007615435495;	// Enc x
		rk.d[4] = 0.00007615435495;	// Enc y
		rk.d[5] = 0.00007615435495;	// Enc z
		rk.d[6] = 0.07039*20;	// Accel x
		rk.d[7] = 0.07039*20;	// Accel y
	}
	if (spin_flag == 4){
		// Spinning fast noise
		qk.d[0] = 0.00001;	// Theta x
		qk.d[1] = 0.00001;	// Theta y
		qk.d[2] = 0.001;	// Theta z
		qk.d[3] = 0.1*1;	// Phi x
		qk.d[4] = 0.1*1;	// Phi y
		qk.d[5] = 0.1*1;	// Theta dot x
		qk.d[6] = 0.1*1;	// Theta dot y
		qk.d[7] = 0.01*1;	// Theta dot z
		qk.d[8] = 0.0001*1;	// Phi dot x
		qk.d[9] = 0.0001*1;	// Phi dot y
		qk.d[10] = 0.001;	// Enc phi aug x or enc phi x
		qk.d[11] = 0.001;	// Enc phi aug y or enc phi y
		qk.d[12] = 0.001;	// Enc phi aug z or enc phi z		
		
		rk.d[0] = 0.0000020764;	// Gyro x
		rk.d[1] = 0.0000020764;	// Gyro y
		rk.d[2] = 0.0000020764;	// Gyro z
		rk.d[3] = 0.00007615435495;	// Enc x
		rk.d[4] = 0.00007615435495;	// Enc y
		rk.d[5] = 0.00007615435495;	// Enc z
		rk.d[6] = 0.07039*20;	// Accel x
		rk.d[7] = 0.07039*20;	// Accel y
	}
	else {
		// not spinning noise
		qk.d[0] = 0.00001;	// Theta x
		qk.d[1] = 0.00001;	// Theta y
		qk.d[2] = 0.001;	// Theta z
		qk.d[3] = 0.1*1;		// Phi x
		qk.d[4] = 0.1*1;		// Phi y
		qk.d[5] = 0.1*1;		// Theta dot x
		qk.d[6] = 0.1*1;		// Theta dot y
		qk.d[7] = 0.1*20;		// Theta dot z
		qk.d[8] = 0.0001*1;	// Phi dot x
		qk.d[9] = 0.0001*1;	// Phi dot y
		qk.d[10] = 0.001;	// Enc phi aug x
		qk.d[11] = 0.001;	// Enc phi aug y
		qk.d[12] = 0.001;	// Enc phi aug z
		
		rk.d[0] = 0.0000020764;	// Gyro x
		rk.d[1] = 0.0000020764;	// Gyro y
		rk.d[2] = 0.0000020764;	// Gyro z
		rk.d[3] = 0.00007615435495;	// Enc x
		rk.d[4] = 0.00007615435495;	// Enc y
		rk.d[5] = 0.00007615435495;	// Enc z
		rk.d[6] = 0.07039*20;	// Accel x
		rk.d[7] = 0.07039*20;	// Accel y
	}
	
	// Update the noise covariance
	s->Q.d[0][0]  	= s->Q.d[0][0]*LP_Q + (1-LP_Q)*qk.d[0];
	s->Q.d[1][1]  	= s->Q.d[1][1]*LP_Q + (1-LP_Q)*qk.d[1];
	s->Q.d[2][2]  	= s->Q.d[2][2]*LP_Q + (1-LP_Q)*qk.d[2];
	s->Q.d[3][3]  	= s->Q.d[3][3]*LP_Q + (1-LP_Q)*qk.d[3];
	s->Q.d[4][4]  	= s->Q.d[4][4]*LP_Q + (1-LP_Q)*qk.d[4];
	s->Q.d[5][5]  	= s->Q.d[5][5]*LP_Q + (1-LP_Q)*qk.d[5];
	s->Q.d[6][6]  	= s->Q.d[6][6]*LP_Q + (1-LP_Q)*qk.d[6];
	s->Q.d[7][7]  	= s->Q.d[7][7]*LP_Q + (1-LP_Q)*qk.d[7];
	s->Q.d[8][8]  	= s->Q.d[8][8]*LP_Q + (1-LP_Q)*qk.d[8];
	s->Q.d[9][9]	= s->Q.d[9][9]*LP_Q + (1-LP_Q)*qk.d[9];
	s->Q.d[10][10]	= s->Q.d[10][10]*LP_Q + (1-LP_Q)*qk.d[10];
	s->Q.d[11][11]	= s->Q.d[11][11]*LP_Q + (1-LP_Q)*qk.d[11];
	s->Q.d[12][12]	= s->Q.d[12][12]*LP_Q + (1-LP_Q)*qk.d[12];
	
	s->R.d[0][0]  	= s->R.d[0][0]*LP_Q + (1-LP_Q)*rk.d[0];
	s->R.d[1][1]  	= s->R.d[1][1]*LP_Q + (1-LP_Q)*rk.d[1];
	s->R.d[2][2]  	= s->R.d[2][2]*LP_Q + (1-LP_Q)*rk.d[2];
	s->R.d[3][3]  	= s->R.d[3][3]*LP_Q + (1-LP_Q)*rk.d[3];
	s->R.d[4][4]  	= s->R.d[4][4]*LP_Q + (1-LP_Q)*rk.d[4];
	s->R.d[5][5]  	= s->R.d[5][5]*LP_Q + (1-LP_Q)*rk.d[5];
	s->R.d[6][6]  	= s->R.d[6][6]*LP_Q + (1-LP_Q)*rk.d[6];
	s->R.d[7][7]  	= s->R.d[7][7]*LP_Q + (1-LP_Q)*rk.d[7];
	
	rc_vector_free(&qk);
	rc_vector_free(&rk);
	return;
}

/*==============================================================================
	STATE FEEDBACK CONTROLLER MATRIX DATA
==============================================================================*/

void set_controller_gain(rc_matrix_t* K, int spin_flag){
	
	if (spin_flag == 2 ){
		// spinning 0.5 Hz
	
		// x controller
		K->d[0][0] = CONT_K1_S2;
		K->d[0][3] = CONT_K2_S2;
		K->d[0][5] = CONT_K3_S2;
		K->d[0][8] = CONT_K4_S2;
		
		K->d[0][1] = -CONT_K1_S2*CONT_M_K1_S2;
		K->d[0][4] = -CONT_K2_S2*CONT_M_K2_S2;
		K->d[0][6] = -CONT_K3_S2*CONT_M_K3_S2;
		K->d[0][9] = -CONT_K4_S2*CONT_M_K4_S2;

		// y controller
		K->d[1][1] = CONT_K1_S2;
		K->d[1][4] = CONT_K2_S2;
		K->d[1][6] = CONT_K3_S2;
		K->d[1][9] = CONT_K4_S2;
		
		K->d[1][0] = CONT_K1_S2*CONT_M_K1_S2;
		K->d[1][3] = CONT_K2_S2*CONT_M_K2_S2;
		K->d[1][5] = CONT_K3_S2*CONT_M_K3_S2;
		K->d[1][8] = CONT_K4_S2*CONT_M_K4_S2;
		
	}
	else if (spin_flag == 3){
		// x controller
		K->d[0][0] = CONT_K1_S3;
		K->d[0][3] = CONT_K2_S3;
		K->d[0][5] = CONT_K3_S3;
		K->d[0][8] = CONT_K4_S3;
		
		K->d[0][1] = -CONT_K1_S3*CONT_M_K1_S3;
		K->d[0][4] = -CONT_K2_S3*CONT_M_K2_S3;
		K->d[0][6] = -CONT_K3_S3*CONT_M_K3_S3;
		K->d[0][9] = -CONT_K4_S3*CONT_M_K4_S3;

		// y controller
		K->d[1][1] = CONT_K1_S3;
		K->d[1][4] = CONT_K2_S3;
		K->d[1][6] = CONT_K3_S3;
		K->d[1][9] = CONT_K4_S3;
		
		K->d[1][0] = CONT_K1_S3*CONT_M_K1_S3;
		K->d[1][3] = CONT_K2_S3*CONT_M_K2_S3;
		K->d[1][5] = CONT_K3_S3*CONT_M_K3_S3;
		K->d[1][8] = CONT_K4_S3*CONT_M_K4_S3;
	}
	else if (spin_flag == 4){
		// x controller
		K->d[0][0] = CONT_K1_S4;
		K->d[0][3] = CONT_K2_S4;
		K->d[0][5] = CONT_K3_S4;
		K->d[0][8] = CONT_K4_S4;
		
		K->d[0][1] = -CONT_K1_S4*CONT_M_K1_S4;
		K->d[0][4] = -CONT_K2_S4*CONT_M_K2_S4;
		K->d[0][6] = -CONT_K3_S4*CONT_M_K3_S4;
		K->d[0][9] = -CONT_K4_S4*CONT_M_K4_S4;

		// y controller
		K->d[1][1] = CONT_K1_S4;
		K->d[1][4] = CONT_K2_S4;
		K->d[1][6] = CONT_K3_S4;
		K->d[1][9] = CONT_K4_S4;
		
		K->d[1][0] = CONT_K1_S4*CONT_M_K1_S4;
		K->d[1][3] = CONT_K2_S4*CONT_M_K2_S4;
		K->d[1][5] = CONT_K3_S4*CONT_M_K3_S4;
		K->d[1][8] = CONT_K4_S4*CONT_M_K4_S4;
		
	}
	else if (spin_flag == 5){
		// x controller
		K->d[0][0] = CONT_K1_S5;
		K->d[0][3] = CONT_K2_S5;
		K->d[0][5] = CONT_K3_S5;
		K->d[0][8] = CONT_K4_S5;
		
		K->d[0][1] = -CONT_K1_S5*CONT_M_K1_S5;
		K->d[0][4] = -CONT_K2_S5*CONT_M_K2_S5;
		K->d[0][6] = -CONT_K3_S5*CONT_M_K3_S5;
		K->d[0][9] = -CONT_K4_S5*CONT_M_K4_S5;

		// y controller
		K->d[1][1] = CONT_K1_S5;
		K->d[1][4] = CONT_K2_S5;
		K->d[1][6] = CONT_K3_S5;
		K->d[1][9] = CONT_K4_S5;
		
		K->d[1][0] = CONT_K1_S5*CONT_M_K1_S5;
		K->d[1][3] = CONT_K2_S5*CONT_M_K2_S5;
		K->d[1][5] = CONT_K3_S5*CONT_M_K3_S5;
		K->d[1][8] = CONT_K4_S5*CONT_M_K4_S5;
		
	}
	else {
		// not spinning or spin slowly
		
		// x controller
		K->d[0][0] = CONT_K1;
		K->d[0][3] = CONT_K2;
		K->d[0][5] = CONT_K3;
		K->d[0][8] = CONT_K4;
		
		K->d[0][1] = 0;
		K->d[0][4] = 0;
		K->d[0][6] = 0;
		K->d[0][9] = 0;

		// y controller
		K->d[1][1] = CONT_K1;
		K->d[1][4] = CONT_K2;
		K->d[1][6] = CONT_K3;
		K->d[1][9] = CONT_K4;
		
		K->d[1][0] = 0;
		K->d[1][3] = 0;
		K->d[1][5] = 0;
		K->d[1][8] = 0;
	}
	
	// z controller
	K->d[2][2] = CONT_KZ1;
	K->d[2][7] = CONT_KZ2;	
	
	return;
}


/*==============================================================================
	TRANFORMATION MATRIX
==============================================================================*/
void init_transformation_matrix(rc_matrix_t* T_b2w, rc_matrix_t* T_w2b){
	/*==========================================================================
		Transformation matrix between wheel and body axis and vice versa.
		- T_p = transformation matrix from wheel to body axis for phi.
		- T_u = transformation matrix from body to wheel axis for motor command.
	==========================================================================*/
	
	// Initialize temporary matrix
	rc_matrix_zeros(T_b2w, 3, 3);
	rc_matrix_zeros(T_w2b, 3, 3);
	
	rc_matrix_t Rz = rc_matrix_empty();
	rc_matrix_zeros(&Rz, 3, 3);
	
	rc_matrix_t Rx = rc_matrix_empty();
	rc_matrix_zeros(&Rx, 3, 3);
	
	// Yaw correction
	double a = 0.01745329252*T_MATRIX_YAW;
	Rz.d[0][0] = cos(a);
	Rz.d[0][1] = sin(a);
	Rz.d[1][0] = -sin(a);
	Rz.d[1][1] = cos(a);
	Rz.d[2][2] = 1;	
	
	double b = 0.01745329252*180;
	Rx.d[0][0] = 1;
	Rx.d[1][1] = cos(b);
	Rx.d[1][2] = sin(b);
	Rx.d[2][1] = -sin(b);
	Rx.d[2][2] = cos(b);	
	
	// Transformation matrix values from wheel to body axis (from Matlab)
	T_w2b->d[0][0]=-0.6613524024;
	T_w2b->d[0][1]=-0.08395780794;
	T_w2b->d[0][2]=0.7453102104;
	T_w2b->d[1][0]=-0.4787781136;
	T_w2b->d[1][1]=0.8121370382;
	T_w2b->d[1][2]=-0.3333589246;
	T_w2b->d[2][0]=-0.5773945945;
	T_w2b->d[2][1]=-0.5773945945;
	T_w2b->d[2][2]=-0.5773945945;
	
	// Rotate in z for yaw correction
	rc_matrix_left_multiply_inplace(Rz,T_w2b);
	rc_matrix_left_multiply_inplace(Rx,T_w2b);
	
	// Adjust body polarity
	// T_w2b->d[0][0] *= T_MATRIX_SIGN_X;
	// T_w2b->d[0][1] *= T_MATRIX_SIGN_X;
	// T_w2b->d[0][2] *= T_MATRIX_SIGN_X;
	
	// T_w2b->d[1][0] *= T_MATRIX_SIGN_Y;
	// T_w2b->d[1][1] *= T_MATRIX_SIGN_Y;
	// T_w2b->d[1][2] *= T_MATRIX_SIGN_Y;
	
	// T_w2b->d[2][0] *= T_MATRIX_SIGN_Z;
	// T_w2b->d[2][1] *= T_MATRIX_SIGN_Z;
	// T_w2b->d[2][2] *= T_MATRIX_SIGN_Z;
	
	// Transformation matrix values from body to wheel axis
	rc_matrix_transpose(*T_w2b, T_b2w);
	
	// Console printout
	printf("Transformation matrices:\n");
	printf("Wheel to body:\n");
	rc_matrix_print(*T_w2b);
	printf("\nBody to wheel:\n");
	rc_matrix_print(*T_b2w);
	printf("\n");
	
	// Scaling constant between the ball and the wheel
	// rc_matrix_times_scalar(&T_p, R_WHEEL/R_BALL);	// Encoder to ball phi
	// rc_matrix_times_scalar(&T_u, R_WHEEL/R_BALL);	// Ball u to wheel u
	
	// Copy the matrix into the structs
	// rc_matrix_duplicate(T_u, &s->T_b2w);
	// rc_matrix_duplicate(T_p, &s->T_w2b);
	
	// Free memory
	rc_matrix_free(&Rz);
	rc_matrix_free(&Rx);
	
	return;
}

void init_tap_filter_matrix(rc_matrix_t* A, int n, int p){
	
	rc_matrix_zeros(A, p+1, n+1);
	
	// all p=1
	
	// n = 5
	if (n==5){
		A->d[0][0]=0.5238095238;
		A->d[0][1]=0.380952381;
		A->d[0][2]=0.2380952381;
		A->d[0][3]=0.09523809524;
		A->d[0][4]=-0.04761904762;
		A->d[0][5]=-0.1904761905;
		A->d[1][0]=28.57142857;
		A->d[1][1]=17.14285714;
		A->d[1][2]=5.714285714;
		A->d[1][3]=-5.714285714;
		A->d[1][4]=-17.14285714;
		A->d[1][5]=-28.57142857;
	}
	else if (n == 10){
	// n = 10
		A->d[0][0]=0.3181818182;
		A->d[0][1]=0.2727272727;
		A->d[0][2]=0.2272727273;
		A->d[0][3]=0.1818181818;
		A->d[0][4]=0.1363636364;
		A->d[0][5]=0.09090909091;
		A->d[0][6]=0.04545454545;
		A->d[0][7]=-5.551115123e-17;
		A->d[0][8]=-0.04545454545;
		A->d[0][9]=-0.09090909091;
		A->d[0][10]=-0.1363636364;
		A->d[1][0]=9.090909091;
		A->d[1][1]=7.272727273;
		A->d[1][2]=5.454545455;
		A->d[1][3]=3.636363636;
		A->d[1][4]=1.818181818;
		A->d[1][5]=4.33680869e-19;
		A->d[1][6]=-1.818181818;
		A->d[1][7]=-3.636363636;
		A->d[1][8]=-5.454545455;
		A->d[1][9]=-7.272727273;
		A->d[1][10]=-9.090909091;
	}
	else if (n == 20){
		
	
	
	
	// // n = 20
		A->d[0][0]=0.1774891775;
		A->d[0][1]=0.1645021645;
		A->d[0][2]=0.1515151515;
		A->d[0][3]=0.1385281385;
		A->d[0][4]=0.1255411255;
		A->d[0][5]=0.1125541126;
		A->d[0][6]=0.09956709957;
		A->d[0][7]=0.08658008658;
		A->d[0][8]=0.07359307359;
		A->d[0][9]=0.06060606061;
		A->d[0][10]=0.04761904762;
		A->d[0][11]=0.03463203463;
		A->d[0][12]=0.02164502165;
		A->d[0][13]=0.008658008658;
		A->d[0][14]=-0.004329004329;
		A->d[0][15]=-0.01731601732;
		A->d[0][16]=-0.0303030303;
		A->d[0][17]=-0.04329004329;
		A->d[0][18]=-0.05627705628;
		A->d[0][19]=-0.06926406926;
		A->d[0][20]=-0.08225108225;
		A->d[1][0]=2.597402597;
		A->d[1][1]=2.337662338;
		A->d[1][2]=2.077922078;
		A->d[1][3]=1.818181818;
		A->d[1][4]=1.558441558;
		A->d[1][5]=1.298701299;
		A->d[1][6]=1.038961039;
		A->d[1][7]=0.7792207792;
		A->d[1][8]=0.5194805195;
		A->d[1][9]=0.2597402597;
		A->d[1][10]=4.33680869e-19;
		A->d[1][11]=-0.2597402597;
		A->d[1][12]=-0.5194805195;
		A->d[1][13]=-0.7792207792;
		A->d[1][14]=-1.038961039;
		A->d[1][15]=-1.298701299;
		A->d[1][16]=-1.558441558;
		A->d[1][17]=-1.818181818;
		A->d[1][18]=-2.077922078;
		A->d[1][19]=-2.337662338;
		A->d[1][20]=-2.597402597;
	}
	
	
	
	return;
}

void input_transformation_matrix(rc_matrix_t* Ru, double x1, double x2){
	rc_matrix_zeros(Ru, 3, 3);
	
	// Ru->d[0][0]=1.0;
	// Ru->d[0][2]=-2.38825828*x2;
	// Ru->d[1][1]=1.0;
	// Ru->d[1][2]=2.38825828*x1;
	// Ru->d[2][0]=0.03549531838*x2;
	// Ru->d[2][1]=-0.6156672144*x1;
	// Ru->d[2][2]=1.0;
	
	Ru->d[0][0]=1.0;
	Ru->d[0][2]=-2.31591971*x2;
	Ru->d[1][1]=1.0;
	Ru->d[1][2]=2.31591971*x1;
	Ru->d[2][0]=0.03420422588*x2;
	Ru->d[2][1]=-0.6426073769*x1;
	Ru->d[2][2]=1.0;
	
	return;
}

void init_phi_transformation(rc_matrix_t* T){
	T->d[0][0]=1.004765523;
	T->d[0][1]=-0.061576272;
	T->d[0][2]=-0.03416060504;
	T->d[1][0]=-0.061576272;
	T->d[1][1]=1.005416008;
	T->d[1][2]=0.04265273692;
	T->d[2][0]=-0.03416060504;
	T->d[2][1]=0.04265273692;
	T->d[2][2]=1.002803766;
	
	
	return;
}


/*==============================================================================
	SYSTEM DYNAMIC MODEL / NUMERICAL MODEL
==============================================================================*/

void mbbr_model_3d(mbbr_states* s){
	/*==========================================================================
		Calculates the state space model:
			x[k+1] = f(x[k],u[k])
			F[k] = Jacobian of f(x[k],u[k]) w.r.t. x[k]
		
		For planar MIP dynamic, Nx = 4, Nu = 1
			x = [theta, phi, theta_dot, phi_dot]
			u = u_x
		
		Updates f and F
	==========================================================================*/
	
	// System Dynamic Model ----------------------------------------------------
	
	// States and input variables
	double x1 = s->x.d[0];
	double x2 = s->x.d[1];
	double x3 = s->x.d[2];
	double x4 = s->x.d[3];
	double x5 = s->x.d[4];
	double x6 = s->x.d[5];
	double x7 = s->x.d[6];
	double x8 = s->x.d[7];
	double x9 = s->x.d[8];
	double x10 = s->x.d[9];
	double x11 = s->x.d[10];
	double x12 = s->x.d[11];
	double x13 = s->x.d[12];	

	double ux = s->u.d[0];
	double uy = s->u.d[1];
	double uz = s->u.d[2];
	
	// Coulomb friction model
	double sx = friction_comp_exp(x9 - x6 + x2*x8, 10, COULOMB_FC);
	double sy = friction_comp_exp(x10 - x7 - x1*x8, 10, COULOMB_FC);
	double sz = friction_comp_exp(-x8 - x1*x10 + x2*x9, 10, COULOMB_FC_Z);
	
	if (-x8 - x1*x10 + x2*x9 > 0) {
		sz = sz*FRIC_Z_MULT_RIGHT;
	}
	
	double mu_b = VISCOUS_FC;
	
	// The system nonlinear model x[k+1] = f(x[k],u[k]), Explicit Euler
	s->f.d[0]=x1 + 0.005*x6;
	s->f.d[1]=x2 + 0.005*x7;
	s->f.d[2]=x3 + 0.005*x8;
	s->f.d[3]=x4 + 0.005*x9;
	s->f.d[4]=x5 + 0.005*x10;
	s->f.d[5]= - 0.6633964067*ux + 0.6471969301*x1 + 0.9595669974*x6 + 0.04043300262*x9 - 3.773289978*mu_b*x6 + 3.773289978*mu_b*x9 + 8.738636634*sz*x2 - 1.536372814*uz*x2 - 0.0532065851*x2*x8 + 0.009003048491*x7*x8 + 0.00001437611675*x8*x10 + 0.004003048491*x1*x8*x8 - 4.965346655*mu_b*x2*x8;
	s->f.d[6]= - 0.6633964067*uy + 0.6471969301*x2 + 0.9595669974*x7 + 0.04043300262*x10 - 3.773289978*mu_b*x7 + 3.773289978*mu_b*x10 - 8.738636634*sz*x1 + 1.536372814*uz*x1 + 0.0532065851*x1*x8 - 0.009003048491*x6*x8 - 0.00001437611675*x8*x9 + 0.004003048491*x2*x8*x8 + 4.965346655*mu_b*x1*x8;
	s->f.d[7]= - 1.090389498*uz + x8 + 1827231.899*x2*(0.00000002041116331*ux - 0.0000001160953499*sx + 4.046902542e-11*x8*x10 + x9*(0.000003278086127*mu_b + 0.00000003512660456)) + 1827231.899*x1*(0.000002181126055*sy - 0.0000003834720353*uy + 3.260132305e-11*x8*x9 - 1.0*x10*(0.000001213055421*mu_b + 0.00000001299859627)) - 0.00007440601529*x6*x10 + 0.00007440601529*x7*x9 - 6.201956667*x8*(mu_b + 0.01071558318);
	s->f.d[8]=2.50624419*ux - 14.25510598*sx - 1.390756987*x1 + 0.1527517739*x6 + 0.8472482261*x9 + 14.25510598*mu_b*x6 - 14.25510598*mu_b*x9 - 11.50629289*sz*x2 + 2.022964945*uz*x2 - 0.02945513535*x2*x8 + 0.002216085375*x7*x8 - 0.00003089273741*x8*x10 + 0.002216085375*x1*x8*x8 - 2.748813094*mu_b*x2*x8;
	s->f.d[9]=2.50624419*uy - 14.25510598*sy - 1.390756987*x2 + 0.1527517739*x7 + 0.8472482261*x10 + 14.25510598*mu_b*x7 - 14.25510598*mu_b*x10 + 11.50629289*sz*x1 - 2.022964945*uz*x1 + 0.02945513535*x1*x8 - 0.002216085375*x6*x8 + 0.00003089273741*x8*x9 + 0.002216085375*x2*x8*x8 + 2.748813094*mu_b*x1*x8;
	
	s->f.d[10]=x11 + 0.005*x2*x8;
	s->f.d[11]=x12 - 0.005*x1*x8;
	s->f.d[12]=x13 - 0.005*x1*x10;
	s->F.d[10][1]=0.005*x8;
	s->F.d[10][7]=0.005*x2;
	s->F.d[10][10]=1.0;
	s->F.d[11][0]=-0.005*x8;
	s->F.d[11][7]=-0.005*x1;
	s->F.d[11][11]=1.0;
	s->F.d[12][0]=-0.005*x10;
	s->F.d[12][1]=0.005*x9;
	s->F.d[12][8]=0.005*x2;
	s->F.d[12][9]=-0.005*x1;
	s->F.d[12][12]=1.0;
	

	
	
	// The Jacobian of the f(x[k],u[k]) model
	s->F.d[0][0]=1.0;
	s->F.d[0][5]=0.005;
	s->F.d[1][1]=1.0;
	s->F.d[1][6]=0.005;
	s->F.d[2][2]=1.0;
	s->F.d[2][7]=0.005;
	s->F.d[3][3]=1.0;
	s->F.d[3][8]=0.005;
	s->F.d[4][4]=1.0;
	s->F.d[4][9]=0.005;
	s->F.d[5][0]=0.004003048491*x8*x8 + 0.6471969301;
	s->F.d[5][1]=8.738636634*sz - 1.536372814*uz - 1827231.899*x8*(0.000002717414608*mu_b + 0.00000002911868226);
	s->F.d[5][5]=0.9595669974 - 3.773289978*mu_b;
	s->F.d[5][6]=0.009003048491*x8;
	s->F.d[5][7]=0.009003048491*x7 + 0.00001437611675*x10 + 0.008006096982*x1*x8 - 1827231.899*x2*(0.000002717414608*mu_b + 0.00000002911868226);
	s->F.d[5][8]=3.773289978*mu_b + 0.04043300262;
	s->F.d[5][9]=0.00001437611675*x8;
	s->F.d[6][0]=1.536372814*uz - 8.738636634*sz + 1827231.899*x8*(0.000002717414608*mu_b + 0.00000002911868226);
	s->F.d[6][1]=0.004003048491*x8*x8 + 0.6471969301;
	s->F.d[6][5]=-0.009003048491*x8;
	s->F.d[6][6]=0.9595669974 - 3.773289978*mu_b;
	s->F.d[6][7]=0.008006096982*x2*x8 - 0.00001437611675*x9 - 0.009003048491*x6 + 1827231.899*x1*(0.000002717414608*mu_b + 0.00000002911868226);
	s->F.d[6][8]=-0.00001437611675*x8;
	s->F.d[6][9]=3.773289978*mu_b + 0.04043300262;
	s->F.d[7][0]=3.985423105*sy - 0.7006923354*uy + 0.00005957017745*x8*x9 - 1827231.899*x10*(0.000001213055421*mu_b + 0.00000001299859627);
	s->F.d[7][1]=0.0372959287*ux - 0.2121331267*sx + 0.00007394629419*x8*x10 + 1827231.899*x9*(0.000003278086127*mu_b + 0.00000003512660456);
	s->F.d[7][5]=-0.00007440601529*x10;
	s->F.d[7][6]=0.00007440601529*x9;
	s->F.d[7][7]=0.00005957017745*x1*x9 - 6.201956667*mu_b + 0.00007394629419*x2*x10 + 0.9335424175;
	s->F.d[7][8]=0.00007440601529*x7 + 0.00005957017745*x1*x8 + 1827231.899*x2*(0.000003278086127*mu_b + 0.00000003512660456);
	s->F.d[7][9]=0.00007394629419*x2*x8 - 0.00007440601529*x6 - 1827231.899*x1*(0.000001213055421*mu_b + 0.00000001299859627);
	s->F.d[8][0]=0.002216085375*x8*x8 - 1.390756987;
	s->F.d[8][1]=2.022964945*uz - 11.50629289*sz - 1827231.899*x8*(0.000001504359186*mu_b + 0.00000001612008599);
	s->F.d[8][5]=14.25510598*mu_b + 0.1527517739;
	s->F.d[8][6]=0.002216085375*x8;
	s->F.d[8][7]=0.002216085375*x7 - 0.00003089273741*x10 + 0.004432170751*x1*x8 - 1827231.899*x2*(0.000001504359186*mu_b + 0.00000001612008599);
	s->F.d[8][8]=0.8472482261 - 14.25510598*mu_b;
	s->F.d[8][9]=-0.00003089273741*x8;
	s->F.d[9][0]=11.50629289*sz - 2.022964945*uz + 1827231.899*x8*(0.000001504359186*mu_b + 0.00000001612008599);
	s->F.d[9][1]=0.002216085375*x8*x8 - 1.390756987;
	s->F.d[9][5]=-0.002216085375*x8;
	s->F.d[9][6]=14.25510598*mu_b + 0.1527517739;
	s->F.d[9][7]=0.00003089273741*x9 - 0.002216085375*x6 + 0.004432170751*x2*x8 + 1827231.899*x1*(0.000001504359186*mu_b + 0.00000001612008599);
	s->F.d[9][8]=0.00003089273741*x8;
	s->F.d[9][9]=0.8472482261 - 14.25510598*mu_b;
	
	
	
	
	
	
	
	
	
	
	
	// // s->f.d[12]=x13;
	// // s->F.d[12][12]=1.0;
	
	// // The system nonlinear model x[k+1] = f(x[k],u[k]), Explicit Euler
// s->f.d[0]=x1 + 0.005*x6;
// s->f.d[1]=x2 + 0.005*x7;
// s->f.d[2]=x3 + 0.005*x8;
// s->f.d[3]=x4 + 0.005*x9;
// s->f.d[4]=x5 + 0.005*x10;
// s->f.d[5]=0.6471969301*x1 - 0.6633964067*ux + 0.9595669974*x6 + 0.04043300262*x9 - 3.773289978*mu_b*x6 + 3.773289978*mu_b*x9 - 1.536372814*uz*x2 - 0.0532065851*x2*x8 + 0.009003048491*x7*x8 + 0.00001437611675*x8*x10 + 0.004003048491*x1*x8*x8 - 4.965346655*mu_b*x2*x8;
// s->f.d[6]=0.6471969301*x2 - 0.6633964067*uy + 0.9595669974*x7 + 0.04043300262*x10 - 3.773289978*mu_b*x7 + 3.773289978*mu_b*x10 + 1.536372814*uz*x1 + 0.0532065851*x1*x8 - 0.009003048491*x6*x8 - 0.00001437611675*x8*x9 + 0.004003048491*x2*x8*x8 + 4.965346655*mu_b*x1*x8;
// s->f.d[7]=x8 - 1.090389498*uz - 0.00007440601529*x6*x10 + 0.00007440601529*x7*x9 + 1827231.899*x2*(0.00000002041116331*ux + 4.046902542e-11*x8*x10 + x9*(0.000003278086127*mu_b + 0.00000003512660456)) - 1827231.899*x1*(0.0000003834720353*uy - 3.260132305e-11*x8*x9 + x10*(0.000001213055421*mu_b + 0.00000001299859627)) - 6.201956667*x8*(mu_b + 0.01071558318);
// s->f.d[8]=2.50624419*ux - 1.390756987*x1 + 0.1527517739*x6 + 0.8472482261*x9 + 14.25510598*mu_b*x6 - 14.25510598*mu_b*x9 + 2.022964945*uz*x2 - 0.02945513535*x2*x8 + 0.002216085375*x7*x8 - 0.00003089273741*x8*x10 + 0.002216085375*x1*x8*x8 - 2.748813094*mu_b*x2*x8;
// s->f.d[9]=2.50624419*uy - 1.390756987*x2 + 0.1527517739*x7 + 0.8472482261*x10 + 14.25510598*mu_b*x7 - 14.25510598*mu_b*x10 - 2.022964945*uz*x1 + 0.02945513535*x1*x8 - 0.002216085375*x6*x8 + 0.00003089273741*x8*x9 + 0.002216085375*x2*x8*x8 + 2.748813094*mu_b*x1*x8;
// s->f.d[10]=0.005*x9 - 0.005*x6 + x11 + 0.005*x2*x8;
// s->f.d[11]=0.005*x10 - 0.005*x7 + x12 - 0.005*x1*x8;
// s->f.d[12]=x13 - 0.005*x8 - 0.005*x1*x10 + 0.005*x2*x9;
// // The Jacobian of the f(x[k],u[k]) model
// s->F.d[0][0]=1.0;
// s->F.d[0][5]=0.005;
// s->F.d[1][1]=1.0;
// s->F.d[1][6]=0.005;
// s->F.d[2][2]=1.0;
// s->F.d[2][7]=0.005;
// s->F.d[3][3]=1.0;
// s->F.d[3][8]=0.005;
// s->F.d[4][4]=1.0;
// s->F.d[4][9]=0.005;
// s->F.d[5][0]=0.004003048491*x8*x8 + 0.6471969301;
// s->F.d[5][1]=- 1.536372814*uz - 1827231.899*x8*(0.000002717414608*mu_b + 0.00000002911868226);
// s->F.d[5][5]=0.9595669974 - 3.773289978*mu_b;
// s->F.d[5][6]=0.009003048491*x8;
// s->F.d[5][7]=0.009003048491*x7 + 0.00001437611675*x10 + 0.008006096982*x1*x8 - 1827231.899*x2*(0.000002717414608*mu_b + 0.00000002911868226);
// s->F.d[5][8]=3.773289978*mu_b + 0.04043300262;
// s->F.d[5][9]=0.00001437611675*x8;
// s->F.d[6][0]=1.536372814*uz + 1827231.899*x8*(0.000002717414608*mu_b + 0.00000002911868226);
// s->F.d[6][1]=0.004003048491*x8*x8 + 0.6471969301;
// s->F.d[6][5]=-0.009003048491*x8;
// s->F.d[6][6]=0.9595669974 - 3.773289978*mu_b;
// s->F.d[6][7]=0.008006096982*x2*x8 - 0.00001437611675*x9 - 0.009003048491*x6 + 1827231.899*x1*(0.000002717414608*mu_b + 0.00000002911868226);
// s->F.d[6][8]=-0.00001437611675*x8;
// s->F.d[6][9]=3.773289978*mu_b + 0.04043300262;
// s->F.d[7][0]=0.00005957017745*x8*x9 - 0.7006923354*uy - 1827231.899*x10*(0.000001213055421*mu_b + 0.00000001299859627);
// s->F.d[7][1]=0.0372959287*ux + 0.00007394629419*x8*x10 + 1827231.899*x9*(0.000003278086127*mu_b + 0.00000003512660456);
// s->F.d[7][5]=-0.00007440601529*x10;
// s->F.d[7][6]=0.00007440601529*x9;
// s->F.d[7][7]=0.00005957017745*x1*x9 - 6.201956667*mu_b + 0.00007394629419*x2*x10 + 0.9335424175;
// s->F.d[7][8]=0.00007440601529*x7 + 0.00005957017745*x1*x8 + 1827231.899*x2*(0.000003278086127*mu_b + 0.00000003512660456);
// s->F.d[7][9]=0.00007394629419*x2*x8 - 0.00007440601529*x6 - 1827231.899*x1*(0.000001213055421*mu_b + 0.00000001299859627);
// s->F.d[8][0]=0.002216085375*x8*x8 - 1.390756987;
// s->F.d[8][1]=2.022964945*uz - 1827231.899*x8*(0.000001504359186*mu_b + 0.00000001612008599);
// s->F.d[8][5]=14.25510598*mu_b + 0.1527517739;
// s->F.d[8][6]=0.002216085375*x8;
// s->F.d[8][7]=0.002216085375*x7 - 0.00003089273741*x10 + 0.004432170751*x1*x8 - 1827231.899*x2*(0.000001504359186*mu_b + 0.00000001612008599);
// s->F.d[8][8]=0.8472482261 - 14.25510598*mu_b;
// s->F.d[8][9]=-0.00003089273741*x8;
// s->F.d[9][0]=1827231.899*x8*(0.000001504359186*mu_b + 0.00000001612008599) - 2.022964945*uz;
// s->F.d[9][1]=0.002216085375*x8*x8 - 1.390756987;
// s->F.d[9][5]=-0.002216085375*x8;
// s->F.d[9][6]=14.25510598*mu_b + 0.1527517739;
// s->F.d[9][7]=0.00003089273741*x9 - 0.002216085375*x6 + 0.004432170751*x2*x8 + 1827231.899*x1*(0.000001504359186*mu_b + 0.00000001612008599);
// s->F.d[9][8]=0.00003089273741*x8;
// s->F.d[9][9]=0.8472482261 - 14.25510598*mu_b;
// s->F.d[10][1]=0.005*x8;
// s->F.d[10][5]=-0.005;
// s->F.d[10][7]=0.005*x2;
// s->F.d[10][8]=0.005;
// s->F.d[10][10]=1.0;
// s->F.d[11][0]=-0.005*x8;
// s->F.d[11][6]=-0.005;
// s->F.d[11][7]=-0.005*x1;
// s->F.d[11][9]=0.005;
// s->F.d[11][11]=1.0;
// s->F.d[12][0]=-0.005*x10;
// s->F.d[12][1]=0.005*x9;
// s->F.d[12][7]=-0.005;
// s->F.d[12][8]=0.005*x2;
// s->F.d[12][9]=-0.005*x1;
// s->F.d[12][12]=1.0;

	
	// // Gyro bias
	// double x14 = s->x.d[13];
	// double x15 = s->x.d[14];
	// double x16 = s->x.d[15];
	
	// s->f.d[13]=x14;
	// s->f.d[14]=x15;
	// s->f.d[15]=x16;
	
	// s->F.d[13][13]=1;
	// s->F.d[14][14]=1;
	// s->F.d[15][15]=1;
	
	
	// Measurement Dynamic Model -----------------------------------------------
	
	// Kalman prediction states
	x1 = s->f.d[0];
	x2 = s->f.d[1];
	x3 = s->f.d[2];
	x4 = s->f.d[3];
	x5 = s->f.d[4];
	x6 = s->f.d[5];
	x7 = s->f.d[6];
	x8 = s->f.d[7];
	x9 = s->f.d[8];
	x10 = s->f.d[9];
	x11 = s->f.d[10];
	x12 = s->f.d[11];
	x13 = s->f.d[12];
	
	
	sx = friction_comp_exp(x9 - x6 + x2*x8, 10, COULOMB_FC);
	sy = friction_comp_exp(x10 - x7 - x1*x8, 10, COULOMB_FC);
	sz = friction_comp_exp(-x8 - x1*x10 + x2*x9, 10, COULOMB_FC_Z);
	
	if (-x8 - x1*x10 + x2*x9 > 0) {
		sz = sz*FRIC_Z_MULT_RIGHT;
	}
	
	
	// The system nonlinear input y(k) = h(x[k])
	s->h.d[0]=x6 - 1.0*x2*x8;
	s->h.d[1]=x7 + x1*x8;
	s->h.d[2]=x8;
	s->h.d[3]=x4 - 1.0*x1 + x11;
	s->h.d[4]=x5 - 1.0*x2 + x12;
	
	// The Jacobian of h(x[k])
	s->H.d[0][1]=-1.0*x8;
	s->H.d[0][5]=1.0;
	s->H.d[0][7]=-1.0*x2;
	s->H.d[1][0]=x8;
	s->H.d[1][6]=1.0;
	s->H.d[1][7]=x1;
	s->H.d[2][7]=1.0;
	s->H.d[3][0]=-1.0;
	s->H.d[3][3]=1.0;
	s->H.d[3][10]=1.0;
	s->H.d[4][1]=-1.0;
	s->H.d[4][4]=1.0;
	s->H.d[4][11]=1.0;
	
	s->h.d[5]=x13 - 1.0*x3;
	s->H.d[5][2]=-1.0;
	s->H.d[5][12]=1.0;
	
	// // s->h.d[5]=x13;
	// // s->H.d[5][12]=1.0;
	
	
	// x14 = s->f.d[13];
	// x15 = s->f.d[14];
	// x16 = s->f.d[15];
	
	// s->h.d[0] += x14;
	// s->h.d[1] += x15;
	// s->h.d[2] += x16;
	
	// s->H.d[0][13]=1;
	// s->H.d[1][14]=1;
	// s->H.d[2][15]=1;
	
	// x14 = s->f.d[13];
	// x15 = s->f.d[14];
	// x1 += x14;
	// x2 += x15;
	
	// s->H.d[6][13]=17.68020433*sz - 3.108423707*uz + 0.2885230416*x8 + 26.92555662*mu_b*x8;
	// s->H.d[6][14]=0.02170730792*x8*x8 - 4.578244766;
	// s->H.d[7][13]=4.578244766 - 0.02170730792*x8*x8;
	// s->H.d[7][14]=17.68020433*sz - 3.108423707*uz + 0.2885230416*x8 + 26.92555662*mu_b*x8;
	
	
	
	s->h.d[6]=7.842307827*uy - 44.60576095*sy - 4.578244766*x2 + 0.4779767417*x7 - 0.4779767417*x10 + 44.60576095*mu_b*x7 - 44.60576095*mu_b*x10 + 17.68020433*sz*x1 - 3.108423707*uz*x1 + 0.2885230416*x1*x8 - 0.02170730792*x6*x8 + 0.0003198266835*x8*x9 + 0.02170730792*x2*x8*x8 + 26.92555662*mu_b*x1*x8;
	s->h.d[7]=44.60576095*sx - 7.842307827*ux + 4.578244766*x1 - 0.4779767417*x6 + 0.4779767417*x9 - 44.60576095*mu_b*x6 + 44.60576095*mu_b*x9 + 17.68020433*sz*x2 - 3.108423707*uz*x2 + 0.2885230416*x2*x8 - 0.02170730792*x7*x8 + 0.0003198266835*x8*x10 - 0.02170730792*x1*x8*x8 + 26.92555662*mu_b*x2*x8;
	
	s->H.d[6][0]=17.68020433*sz - 3.108423707*uz + 0.2885230416*x8 + 26.92555662*mu_b*x8;
	s->H.d[6][1]=0.02170730792*x8*x8 - 4.578244766;
	s->H.d[6][5]=-0.02170730792*x8;
	s->H.d[6][6]=44.60576095*mu_b + 0.4779767417;
	s->H.d[6][7]=0.2885230416*x1 - 0.02170730792*x6 + 0.0003198266835*x9 + 26.92555662*mu_b*x1 + 0.04341461585*x2*x8;
	s->H.d[6][8]=0.0003198266835*x8;
	s->H.d[6][9]=- 44.60576095*mu_b - 0.4779767417;
	s->H.d[7][0]=4.578244766 - 0.02170730792*x8*x8;
	s->H.d[7][1]=17.68020433*sz - 3.108423707*uz + 0.2885230416*x8 + 26.92555662*mu_b*x8;
	s->H.d[7][5]=- 44.60576095*mu_b - 0.4779767417;
	s->H.d[7][6]=-0.02170730792*x8;
	s->H.d[7][7]=0.2885230416*x2 - 0.02170730792*x7 + 0.0003198266835*x10 + 26.92555662*mu_b*x2 - 0.04341461585*x1*x8;
	s->H.d[7][8]=44.60576095*mu_b + 0.4779767417;
	s->H.d[7][9]=0.0003198266835*x8;
	
	
	
	
	// // The system nonlinear input y(k) = h(x[k])
	// s->h.d[0]=x6 - 1.0*x2*x8;
	// s->h.d[1]=x7 + x1*x8;
	// s->h.d[2]=x8;
	// s->h.d[3]=x11;
	// s->h.d[4]=x12;
	// s->h.d[5]=x13;
	// s->h.d[6]=7.842307827*uy - 4.578244766*x2 + 0.4779767417*x7 - 0.4779767417*x10 + 44.60576095*mu_b*x7 - 44.60576095*mu_b*x10 - 3.108423707*uz*x1 + 0.2885230416*x1*x8 - 0.02170730792*x6*x8 + 0.0003198266835*x8*x9 + 0.02170730792*x2*x8*x8 + 26.92555662*mu_b*x1*x8;
	// s->h.d[7]=4.578244766*x1 - 7.842307827*ux - 0.4779767417*x6 + 0.4779767417*x9 - 44.60576095*mu_b*x6 + 44.60576095*mu_b*x9 - 3.108423707*uz*x2 + 0.2885230416*x2*x8 - 0.02170730792*x7*x8 + 0.0003198266835*x8*x10 - 0.02170730792*x1*x8*x8 + 26.92555662*mu_b*x2*x8;
	// // The Jacobian of h(x[k])
	// s->H.d[0][1]=-1.0*x8;
	// s->H.d[0][5]=1.0;
	// s->H.d[0][7]=-1.0*x2;
	// s->H.d[1][0]=x8;
	// s->H.d[1][6]=1.0;
	// s->H.d[1][7]=x1;
	// s->H.d[2][7]=1.0;
	// s->H.d[3][10]=1.0;
	// s->H.d[4][11]=1.0;
	// s->H.d[5][12]=1.0;
	// s->H.d[6][0]=0.2885230416*x8 - 3.108423707*uz + 26.92555662*mu_b*x8;
	// s->H.d[6][1]=0.02170730792*x8*x8 - 4.578244766;
	// s->H.d[6][5]=-0.02170730792*x8;
	// s->H.d[6][6]=44.60576095*mu_b + 0.4779767417;
	// s->H.d[6][7]=0.2885230416*x1 - 0.02170730792*x6 + 0.0003198266835*x9 + 26.92555662*mu_b*x1 + 0.04341461585*x2*x8;
	// s->H.d[6][8]=0.0003198266835*x8;
	// s->H.d[6][9]=- 44.60576095*mu_b - 0.4779767417;
	// s->H.d[7][0]=4.578244766 - 0.02170730792*x8*x8;
	// s->H.d[7][1]=0.2885230416*x8 - 3.108423707*uz + 26.92555662*mu_b*x8;
	// s->H.d[7][5]=- 44.60576095*mu_b - 0.4779767417;
	// s->H.d[7][6]=-0.02170730792*x8;
	// s->H.d[7][7]=0.2885230416*x2 - 0.02170730792*x7 + 0.0003198266835*x10 + 26.92555662*mu_b*x2 - 0.04341461585*x1*x8;
	// s->H.d[7][8]=44.60576095*mu_b + 0.4779767417;
	// s->H.d[7][9]=0.0003198266835*x8;
	

	return;
}

/*==============================================================================
	MEASUREMENT DYNAMIC MODEL / NUMERICAL MODEL
==============================================================================*/

void mbbr_model_lin(mbbr_states* s){
	/*==========================================================================
		Calculates the state space model:
			y[k] = h(x[k],u[k])
			H = Jacobian of h(x[k],u[k]) w.r.t. x[k]
		
		Calculates h and H for MBBR. Measurements are:
			gyro xyz
			encoder xy
			magnetometer z
			accel xyz
	==========================================================================*/

	// States and input variables
	double x1 = s->x.d[0];
	double x2 = s->x.d[1];
	double x3 = s->x.d[2];
	double x4 = s->x.d[3];
	double x5 = s->x.d[4];
	double x6 = s->x.d[5];
	double x7 = s->x.d[6];
	double x8 = s->x.d[7];
	
	double ux = s->u.d[0];
	double uy = s->u.d[1];
		
	// The system nonlinear model x[k+1] = f(x[k],u[k]), Explicit Euler
	s->f.d[0]=x1 + 0.005*x5;
	s->f.d[1]=x2 + 0.005*x6;
	s->f.d[2]=x3 + 0.005*x7;
	s->f.d[3]=x4 + 0.005*x8;
	s->f.d[4]=0.6471969301*x1 - 0.6633964067*ux + 0.9595669974*x5 + 0.04043300262*x7;
	s->f.d[5]=0.6471969301*x2 - 0.6633964067*uy + 0.9595669974*x6 + 0.04043300262*x8;
	s->f.d[6]=2.50624419*ux - 1.390756987*x1 + 0.1527517739*x5 + 0.8472482261*x7;
	s->f.d[7]=2.50624419*uy - 1.390756987*x2 + 0.1527517739*x6 + 0.8472482261*x8;
	
	// The Jacobian of the f(x[k],u[k]) model
	s->F.d[0][0]=1.0;
	s->F.d[0][4]=0.005;
	s->F.d[1][1]=1.0;
	s->F.d[1][5]=0.005;
	s->F.d[2][2]=1.0;
	s->F.d[2][6]=0.005;
	s->F.d[3][3]=1.0;
	s->F.d[3][7]=0.005;
	s->F.d[4][0]=0.6471969301;
	s->F.d[4][4]=0.9595669974;
	s->F.d[4][6]=0.04043300262;
	s->F.d[5][1]=0.6471969301;
	s->F.d[5][5]=0.9595669974;
	s->F.d[5][7]=0.04043300262;
	s->F.d[6][0]=-1.390756987;
	s->F.d[6][4]=0.1527517739;
	s->F.d[6][6]=0.8472482261;
	s->F.d[7][1]=-1.390756987;
	s->F.d[7][5]=0.1527517739;
	s->F.d[7][7]=0.8472482261;


	// Measurement dynamic model -----------------------------------------------
	
	// Kalman prediction states
	x1 = s->f.d[0];
	x2 = s->f.d[1];
	x3 = s->f.d[2];
	x4 = s->f.d[3];
	x5 = s->f.d[4];
	x6 = s->f.d[5];
	x7 = s->f.d[6];
	x8 = s->f.d[7];
	
	// The system nonlinear input y(k) = h(x[k])
	s->h.d[0]=x5;
	s->h.d[1]=x6;
	s->h.d[2]=x3 - 1.0*x1;
	s->h.d[3]=x4 - 1.0*x2;
	s->h.d[4]=7.842307827*uy - 4.578244766*x2 + 0.4779767417*x6 - 0.4779767417*x8;
	s->h.d[5]=4.578244766*x1 - 7.842307827*ux - 0.4779767417*x5 + 0.4779767417*x7;
	
	// The Jacobian of h(x[k])
	s->H.d[0][4]=1.0;
	s->H.d[1][5]=1.0;
	s->H.d[2][0]=-1.0;
	s->H.d[2][2]=1.0;
	s->H.d[3][1]=-1.0;
	s->H.d[3][3]=1.0;
	s->H.d[4][1]=-4.578244766;
	s->H.d[4][5]=0.4779767417;
	s->H.d[4][7]=-0.4779767417;
	s->H.d[5][0]=4.578244766;
	s->H.d[5][4]=-0.4779767417;
	s->H.d[5][6]=0.4779767417;

	return;
}

