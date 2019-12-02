/* =============================================================================
	Beaglebone Black MBBR Extended Kalman Filter
	----------------------------------------------------------------------------
	Source file for the ball balancing robot using Extended Kalman Filter
	using Beaglebone Black and Robotics Cape.

	Eric Nauli Sihite
	Spring/Summer 2018
============================================================================= */

#include "mbbr_ekf.h"


/* =============================================================================
	THREAD FUNCTIONS
============================================================================= */

void mbbr_imu_interrupt_loop(){
	/* -------------------------------------------------------------------------
		IMU interrupt loop:
			update measurement
			update EKF
			update controller
			update motor driver command
			record data
	------------------------------------------------------------------------- */
	
	// Time stamp and sensor update --------------------------------------------
	
	// Time stamp update (millisecond)
	clock_gettime(CLOCK_MONOTONIC, &t1);
	t_ms = (t1.tv_sec - t0.tv_sec)*1000 + (t1.tv_nsec - t0.tv_nsec)/1000000.0;

	// Sensor measurement update
	update_measurements();
	
	// Check for a tipover
	if (theta_xy_mag(uy.theta) > TIP_ANGLE*0.01745 && motor_state == ARMED){
		disarm_controller();
	}
	
	// Kalman Filter Update ----------------------------------------------------
	
	if (!DISABLE_MOTORS && motor_state == ARMED){
		// Update the predicted f and h. 
		// Can edit these functions to also update Q and R
		mbbr_model_3d(&s);
		mbbr_model_lin(&sl);
		
		// EKF updates
		if (!DISABLE_EKF){
			// allocate the updated Q and R matrices here if it's changed
			// rc_kalman_alloc_ekf(&ekf, s.Q, s.R, s.Pi);
			
			// EKF update
			rc_kalman_update_ekf(&ekf, s.F, s.H, s.f, s.y, s.h);
		}		
		
		// KF update, constant Q and R
		if (!DISABLE_LIN_KF){
			// rc_kalman_alloc_ekf(&ekfl, sl.Q, sl.R, sl.Pi);
			rc_kalman_update_ekf(&ekfl, sl.F, sl.H, sl.f, sl.y, sl.h);
		}
		
		// Copy the result to my own state struct
		rc_vector_duplicate(ekf.x_est, &s.x);	// s.x = ekf.x_est;
		rc_vector_duplicate(ekfl.x_est, &sl.x);	// sl.x = ekfl.x_est;
	}
	
	// Controller and motor command update -------------------------------------
	
	update_controller();
	
	// Drive the motors
	if (!DISABLE_MOTORS && motor_state == ARMED){
		rc_motor_set(MOTOR_CH_1, MTR_SGN_1*uy.uw_dc.d[0]);
		rc_motor_set(MOTOR_CH_2, MTR_SGN_2*uy.uw_dc.d[1]);
		rc_motor_set(MOTOR_CH_3, MTR_SGN_3*uy.uw_dc.d[2]);
	}
	else {
		rc_motor_set(MOTOR_CH_1, 0);
		rc_motor_set(MOTOR_CH_2, 0);
		rc_motor_set(MOTOR_CH_3, 0);
	}
	
	// Record data -------------------------------------------------------------
	
	if (rec_count < data_row && start_fprintf){
		// Controller is running and enough data to record	
		
		if (n_skip < SKIP_REC_STEP){
			// Skip a measurement
			n_skip++;
		}
		else {
			// Time stamp and DMP measurements
			*(data + rec_count*data_col +  0) = t_ms;
			*(data + rec_count*data_col +  1) = uy.theta.d[0];	// DMP x
			*(data + rec_count*data_col +  2) = uy.theta.d[1];	// DMP y
			*(data + rec_count*data_col +  3) = uy.theta.d[2];	// DMP z
			
			// Other data
			*(data + rec_count*data_col +  4) = uy.uw.d[0];	// Controller w1
			*(data + rec_count*data_col +  5) = uy.uw.d[1];	// Controller w2
			*(data + rec_count*data_col +  6) = uy.uw.d[2];	// Controller w3
			
			// Raw measurements
			*(data + rec_count*data_col +  7) = s.y.d[0];	// Gyro x
			*(data + rec_count*data_col +  8) = s.y.d[1];	// Gyro y
			*(data + rec_count*data_col +  9) = s.y.d[2];	// Gyro z
			*(data + rec_count*data_col + 10) = s.y.d[3];	// Phi x
			*(data + rec_count*data_col + 11) = s.y.d[4];	// Phi y
			*(data + rec_count*data_col + 12) = s.y.d[5];	// Phi z
			*(data + rec_count*data_col + 13) = s.y.d[6];	// Accel x
			*(data + rec_count*data_col + 14) = s.y.d[7];	// Accel y
			
			// 3D EKF sensor
			*(data + rec_count*data_col + 15) = s.h.d[0];
			*(data + rec_count*data_col + 16) = s.h.d[1];
			*(data + rec_count*data_col + 17) = s.h.d[2];	// x8
			*(data + rec_count*data_col + 18) = s.h.d[3];	// x11
			*(data + rec_count*data_col + 19) = s.h.d[4];	// x12
			*(data + rec_count*data_col + 20) = s.h.d[5];	// x13
			*(data + rec_count*data_col + 21) = s.h.d[6];
			*(data + rec_count*data_col + 22) = s.h.d[7];
			
			// xc
			*(data + rec_count*data_col + 23) = uy.xc.d[0];	// Theta x
			*(data + rec_count*data_col + 24) = uy.xc.d[1];	// Theta y
			*(data + rec_count*data_col + 25) = uy.xc.d[2];	// Theta z
			*(data + rec_count*data_col + 26) = uy.xc.d[3];	// Phi x
			*(data + rec_count*data_col + 27) = uy.xc.d[4];	// Phi y
			*(data + rec_count*data_col + 28) = uy.xc.d[5];	// Theta d x
			*(data + rec_count*data_col + 29) = uy.xc.d[6];	// Theta d y
			*(data + rec_count*data_col + 30) = uy.xc.d[7];	// Theta d z
			*(data + rec_count*data_col + 31) = uy.xc.d[8];	// Phi d x
			*(data + rec_count*data_col + 32) = uy.xc.d[9];	// Phi d y
			
			// xr
			*(data + rec_count*data_col + 33) = uy.xr.d[0];	// Theta x
			*(data + rec_count*data_col + 34) = uy.xr.d[1];	// Theta x
			*(data + rec_count*data_col + 35) = uy.xr.d[2];	// Theta x
			*(data + rec_count*data_col + 36) = uy.xr.d[3];	// Theta x
			*(data + rec_count*data_col + 37) = uy.xr.d[4];	// Theta x
			*(data + rec_count*data_col + 38) = uy.xr.d[5];	// Theta x
			*(data + rec_count*data_col + 39) = uy.xr.d[6];	// Theta x
			*(data + rec_count*data_col + 40) = uy.xr.d[7];	// Theta x
			*(data + rec_count*data_col + 41) = uy.xr.d[8];	// Theta x
			*(data + rec_count*data_col + 42) = uy.xr.d[9];	// Theta x
			
			// Inertial estimates (speed)
			*(data + rec_count*data_col + 43) = phidot_in_x; 	// phi x in
			*(data + rec_count*data_col + 44) = phidot_in_y; 	// phi y in
			*(data + rec_count*data_col + 45) = 0;	// phi dot x in
			*(data + rec_count*data_col + 46) = 0;	// phi dot y in
			
			// Inertial estimates (pos)
			*(data + rec_count*data_col + 47) = phi_in_x; // phi x
			*(data + rec_count*data_col + 48) = phi_in_y; // phi y
			*(data + rec_count*data_col + 49) = 0; // phi x
			*(data + rec_count*data_col + 50) = 0; // phi y
			
			*(data + rec_count*data_col + 51) = v_battery; // phi d y
			
			if (est_st == KF) *(data + rec_count*data_col + 52) = 0;
			else  *(data + rec_count*data_col + 52) = 1;
			
			*(data + rec_count*data_col + 53) = yaw_0;
			*(data + rec_count*data_col + 54) = spinning_flag;
			
			// EKF estimates
			*(data + rec_count*data_col + 55) = s.x.d[0];
			*(data + rec_count*data_col + 56) = s.x.d[1];
			*(data + rec_count*data_col + 57) = s.x.d[2];
			*(data + rec_count*data_col + 58) = s.x.d[3];
			*(data + rec_count*data_col + 59) = s.x.d[4];
			*(data + rec_count*data_col + 60) = s.x.d[5];
			*(data + rec_count*data_col + 61) = s.x.d[6];
			*(data + rec_count*data_col + 62) = s.x.d[7];
			*(data + rec_count*data_col + 63) = s.x.d[8];
			*(data + rec_count*data_col + 64) = s.x.d[9];
			*(data + rec_count*data_col + 65) = s.x.d[10];
			*(data + rec_count*data_col + 66) = s.x.d[11];
			*(data + rec_count*data_col + 67) = s.x.d[12];
			
			// KF estimates
			*(data + rec_count*data_col + 68) = sl.x.d[0];
			*(data + rec_count*data_col + 69) = sl.x.d[1];
			*(data + rec_count*data_col + 70) = sl.x.d[2];
			*(data + rec_count*data_col + 71) = sl.x.d[3];
			*(data + rec_count*data_col + 72) = sl.x.d[4];
			*(data + rec_count*data_col + 73) = sl.x.d[5];
			*(data + rec_count*data_col + 74) = sl.x.d[6];
			*(data + rec_count*data_col + 75) = sl.x.d[7];
			
			// Extras
			*(data + rec_count*data_col + 76) = uy.u.d[0];
			*(data + rec_count*data_col + 77) = uy.u.d[1];
			*(data + rec_count*data_col + 78) = uy.u.d[2];
			*(data + rec_count*data_col + 79) = rc_x;
			*(data + rec_count*data_col + 80) = rc_y;
			*(data + rec_count*data_col + 81) = rc_z;
			
			// Prediction error covariance
			*(data + rec_count*data_col + 82) = ekf.P.d[0][0];
			*(data + rec_count*data_col + 83) = ekf.P.d[1][1];
			*(data + rec_count*data_col + 84) = ekf.P.d[2][2];
			*(data + rec_count*data_col + 85) = ekf.P.d[3][3];
			*(data + rec_count*data_col + 86) = ekf.P.d[4][4];
			*(data + rec_count*data_col + 87) = ekf.P.d[5][5];
			*(data + rec_count*data_col + 88) = ekf.P.d[6][6];
			*(data + rec_count*data_col + 89) = ekf.P.d[7][7];
			*(data + rec_count*data_col + 90) = ekf.P.d[8][8];
			*(data + rec_count*data_col + 91) = ekf.P.d[9][9];
			*(data + rec_count*data_col + 92) = ekf.P.d[10][10];
			*(data + rec_count*data_col + 93) = ekf.P.d[11][11];
			*(data + rec_count*data_col + 94) = ekf.P.d[12][12];
			
			*(data + rec_count*data_col + 95) = uy.xe.d[8];
			*(data + rec_count*data_col + 96) = uy.xe.d[9];
			*(data + rec_count*data_col + 97) = phide_raw_x;
			*(data + rec_count*data_col + 98) = phide_raw_y;
			
			
			// if (est_st == KF) {
				// *(data + rec_count*data_col + 59) = ekfl.P.d[0][0];
				// *(data + rec_count*data_col + 60) = ekfl.P.d[1][1];
				// *(data + rec_count*data_col + 61) = 0;
				// *(data + rec_count*data_col + 62) = ekfl.P.d[2][2];
				// *(data + rec_count*data_col + 63) = ekfl.P.d[3][3];
				
				// *(data + rec_count*data_col + 64) = ekfl.P.d[4][4];
				// *(data + rec_count*data_col + 65) = ekfl.P.d[5][5];
				// *(data + rec_count*data_col + 66) = 0;
				// *(data + rec_count*data_col + 67) = ekfl.P.d[6][6];
				// *(data + rec_count*data_col + 68) = ekfl.P.d[7][7];
				
				// *(data + rec_count*data_col + 69) = 0;
				// *(data + rec_count*data_col + 70) = 0;
				// *(data + rec_count*data_col + 71) = 0;
			// }
			// else {
				// *(data + rec_count*data_col + 59) = ekf.P.d[0][0];
				// *(data + rec_count*data_col + 60) = ekf.P.d[1][1];
				// *(data + rec_count*data_col + 61) = ekf.P.d[2][2];
				// *(data + rec_count*data_col + 62) = ekf.P.d[3][3];
				// *(data + rec_count*data_col + 63) = ekf.P.d[4][4];
				
				// *(data + rec_count*data_col + 64) = ekf.P.d[5][5];
				// *(data + rec_count*data_col + 65) = ekf.P.d[6][6];
				// *(data + rec_count*data_col + 66) = ekf.P.d[7][7];
				// *(data + rec_count*data_col + 67) = ekf.P.d[8][8];
				// *(data + rec_count*data_col + 68) = ekf.P.d[9][9];
				
				// *(data + rec_count*data_col + 69) = ekf.P.d[10][10];
				// *(data + rec_count*data_col + 70) = ekf.P.d[11][11];
				// *(data + rec_count*data_col + 71) = ekf.P.d[12][12];
				
				// // Phi dot x related
				// *(data + rec_count*data_col + 72) = ekf.P.d[8][0];
				// *(data + rec_count*data_col + 73) = ekf.P.d[8][1];
				// *(data + rec_count*data_col + 74) = ekf.P.d[8][2];
				// *(data + rec_count*data_col + 75) = ekf.P.d[8][3];
				// *(data + rec_count*data_col + 76) = ekf.P.d[8][4];
				// *(data + rec_count*data_col + 77) = ekf.P.d[8][5];
				// *(data + rec_count*data_col + 78) = ekf.P.d[8][6];
				// *(data + rec_count*data_col + 79) = ekf.P.d[8][7];
				// *(data + rec_count*data_col + 80) = ekf.P.d[8][8];
				// *(data + rec_count*data_col + 81) = ekf.P.d[8][9];
				// *(data + rec_count*data_col + 82) = ekf.P.d[8][10];
				// *(data + rec_count*data_col + 83) = ekf.P.d[8][11];
				// *(data + rec_count*data_col + 84) = ekf.P.d[8][12];
			// }			
			
			
			
			
			n_skip = 0;
			rec_count++;
		}
	}
	
	// End of loop routine -----------------------------------------------------
	
	return;
}
//void* setpoint_loop(__attribute__ ((unused)) void* ptr){
void* setpoint_loop(void* ptr){
	/* -------------------------------------------------------------------------
		Setpoint and starting check thread.
	------------------------------------------------------------------------- */
	int checks = 0;
	int checks_needed = round(START_DELAY*SETPOINT_RATE_HZ);
	
	// Wait some time
	//rc_usleep(1000000);
	
	// Wait some time
	int xx = 0;
	while (xx < INIT_IDLE_TIME*SETPOINT_RATE_HZ){
		xx++;		
		rc_usleep(1000000 / SETPOINT_RATE_HZ);
	}
	
	// Loop begins -------------------------------------------------------------
	while(rc_get_state()!=EXITING){
		// Check for starting angle if disabled
		if(motor_state == DISARMED){
			if (theta_xy_mag(uy.theta) < START_ANGLE*0.01745 && start_fprintf){
				checks++;
			} 
			else checks = 0;
			
			if (checks >= checks_needed){
				arm_controller();
				checks = 0;
			}
		}
		else {
			// Motor is armed, update setpoints
			read_bluetooth();
		}
		
		rc_usleep(1000000 / SETPOINT_RATE_HZ);
	}
	
	
	// Exit routine ------------------------------------------------------------
	
	
	return NULL;
}
void* printf_loop(void* ptr){
	/* -------------------------------------------------------------------------
		printf thread.
	------------------------------------------------------------------------- */
	
	rc_usleep(1000000);	// Sleep for 1 seconds	

	// Header
	printf("   t_ms  |");
	printf("  MOTOR  |");
	printf("  DRIVE  |");
	printf("   Vb    |");
	
	printf("   θx    |");
	printf("   θy    |");
	printf("   θz    |");
	
	printf("  enc1   |");
	printf("  enc2   |");
	printf("  enc3   |");
	
	
	// printf("   ax    |");
	// printf("   ay    |");
	// printf("   az    |");
		
	// Raw IMU
	// printf("   ωx    |");
	// printf("   ωy    |");
	// printf("   ωz    |");
	// printf("   ax    |");
	// printf("   ay    |");
	// printf("   az    |");
	
	// Bluetooth
	printf("  bt_x   |");
	printf("  bt_y   |");
	printf("  button |");
	
	// Motor state
	
	
	
	// printf("   φ1    |");
	// printf("   φ2    |");
	// printf("   φ3    |");
	
	// printf("   u1    |");
	// printf("   u2    |");
	// printf("   u3    |");
	
	// printf("   x1    |");
	// printf("   x2    |");
	
	
	printf("  n_rec  |");
	
	printf("\n");

	// Loop begins -------------------------------------------------------------
	
	while(rc_get_state()!=EXITING){
		// Printout stuff
		printf("\r");
		
		// Time and motor state
		printf("%7.3f  |", t_ms/1000.0);
		if (motor_state == ARMED) 	printf("  ARMED  |");
		else 						printf("   OFF   |");
		
		if (driving_state == DRIVING && spinning_state == SPINNING){
			printf("  D & S  |");
		}
		else if (driving_state == DRIVING && spinning_state == NOT_SPINNING){
			printf("  DRIVE  |");
		}
		else if (driving_state == STOP && spinning_state == SPINNING){
			printf("  SPIN   |");
		}
		else	printf("  STOP   |");
		
		
		printf("%7.3f  |", v_battery);
		
		// DMP
		
		
		printf("%7.3f  |", uy.theta.d[0]);
		printf("%7.3f  |", uy.theta.d[1]);
		printf("%7.3f  |", uy.theta.d[2]);
		
		// Encoder
		
		
		
		
		
		// printf("%7.3f  |", uy.phid.d[0]);
		// printf("%7.3f  |", uy.phid.d[1]);
		// printf("%7.3f  |", uy.phid.d[2]);
		
		
		// Motor
		// printf("%7.3f  |", uy.u.d[0]);
		// printf("%7.3f  |", uy.u.d[1]);
		// printf("%7.3f  |", uy.u.d[2]);
		
		// printf("%7.3f  |", uy.phi_w.d[0]);
		// printf("%7.3f  |", uy.phi_w.d[1]);
		// printf("%7.3f  |", uy.phi_w.d[2]);
		// printf("%7.3f  |", uy.uw.d[0]);
		// printf("%7.3f  |", uy.uw.d[1]);
		// printf("%7.3f  |", uy.uw.d[2]);
		
		printf("%i  |", ENC_SGN_1*rc_encoder_eqep_read(ENC_CH_1));
		printf("%i  |", ENC_SGN_2*rc_encoder_eqep_read(ENC_CH_2));
		printf("%i  |", ENC_SGN_3*rc_encoder_eqep_read(ENC_CH_3));
		
		// IMU
		// printf("%7.3f  |", uy.gyro.d[0]);
		// printf("%7.3f  |", uy.gyro.d[1]);
		// printf("%7.3f  |", uy.gyro.d[2]);
		// printf("%7.3f  |", uy.accel.d[0]);
		// printf("%7.3f  |", uy.accel.d[1]);
		// printf("%7.3f  |", uy.accel.d[2]);
		
		// printf("%7.3f  |", bt_x);
		// printf("%7.3f  |", bt_y);
		// printf("%i     |", bt_button);
		
		// printf("%7.3f  |", s.x.d[0]);
		// printf("%7.3f  |", s.x.d[1]);
		// printf("%7.3f  |", s.x.d[2]);
		
		
		printf("   %i     |", rec_count);

		// Flush out text
		fflush(stdout);
		rc_usleep(1000000 / PRINTF_RATE_HZ);
	}
	
	// Exit routine ------------------------------------------------------------
	
	printf("\n");	

	return NULL;
}
void* fprintf_loop(void* ptr){
	/* -------------------------------------------------------------------------
		Print the data into a text file.
	------------------------------------------------------------------------- */
	
	FILE *f = fopen(FILENAME, "w");	// Initialize the fprintf file
	fprintf_not_done = 1;

	// Text header
	fprintf(f, "Ball Balancer Experiment. \n");
	fprintf(f, "Sampling rate: %i\n", SAMPLING_RATE_HZ);
	fprintf(f, "Motor driver PWM frequency: %i\n", PWM_FREQ);
	fprintf(f, "Output data: %i rows and %i columns\n", data_row, data_col);
	fprintf(f, "End time = %7.3f seconds\n", t_end/1000.0);	
	
	// Data recorded:
	fprintf(f,"time (ms), DMP(3), y(8), u(3), h_3d(8), h_lin(6)\n");
	
	// Loop begins -------------------------------------------------------------
	
	int n = 0;
	int m = 0;
	
	
	while(rc_get_state()!=EXITING){
		// Check if there is something to fprintf
		if (n < rec_count){
			// Record new row of data			
			fprintf(f,"%7.6f", data[n*data_col]); // First column
			m = 1;			
			while (m < data_col){
				// The rest of the column
				fprintf(f,",%7.6f", data[n*data_col + m]);	
				m += 1;
			}	
			fprintf(f,"\n");
			n++;
		}
		
		rec_left = rec_count - n;
		
		// Sleep until the next time we need to fprintf out something.
		rc_usleep(1000000 / FPRINTF_RATE_HZ);
	}
	
	// Exit routine ------------------------------------------------------------
	
	// Check if there is still something left to print out
	while (n < rec_count){
		// Record new row of data			
		fprintf(f,"%7.6f", data[n*data_col]); // First column
		m = 1;			
		while (m < data_col){
			// The rest of the column
			fprintf(f,",%7.6f", data[n*data_col + m]);	
			m += 1;
		}	
		fprintf(f,"\n");
		n++;
	}
	
	fprintf_not_done = 0;
	fclose(f);

	return NULL;
}

/* =============================================================================
	I/O FUNCTIONS
============================================================================= */
void update_measurements(){
	/* -------------------------------------------------------------------------
		Update the measurement vectors.
	------------------------------------------------------------------------- */
	
	// DMP Measurements
	uy.theta.d[0] = TX_SGN*(mpu_data.dmp_TaitBryan[TB_PITCH_X]-THETA_OFFSET_X);
	uy.theta.d[1] = TY_SGN*(mpu_data.dmp_TaitBryan[TB_ROLL_Y]-THETA_OFFSET_Y);
	uy.theta.d[2] = TZ_SGN*mpu_data.dmp_TaitBryan[TB_YAW_Z];
	
	// Raw accel measurements
	uy.accel.d[0] = AX_SGN*mpu_data.accel[0]*ACCEL_G_SCALE;
	uy.accel.d[1] = AY_SGN*mpu_data.accel[2]*ACCEL_G_SCALE;
	uy.accel.d[2] = AZ_SGN*mpu_data.accel[1]*ACCEL_G_SCALE;
	
	// Rotate acceleration vectors
	rc_matrix_times_col_vec(uy.R_accel, uy.accel ,&temp_vec_1);	
	rc_vector_duplicate(temp_vec_1, &uy.accel);
	
	// Raw gyro measurements
	uy.gyro.d[0] = WX_SGN*(mpu_data.gyro[0]*DEG_TO_RAD - GYRO_OFFSET_X);
	uy.gyro.d[1] = WY_SGN*(mpu_data.gyro[1]*DEG_TO_RAD - GYRO_OFFSET_Y);
	uy.gyro.d[2] = WZ_SGN*(mpu_data.gyro[2]*DEG_TO_RAD - GYRO_OFFSET_Z);
	
	// Encoder measurements
	rc_vector_duplicate(uy.phi_w, &uy.phi_w_old);	// record old values
	uy.phi_w.d[0] = ENC_SGN_1*rc_encoder_eqep_read(ENC_CH_1)*TWO_PI/ENC_CPR;
	uy.phi_w.d[1] = ENC_SGN_2*rc_encoder_eqep_read(ENC_CH_2)*TWO_PI/ENC_CPR;
	uy.phi_w.d[2] = ENC_SGN_3*rc_encoder_eqep_read(ENC_CH_3)*TWO_PI/ENC_CPR;
	
	// Magnetometer measurements
	uy.magnet.d[0] = mpu_data.mag[0];
	uy.magnet.d[1] = mpu_data.mag[1];
	uy.magnet.d[2] = mpu_data.mag[2];
	
	// Attitude angles from accelerometer
	theta_x_atan2 = atan2(-uy.accel.d[1],-uy.accel.d[2]);
	theta_y_atan2 = atan2(uy.accel.d[0],-uy.accel.d[2]);
	
	// Transform encoder measurements into body axis ball spin angle
	rc_vector_duplicate(uy.phi, &uy.phi_old);	// Record old values
	rc_matrix_times_col_vec(uy.T_w2b, uy.phi_w, &uy.phi);
	rc_vector_times_scalar(&uy.phi, R_WHEEL/R_BALL);
	
	// Ball speed estimates with dirty differentiator
	rc_vector_subtract(uy.phi, uy.phi_old, &temp_vec_1);	// (u[k] - u[k-1])
	rc_vector_times_scalar(&temp_vec_1, GAIN_DDIFF);	// g*(u[k] - u[k-1])
	rc_vector_times_scalar(&uy.phid, WC_DDIFF);		// a*y[k-1]
	rc_vector_sum_inplace(&uy.phid, temp_vec_1);
	
	// Wheels speed estimate using tap filtering
	uy.phid_w.d[0] = tap_filter_diff(uy.A_tap, &uy.w_1, uy.phi_w.d[0]);
	uy.phid_w.d[1] = tap_filter_diff(uy.A_tap, &uy.w_2, uy.phi_w.d[1]);
	uy.phid_w.d[2] = tap_filter_diff(uy.A_tap, &uy.w_3, uy.phi_w.d[2]);	
	
	// Battery voltage measurement
	v_battery = rc_adc_batt();
	
	// Update Kalman Filter struct measurements
	
	// EKF
	s.y.d[0] = uy.gyro.d[0];
	s.y.d[1] = uy.gyro.d[1];
	s.y.d[2] = uy.gyro.d[2];
	s.y.d[3] = uy.phi.d[0];
	s.y.d[4] = uy.phi.d[1];
	s.y.d[5] = uy.phi.d[2];
	s.y.d[6] = uy.accel.d[0];
	s.y.d[7] = uy.accel.d[1];
	
	// KF
	sl.y.d[0] = uy.gyro.d[0];
	sl.y.d[1] = uy.gyro.d[1];
	sl.y.d[2] = uy.phi.d[0];
	sl.y.d[3] = uy.phi.d[1];
	sl.y.d[4] = uy.accel.d[0];
	sl.y.d[5] = uy.accel.d[1];	
	
	return;
}
void update_controller(){
	/* -------------------------------------------------------------------------
		Update the controller state and motor command vectors.
	------------------------------------------------------------------------- */
	
	// Import bluetooth control command ----------------------------------------
	
	// Start / select button to reset yaw
	if (bt_start == 2 || bt_start == 4 || bt_start == 6 ){
		yaw_forward = uy.xc.d[2];	// The new zero yaw
		// uy.xr.d[2] = yaw_forward;	// New yaw zero reference
	}
	
	// Limit the remote control directional commands magnitude to 1
	temp_d_1 = bt_y;
	temp_d_2 = -bt_x;
	temp_d_3 = sqrt(temp_d_1*temp_d_1 + temp_d_2*temp_d_2);
	if (temp_d_3 > 1){
		temp_d_1 = temp_d_1/temp_d_3;	// x driving command rate
		temp_d_2 = temp_d_2/temp_d_3;	// y driving command rate
	}
	
	// Initialize some Bluetooth driving variables
	flag_body_frame_drive = 0;	// 1 = x direction is about the body's forward.
	spin_rate = 0;				// yaw rate
	drive_rate_x = 0;			// drive rate, x direction
	drive_rate_y = 0;			// drive rate, y direction
	spinning_flag_old = spinning_flag;		// record old spinning flag value
		
	// Yaw command from bluetooth
	if (bt_button == 2){
		
		// Circle button
		spin_rate = BT_SPIN_RATE*1.0;
		drive_rate_x = temp_d_1*BT_DRIVE_RATE*0.50;
		drive_rate_y = temp_d_2*BT_DRIVE_RATE*0.50;
		spinning_flag = 3;
		
		
		// // Fast spin
		// if (bt_y > 0.75) {
			// // Spin left
			// spin_rate = BT_SPIN_RATE*1.0;
		// }
		// else if (bt_y < -0.75){
			// // Spin right
			// spin_rate = -BT_SPIN_RATE*1.0;
		// }
		// spinning_flag = 3;
		
		// if (bt_y > 0.75){
			// spin_rate = BT_SPIN_RATE*0.25;
			// drive_rate_x = BT_DRIVE_RATE*1.25;
			// drive_rate_y = 0;
			// flag_body_frame_drive = 1;	
		// }
		// else if (bt_y < -0.75){
			// spin_rate = -BT_SPIN_RATE*0.5;
			// drive_rate_x = BT_DRIVE_RATE*1.25;
			// drive_rate_y = 0;
			// flag_body_frame_drive = 1;	
		// }
		// else {
			// spin_rate = 0;
			// drive_rate_x = 0;
			// drive_rate_y = 0;
			// flag_body_frame_drive = 1;
		// }
		// spinning_flag = 0;
	}
	else if (bt_button == 8){
		// Square button
		
		spin_rate = BT_SPIN_RATE*0.5;
		drive_rate_x = temp_d_1*BT_DRIVE_RATE*0.99;
		drive_rate_y = temp_d_2*BT_DRIVE_RATE*0.99;
		spinning_flag = 1;	
		
		
		// // Slow spin
		// if (bt_y > 0.75) {
			// // Spin left
			// spin_rate = BT_SPIN_RATE*0.5;
		// }
		// else if (bt_y < -0.75){
			// // Spin right
			// spin_rate = -BT_SPIN_RATE*0.5;
		// }
		// spinning_flag = 1;	
		
		
		// if (bt_y > 0.75){
			// spin_rate = BT_SPIN_RATE*0.25;
			// drive_rate_x = BT_DRIVE_RATE*0.75;
			// drive_rate_y = 0;
			// flag_body_frame_drive = 1;	
		// }
		// else if (bt_y < -0.75){
			// spin_rate = -BT_SPIN_RATE*0.25;
			// drive_rate_x = BT_DRIVE_RATE*0.75;
			// drive_rate_y = 0;
			// flag_body_frame_drive = 1;	
		// }
		// else {
			// spin_rate = 0;
			// drive_rate_x = 0;
			// drive_rate_y = 0;
			// flag_body_frame_drive = 1;
		// }
		// spinning_flag = 1;
	}
	else if (bt_button == 1){
		// Triangle button
		
		spin_rate = BT_SPIN_RATE*0.75;
		drive_rate_x = temp_d_1*BT_DRIVE_RATE*0.75;
		drive_rate_y = temp_d_2*BT_DRIVE_RATE*0.75;
		spinning_flag = 2;	
		
		// // Medium spin
		// if (bt_y > 0.75) {
			// // Spin left
			// spin_rate = BT_SPIN_RATE*0.75;
		// }
		// else if (bt_y < -0.75){
			// // Spin right
			// spin_rate = -BT_SPIN_RATE*0.75;
		// }
		// spinning_flag = 2;
		
		// if (bt_y > 0.75){
			// spin_rate = BT_SPIN_RATE*0.25;
			// drive_rate_x = BT_DRIVE_RATE*1.0;
			// drive_rate_y = 0;
			// flag_body_frame_drive = 1;	
		// }
		// else if (bt_y < -0.75){
			// spin_rate = -BT_SPIN_RATE*0.25;
			// drive_rate_x = BT_DRIVE_RATE*1.0;
			// drive_rate_y = 0;
			// flag_body_frame_drive = 1;	
		// }
		// else {
			// spin_rate = 0;
			// drive_rate_x = 0;
			// drive_rate_y = 0;
			// flag_body_frame_drive = 1;
		// }
		// spinning_flag = 1;
	}
	else if (bt_button == 4){
		// X button 
		
		// if (bt_y > 0.75){
			// spin_rate = BT_SPIN_RATE*0.25;
			// drive_rate_x = BT_DRIVE_RATE*1.5;
			// drive_rate_y = 0;
			// flag_body_frame_drive = 1;	
		// }
		// else if (bt_y < -0.75){
			// spin_rate = BT_SPIN_RATE*0.25;
			// drive_rate_x = 0;
			// drive_rate_y = BT_DRIVE_RATE*1.5;
			// flag_body_frame_drive = 1;	
		// }
		// else {
			// spin_rate = 0;
			// drive_rate_x = 0;
			// drive_rate_y = 0;
			// flag_body_frame_drive = 1;
		// }
		// spinning_flag = 0;
		
		
		// // Super fast
		// if (bt_y > 0.75) {
			// // Spin left
			// spin_rate = BT_SPIN_RATE*1.5;
		// }
		// else if (bt_y < -0.75){
			// // Spin right
			// spin_rate = -BT_SPIN_RATE*1.5;
		// }
		// spinning_flag = 4;	
		
		spin_rate = BT_SPIN_RATE*1.5;
		drive_rate_x = temp_d_1*BT_DRIVE_RATE*0.25;
		drive_rate_y = temp_d_2*BT_DRIVE_RATE*0.25;
		spinning_flag = 4;	
	}
	else {
		// Not spinning
		drive_rate_x = temp_d_1*BT_DRIVE_RATE*1.25;
		drive_rate_y = temp_d_2*BT_DRIVE_RATE*1.25;
		spinning_flag = 0;
	}
	
	// Ball speed command from bluetooth	
	rc_x = R_C*rc_x + (1-R_C)*drive_rate_x;		// forward
	rc_y = R_C*rc_y + (1-R_C)*drive_rate_y;		// left
	rc_z = R_CZ*rc_z + (1-R_CZ)*spin_rate;
	
	
	// Determine the controller state xc ---------------------------------------
	
	if (est_st == EKF){

		if (USE_DMP_THETA){
			uy.xc.d[0] = uy.theta.d[0];	// Theta x
			uy.xc.d[1] = uy.theta.d[1];	// Theta y	
		}
		else {
			uy.xc.d[0] = s.x.d[0];
			uy.xc.d[1] = s.x.d[1];
		}
		
		uy.xc.d[2] = s.x.d[2] - offset_yaw;	// Theta z
		uy.xc.d[5] = s.x.d[5];	// Theta dot x
		uy.xc.d[6] = s.x.d[6];	// Theta dot y
		uy.xc.d[7] = s.x.d[7];	// Theta dot z
		
		// Update phi integrators (inertial frame)
		phi_in_x += phidot_in_x*DT_S;
		phi_in_y += phidot_in_y*DT_S;
		
		// Phi dot (inertial frame)
		phidot_in_x = cos(uy.xc.d[2])*s.x.d[8] - sin(uy.xc.d[2])*s.x.d[9];
		phidot_in_y = sin(uy.xc.d[2])*s.x.d[8] + cos(uy.xc.d[2])*s.x.d[9];
		
		uy.xc.d[3] = phi_in_x;	// Inertial phi x (integrated)
		uy.xc.d[4] = phi_in_y;	// Inertial phi y (integrated)
		uy.xc.d[8] = phidot_in_x;	// Inertial phi dot x
		uy.xc.d[9] = phidot_in_y;	// Inertial phi dot y
		
	}
	else if (est_st == KF){
		// Linear KF
	
		if (USE_DMP_THETA){
			uy.xc.d[0] = uy.theta.d[0];	// Theta x
			uy.xc.d[1] = uy.theta.d[1];	// Theta y	
		}
		else {
			uy.xc.d[0] = sl.x.d[0];
			uy.xc.d[1] = sl.x.d[1];
		}
		
		uy.xc.d[2] = uy.theta.d[2] - offset_yaw;
		uy.xc.d[5] = sl.x.d[4];
		uy.xc.d[6] = sl.x.d[5];
		uy.xc.d[7] = uy.gyro.d[2];
		
		// Update phi integrators (inertial frame)
		phi_in_x += phidot_in_x*DT_S;
		phi_in_y += phidot_in_y*DT_S;
		
		// Phi dot (inertial frame)
		phidot_in_x = cos(uy.xc.d[2])*sl.x.d[6] - sin(uy.xc.d[2])*sl.x.d[7];
		phidot_in_y = sin(uy.xc.d[2])*sl.x.d[6] + cos(uy.xc.d[2])*sl.x.d[7];
		
		uy.xc.d[3] = phi_in_x;
		uy.xc.d[4] = phi_in_y;
		uy.xc.d[8] = phidot_in_x;
		uy.xc.d[9] = phidot_in_y;
	}
	else {
		// Assume est_st = RAW
		
		// DMP and raw measurements
		uy.xc.d[0] = uy.theta.d[0];
		uy.xc.d[1] = uy.theta.d[1];
		uy.xc.d[2] = uy.theta.d[2];
		uy.xc.d[3] = uy.phi.d[0];
		uy.xc.d[4] = uy.phi.d[1];
		uy.xc.d[5] = uy.gyro.d[0];
		uy.xc.d[6] = uy.gyro.d[1];
		uy.xc.d[7] = uy.gyro.d[2];
		uy.xc.d[8] = uy.phid.d[0];
		uy.xc.d[9] = uy.phid.d[1];
	}
	
	// Determine the ball reference states and tracking error ------------------
	
	// Ball speed references (inertial phi dot xy)
	if (flag_body_frame_drive){
		// Move about body ref
		uy.xr.d[8] = cos(uy.xc.d[2])*rc_x - sin(uy.xc.d[2])*rc_y;
		uy.xr.d[9] = sin(uy.xc.d[2])*rc_x + cos(uy.xc.d[2])*rc_y;
	}
	else {
		// About inertial ref
		uy.xr.d[8] = rc_x*cos(yaw_forward) - rc_y*sin(yaw_forward);
		uy.xr.d[9] = rc_x*sin(yaw_forward) + rc_y*cos(yaw_forward);		
	}
	
	// Integrate the phi dot reference 
	uy.xr.d[3] += DT_S*uy.xr.d[8];
	uy.xr.d[4] += DT_S*uy.xr.d[9];
	
	// State errors (phi and phidot variables)
	uy.xe.d[3] = uy.xc.d[3] - uy.xr.d[3];	// phi x
	uy.xe.d[4] = uy.xc.d[4] - uy.xr.d[4];	// phi y
	uy.xe.d[8] = uy.xc.d[8] - uy.xr.d[8];	// phid x
	uy.xe.d[9] = uy.xc.d[9] - uy.xr.d[9];	// phid y
	
	// Rotate back inertial frame difference into yaw normalized frame
	temp_d_1 = uy.xe.d[3];
	temp_d_2 = uy.xe.d[4];
	uy.xe.d[3] =  cos(uy.xc.d[2])*temp_d_1 + sin(uy.xc.d[2])*temp_d_2;
	uy.xe.d[4] = -sin(uy.xc.d[2])*temp_d_1 + cos(uy.xc.d[2])*temp_d_2;
	temp_d_1 = uy.xe.d[8];
	temp_d_2 = uy.xe.d[9];
	uy.xe.d[8] =  cos(uy.xc.d[2])*temp_d_1 + sin(uy.xc.d[2])*temp_d_2;
	uy.xe.d[9] = -sin(uy.xc.d[2])*temp_d_1 + cos(uy.xc.d[2])*temp_d_2;	

	// Lowpass filter the phidot error states
	if (spinning_flag < 0 || spinning_flag >= N_GAINSCH){
		// Unexpected flag value, use default (not spinning)
		lp_wc_target = PHIDOT_LP_WC[0];
	}
	else {
		lp_wc_target = PHIDOT_LP_WC[spinning_flag];
	}
	filter_2nd_coef_update(&phid_fx,lp_wc_target);
	filter_2nd_coef_update(&phid_fy,lp_wc_target);
	
	// Record the raw values
	phide_raw_x = uy.xe.d[8]; 						
	phide_raw_y = uy.xe.d[9];
	
	// Controller error states
	uy.xe.d[8] = filter_2nd(&phid_fx,uy.xe.d[8]);
	uy.xe.d[9] = filter_2nd(&phid_fy,uy.xe.d[9]);
	
	// Optional?	
	if (USE_YAWRATE_CORRECTION){
		temp_d_3 = YAWRATE_CORRECTION_COEF*uy.xc.d[7];
		
		temp_d_1 = uy.xe.d[8];
		temp_d_2 = uy.xe.d[9];
		uy.xe.d[8] = cos(temp_d_3)*temp_d_1 - sin(temp_d_3)*temp_d_2;
		uy.xe.d[9] = sin(temp_d_3)*temp_d_1 + cos(temp_d_3)*temp_d_2;	
		
		temp_d_1 = uy.xe.d[3];
		temp_d_2 = uy.xe.d[4];
		uy.xe.d[3] = cos(temp_d_3)*temp_d_1 - sin(temp_d_3)*temp_d_2;
		uy.xe.d[4] = sin(temp_d_3)*temp_d_1 + cos(temp_d_3)*temp_d_2;
	}
	
	// Detect spinning imbalance 
	if (spinning_flag > 1 && flag_spin_imbalance == 0 && 
													USE_SPIN_FAILURE_CHECK) {
		// Check if losing balance
		
		// Phi error and theta xy magnitudes
		temp_d_1 = sqrt(uy.xc.d[0]*uy.xe.d[0] + uy.xc.d[1]*uy.xe.d[1]);
		temp_d_2 = sqrt(uy.xe.d[3]*uy.xe.d[3] + uy.xe.d[4]*uy.xe.d[4]);
		temp_d_3 = sqrt(uy.xe.d[8]*uy.xe.d[8] + uy.xe.d[9]*uy.xe.d[9]);
		
		// Check the thresholds
		if (temp_d_1 > SPIN_FAIL_THETA_THRESHOLD || 
			temp_d_2 > SPIN_FAIL_PHI_THRESHOLD ||
			temp_d_3 > SPIN_FAIL_PHID_THRESHOLD) {
			flag_spin_imbalance = 1;	// Losing balance
		}
	}
	else if (flag_spin_imbalance == 1 && USE_SPIN_FAILURE_CHECK){
		// Check if regain balance
		
		// Phi error and theta xy magnitudes
		temp_d_1 = sqrt(uy.xc.d[0]*uy.xe.d[0] + uy.xc.d[1]*uy.xe.d[1]);
		temp_d_2 = sqrt(uy.xe.d[3]*uy.xe.d[3] + uy.xe.d[4]*uy.xe.d[4]);
		temp_d_3 = sqrt(uy.xe.d[8]*uy.xe.d[8] + uy.xe.d[9]*uy.xe.d[9]);
		
		// The theta and phi error must be smaller than the threshold
		if (temp_d_1 < SPIN_SAFE_THETA_THRESHOLD && 
			temp_d_2 < SPIN_SAFE_PHI_THRESHOLD &&
			temp_d_3 < SPIN_SAFE_PHID_THRESHOLD) {
			flag_spin_imbalance = 0;
		}
	}
	
	// Update yaw references
	if (flag_spin_imbalance == 1) {
		// spin less until regaining balance
		rc_z2 = R_CZ2*rc_z2 + (1-R_CZ2)*spin_rate*0;
		
		// Yaw references
		uy.xr.d[7] = rc_z2;				// yaw rate ref
		uy.xr.d[2] += DT_S*uy.xr.d[7];	// yaw ref
		spinning_flag = 1;				// Use lower speed spinning controller
	}
	else {
		// Spin normally
		rc_z2 = R_CZ2*rc_z2 + (1-R_CZ2)*spin_rate;
		
		// Yaw references
		uy.xr.d[7] = rc_z2;				// yaw rate ref
		uy.xr.d[2] += DT_S*uy.xr.d[7];	// yaw ref
	}
	
	
	// yaw components tracking error
	uy.xe.d[2] = angle_difference(uy.xc.d[2], uy.xr.d[2]);	// yaw
	uy.xe.d[7] = uy.xc.d[7] - uy.xr.d[7];					// yawd
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	// Determine the theta and theta dot ref -----------------------------------
	
	// Determine SLC gain and cutoff frequency
	if (spinning_flag < 0 || spinning_flag >= N_GAINSCH){
		// Unexpected flag value, use default (not spinning)
		slc_target_gain = SLC_GAIN[0];
		slc_target_wc   = SLC_COEF_2[0];
	}
	else {
		// Gain scheduled based on spinning_flag value
		slc_target_gain = SLC_GAIN[spinning_flag];
		slc_target_wc   = SLC_COEF_2[spinning_flag];
	}
	
	slc_spd_gain   = R_SLC*slc_spd_gain   + (1-R_SLC)*slc_target_gain;
	slc_spd_coef_2 = R_SLC*slc_spd_coef_2 + (1-R_SLC)*slc_target_wc;
	
	// Ball speed input SLC outer loop algorithm, outputs theta references
	slc_theta_x = slc_spd_gain*SLC_COEF_1*phiD_e_x_old
							+ slc_spd_coef_2*slc_theta_x_old;
	slc_theta_y = slc_spd_gain*SLC_COEF_1*phiD_e_y_old
							+ slc_spd_coef_2*slc_theta_y_old;
	
	// Backwards difference
	slc_thetaD_x = (slc_theta_x - slc_theta_x_old)/DT_S;
	slc_thetaD_y = (slc_theta_y - slc_theta_y_old)/DT_S;
	
	// Prescaler
	uy.xr.d[0] = slc_theta_x*SLC_PRESCALER;		// theta x ref
	uy.xr.d[1] = slc_theta_y*SLC_PRESCALER;		// theta y ref
	uy.xr.d[5] = slc_thetaD_x*SLC_PRESCALER;	// theta dot x ref
	uy.xr.d[6] = slc_thetaD_y*SLC_PRESCALER;	// theta dot y ref		

	
	// Record old values

	// Old phi components tracking error
	slc_theta_x_old = slc_theta_x;
	slc_theta_y_old = slc_theta_y;
	phiD_e_x_old = phide_raw_x;
	phiD_e_y_old = phide_raw_y;

			
	// Saturate the reference values
	uy.xr.d[0] = saturate_d(uy.xr.d[0], -THETA_REF_CAP, THETA_REF_CAP);
	uy.xr.d[1] = saturate_d(uy.xr.d[1], -THETA_REF_CAP, THETA_REF_CAP);
	uy.xr.d[5] = saturate_d(uy.xr.d[5], -THETAD_REF_CAP, THETAD_REF_CAP);
	uy.xr.d[6] = saturate_d(uy.xr.d[6], -THETAD_REF_CAP, THETAD_REF_CAP);
	
	// Theta tracking error
	uy.xe.d[0] = uy.xc.d[0] - uy.xr.d[0];	// theta x
	uy.xe.d[1] = uy.xc.d[1] - uy.xr.d[1];	// theta y
	uy.xe.d[5] = uy.xc.d[5] - uy.xr.d[5];	// theta dot x
	uy.xe.d[6] = uy.xc.d[6] - uy.xr.d[6];	// theta dot y
	
	// Controller update -------------------------------------------------------
	
	// Determine the controller gains (spin / no spin)
	if (USE_VAR_GAINS) set_controller_gain(&uy.K_target, spinning_flag);
	else set_controller_gain(&uy.K_target, 0);
	
	if (USE_SPIN_PHID_CONTROL_ONLY && spinning_flag > 1){
		// Don't use position controller
		uy.K_target.d[0][3] = 0;
		uy.K_target.d[1][4] = 0;
	}
	
	// Update gain matrix (gain scheduling)
	rc_matrix_times_scalar(&uy.K_target, 1-R_K);	// (1-R_K)*K_t
	rc_matrix_times_scalar(&uy.K, R_K);				// R_K*K_old
	rc_matrix_add_inplace(&uy.K, uy.K_target);		// K_new = R_K*K_old + (1-R_K)*K_t
	
	// Determine the controller command values
	rc_matrix_times_col_vec(uy.K, uy.xe, &uy.u);	// u = K(x - xr)
	
	// EKF input update
	if (est_st == EKF) {
		// Update EKF input vectors
		sl.u.d[0] = uy.u.d[0];
		sl.u.d[1] = uy.u.d[1];
		
		temp_d_1 = s.x.d[8] - s.x.d[5] + s.x.d[1]*s.x.d[7];
		temp_d_2 = s.x.d[9] - s.x.d[6] - s.x.d[0]*s.x.d[7];
		temp_d_3 = -s.x.d[7] - s.x.d[0]*s.x.d[9] + s.x.d[1]*s.x.d[8];
				
		s.u.d[0] = uy.u.d[0];
		s.u.d[1] = uy.u.d[1];
		s.u.d[2] = uy.u.d[2];
		
		// temp_d_4 = sqrt(temp_d_1*temp_d_1 + temp_d_2*temp_d_2);
		uy.u.d[0] += friction_comp_exp(temp_d_1, FRIC_EXP, COULOMB_FCOMP_X);
		uy.u.d[1] += friction_comp_exp(temp_d_2, FRIC_EXP, COULOMB_FCOMP_Y);
		
		if (temp_d_3 < 0){
			uy.u.d[2] += friction_comp_exp(temp_d_3, 
												FRIC_EXP, COULOMB_FCOMP_Z);
		}
		else {
			uy.u.d[2] += friction_comp_exp(temp_d_3, 
						FRIC_EXP, COULOMB_FCOMP_Z*FRIC_Z_MULT_RIGHT);
		}
		
		
	}
	else if (est_st == KF) {
		temp_d_1 = sl.x.d[6] - sl.x.d[4];
		temp_d_2 = sl.x.d[7] - sl.x.d[5];
		temp_d_3 = uy.phid.d[2];
		
		// Update EKF input vectors
		s.u.d[0] = uy.u.d[0];
		s.u.d[1] = uy.u.d[1];
		s.u.d[2] = uy.u.d[2];	
		
		sl.u.d[0] = uy.u.d[0];
		sl.u.d[1] = uy.u.d[1];
		
		uy.u.d[0] += friction_comp_exp(temp_d_1, FRIC_EXP, COULOMB_FCOMP_X);
		uy.u.d[1] += friction_comp_exp(temp_d_2, FRIC_EXP, COULOMB_FCOMP_Y);
		uy.u.d[2] += friction_comp_exp(temp_d_3, FRIC_EXP, COULOMB_FCOMP_Z);
		
	}
	

	
	
	// count_test_1++;
	// if (count_test_1 > 400){
		// count_test_1 = 0;
		// count_test_2++;
	// }
	// if (count_test_2 == 1){
		// uy.u.d[0] = 1.5;
		// uy.u.d[1] = 0;
		// uy.u.d[2] = 0;
	// }
	// else if (count_test_2 == 2){
		// uy.u.d[0] = 0;
		// uy.u.d[1] = 1.5;
		// uy.u.d[2] = 0;
	// }
	// else if (count_test_2 == 3){
		// uy.u.d[0] = 0;
		// uy.u.d[1] = 0;
		// uy.u.d[2] = 1.5;
	// }
	// else if (count_test_2 > 3) count_test_2 = 0;
	

	
	// Transform into wheel axis motor driver commands
	rc_matrix_times_col_vec(uy.T_b2w, uy.u, &uy.uw);
	rc_vector_times_scalar(&uy.uw, R_WHEEL/R_BALL);
	
	// // DEBUG
	
	// count_test_1++;
	// if (count_test_1 > 200){
		// count_test_1 = 0;
		// count_test_2++;
	// }
	// if (count_test_2 > 20){
		// uy.uw.d[0] = 0;
		// uy.uw.d[1] = 0;
		// uy.uw.d[2] = 0;
	// }
	// else {
		// uy.uw.d[0] = -count_test_2*0.05;
		// uy.uw.d[1] = -count_test_2*0.05;
		// uy.uw.d[2] = -count_test_2*0.05;
	// }
	
	// uy.uw.d[0] = 0.3;
	// uy.uw.d[1] = 0.3;
	// uy.uw.d[2] = 0.3;
	
	// Drive-coast compensations -----------------------------------------------
	
	if (USE_DRIVE_COAST_COMPENSATION){
		if (USE_MOTOR_BOOST){
			// // Motor 1
			if (abs_d(uy.uw.d[0]) < 0.65){
				uy.uw_dc.d[0] = drive_coast_comp(uy.uw.d[0]*1.2, uy.phid_w.d[0], 
										v_battery, PWM_MODEL,MTR_MODEL_L);
			}
			else if (abs_d(uy.uw.d[0]) < 0.8){
				uy.uw_dc.d[0] = drive_coast_comp(uy.uw.d[0]*1.1, uy.phid_w.d[0], 
										v_battery, PWM_MODEL,MTR_MODEL_L);
			}
			else  {
				uy.uw_dc.d[0] = drive_coast_comp(uy.uw.d[0]*1.0, uy.phid_w.d[0], 
										v_battery, PWM_MODEL,MTR_MODEL_L);
			}
			
			// Motor 2
			uy.uw_dc.d[1] = drive_coast_comp(uy.uw.d[1], uy.phid_w.d[1], 
										v_battery, PWM_MODEL,MTR_MODEL_L);
										
			// Motor 3
			if (abs_d(uy.uw.d[2]) > 0.4){
				uy.uw_dc.d[2] = drive_coast_comp(uy.uw.d[2]*1.05, uy.phid_w.d[2], 
										v_battery, PWM_MODEL,MTR_MODEL_L);
			}
			else  {
				uy.uw_dc.d[2] = drive_coast_comp(uy.uw.d[2], uy.phid_w.d[2], 
										v_battery, PWM_MODEL,MTR_MODEL_L);
			}
		}
		else {
			uy.uw_dc.d[0] = drive_coast_comp(uy.uw.d[0], uy.phid_w.d[0], 
										v_battery, PWM_MODEL,MTR_MODEL_L);
			uy.uw_dc.d[1] = drive_coast_comp(uy.uw.d[1], uy.phid_w.d[1], 
										v_battery, PWM_MODEL,MTR_MODEL_L);
			uy.uw_dc.d[2] = drive_coast_comp(uy.uw.d[2], uy.phid_w.d[2], 
										v_battery, PWM_MODEL,MTR_MODEL_L);
		}
	}
	else rc_vector_duplicate(uy.uw, &uy.uw_dc);
	
	// uy.uw_dc.d[0] =1 ;
	// uy.uw_dc.d[1] =1 ;
	// uy.uw_dc.d[2] =1 ;
	
	
	return;
}

void disarm_controller(){
	/* -------------------------------------------------------------------------
		Disable the motor and reset the states.
	------------------------------------------------------------------------- */
	
	motor_state = DISARMED;
	reset_variables();
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 1);	
	
	return;
}
void arm_controller(){
	/* -------------------------------------------------------------------------
		Enable the motor and reset the states.
	------------------------------------------------------------------------- */
	
	motor_state = ARMED;
	reset_variables();
	rc_led_set(RC_LED_GREEN, 1);
	rc_led_set(RC_LED_RED, 0);
	
	return;
}


/* =============================================================================
	DRIVE-COAST MOTOR DRIVER FUNCTIONS
============================================================================= */

double drive_coast_comp(double u, double w, double Vb, int model, double L){
	/* -------------------------------------------------------------------------
		Compensate for the drive-coast nonlinearities
	------------------------------------------------------------------------- */
	
	double v, it, is, wr, ws, tau_r, rhs, v_abs;
	double f, df;
	double temp;
	int n = 0;
	
	double us = saturate_d(u*MTR_MODEL_V/Vb,-0.99,0.99);
	double Vbb = MTR_MODEL_V;	// maybe best
	
	// Linear motor model
	wr = saturate_d(w/(Vbb/MTR_MODEL_K), -0.99, 0.99);
	is = Vbb/MTR_MODEL_R;
	//it = (MTR_MODEL_V*u - MTR_MODEL_K*w)/MTR_MODEL_R;	// linear model target
	it = (Vbb*us - MTR_MODEL_K*w)/MTR_MODEL_R;	// linear model target
	
	ws = sign_d(it)*wr;
	tau_r = (MTR_MODEL_R/L)/PWM_FREQ;
	
	if (abs_d(it) < 0.000001){
		// Zero target current
		v = 0;
	}
	else {
		// Nonzero target current
		
		if (model == 0){
			// Low PWM model
			v = it/is/(1 - ws);
		}
		else if (model == 1){
			// High PWM model
			v = (it/is + wr + sign_d(it))/2;
		}
		else if (model == 2){
			// Assume at the linear region
			v = (it/is + wr + sign_d(it))/2;
			v_abs = abs_d(v);
			rhs = ( exp(tau_r)*(1 + ws) + 1 - ws )/2;
			
			if (rhs < 0){
				// the log function is singular
				printf("Error: log function singular.\n");
				return 0;
			}
			
			if (exp(v_abs*tau_r) <= rhs){
				// Newton raphson
				temp = log(rhs)/tau_r;
				v_abs = temp/2;
				n = 0;
				
				while (n < NR_N_MAX){
					coast_model(&f,&df,v_abs,wr,it,tau_r,is);
					
					if (abs_d(f) < NR_TOL) {
						// Converges
						n = NR_N_MAX;
					}
					else {
						// Not yet converges, next iteration
						v_abs = saturate_d(v_abs - f/df, 0.001, temp);
						n++;
					}
				}
				
				v = sign_d(it)*v_abs;
			}
		}
		else {
			// Shouldn't get here
			v = 0;
		}
	}
	
	// Saturate the compensated u just in case
	if (v > 1) v = 1;
	else if (v < -1) v = -1;
	
	return v;
}
void coast_model(double* f, double* df, 
					double v, double wr, double it, double tau_r, double is){
	
	// v is absolute of u
	double u = sign_d(it)*v;
	double ws = sign_d(it)*wr;
	
	double temp2 = exp(v*tau_r);
	double temp = temp2*(1 + ws)/(2*temp2 - 1 + ws);
	
	
	if (temp < 0){
		// log singular
		printf("\nError: log in f singular. ws = %f. exp = %f, x = %f.\n", ws, temp2, temp);
		*f = 0;
		*df = 0.00001;
	}
	
	*f = is*(u*(1 - ws) + (sign_d(it) + wr)/tau_r*log(temp)) - it;
	//*df = 2*tau_r*(exp(v*tau_r) - 1)*(1 - ws)/(2*exp(v*tau_r) + ws - 1);
	*df = 2*is*(temp2 - 1)*(sign_d(it) - wr)/(2*temp2 + ws - 1);
	
	return;
}
// double newton_raphson_drive_coast(double u, double wr, double tau_r){
	
	// double v = 0;

	
	// return v;
// }



double interpolate_mesh(double x, double y, rc_matrix_t A){
	/* -------------------------------------------------------------------------
		2D interpolation from mesh data
		------------------------------------------------------------------------
		The output is the interpolated using bilinear interpolation.
		The input mesh is normalized with x and y range = [0,N_A-1).
		Matrix format is A[row][col], therefore A[y][x].
		Assume A is a square matrix.
	------------------------------------------------------------------------- */
	
	// Initial square block for the interpolation surface 
	int x0 = floor(x);
	int y0 = floor(y);
	
	double dx = x - x0;
	double dy = y - y0;
	
	return ( A.d[y0][x0]*(1-dx)*(1-dy) + A.d[y0][x0+1]*dx*(1-dy)
			 + A.d[y0+1][x0]*(1-dx)*dy + A.d[y0+1][x0+1]*dx*dy );
}

/* =============================================================================
	BLUETOOTH FUNCTIONS
============================================================================= */

void init_bluetooth(){
	/*
	// OLD CODE
	
	// Open in non blocking read/write mode using UART2 channel.
	uart2_filestream = -1;
	uart2_filestream = open("/dev/ttyO2", O_RDWR | O_NOCTTY | O_NDELAY);		
	if (uart2_filestream == -1)	{
			// ERROR - CAN'T OPEN SERIAL PORT.
			printf("Error - Unable to open UART.\n");
		}
	else {
		// Print out the file descriptor.
		printf("File Descriptor: %i\n", uart2_filestream);
	}
	
	// Bluetooth settings
	// baud rates: B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200
	// character size mask: CS5, CS6, CS7, CS8
	struct termios options;
	tcgetattr(uart2_filestream, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;	// Set baud rate
	options.c_iflag = ICRNL;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart2_filestream, TCIFLUSH);
	tcsetattr(uart2_filestream, TCSANOW, &options);
	
	*/
	
	// Load UART bus
	uart_bus = UART5_BUS;
	
	// disable canonical (0), 1 stop bit (1), disable parity (0)
	printf("\nUART bus %d\n\n", uart_bus);
	if(rc_uart_init(uart_bus, BAUDRATE, TIMEOUT_S, 0,1,0)){
		printf("Failed to rc_uart_init%d\n", uart_bus);
		return;
	}
	
	//rc_uart_flush(uart_bus); // Flush?
	
	return;
}
void read_bluetooth(){
	/*
	// OLD
	if (uart2_filestream != -1)	{
		if (read(uart2_filestream, (void*)rx_buffer, 7) > 0){
			// Read successful, record the controller values.
			
			printf("pika\n");
			
			// Using 4Joy (free) joystick app in Android data structure:
			if (rx_buffer[0] == 0 && rx_buffer[4] == 4) {
				// x-direction
				if (rx_buffer[1] > 128)
					bt_x =  -1 + (rx_buffer[1]-129)/127.0;
				else
					bt_x =  rx_buffer[1]/127.0;
				
				// y-direction
				if (rx_buffer[2] > 128)
					bt_y =  1 - (rx_buffer[2]-129)/127.0;
				else
					bt_y =  -rx_buffer[2]/127.0;
				
				// Buttons pressed
				bt_button = rx_buffer[5];
			}
		}
	}
	*/
	
	memset(rx_buffer,0,sizeof(rx_buffer));	
	read_count = rc_uart_read_bytes(uart_bus, rx_buffer, 7);
	
	spinning_state_old = spinning_state;
	
	if (read_count > 0) {
		// Read successful, record the controller values.
		
		// Using 4Joy (free) joystick app in Android data structure:
		if (rx_buffer[0] == 0 && rx_buffer[4] == 4) {
			// Have the correct format			
			
			// x-direction
			if (rx_buffer[1] > 128)
				bt_x =  -1 + (rx_buffer[1]-129)/127.0;
			else
				bt_x =  rx_buffer[1]/127.0;
			
			// y-direction
			if (rx_buffer[2] > 128)
				bt_y =  1 - (rx_buffer[2]-129)/127.0;
			else
				bt_y =  -rx_buffer[2]/127.0;
			
			// Buttons pressed
			bt_button = rx_buffer[5];	// (1,2,4,8)
			bt_start = rx_buffer[6];	// start/select (2,4, both = 6)
			
			if (rx_buffer[1] == 0 && rx_buffer[2] == 0){
				driving_state = STOP; // Just driving or also spinning?
			}
			else {
				driving_state = DRIVING;
			}
			
			if (bt_button == 1 || bt_button == 2 || 
					bt_button == 4 || bt_button == 8) {
				spinning_state = SPINNING;
			}
			else spinning_state = NOT_SPINNING;
		}
		else {
			// Corrupted data
			driving_state = STOP;
			spinning_state = NOT_SPINNING;
		}
	}
	else if (read_count == 0){
		// Timeout
		driving_state = STOP;
		spinning_state = NOT_SPINNING;
	}
	else {
		// Read failed
		bt_x = 0;
		bt_y = 0;
		bt_button = 0;
		
		driving_state = STOP;
		spinning_state = NOT_SPINNING;
	}
	
	// if (spinning_state_old == NOT_SPINNING && spinning_state == SPINNING){
		// // Change flag 
		// spin_start_flag = -spin_start_flag;
	// }
	
	
	return;
}

/* =============================================================================
	PAUSE AND MODE BUTTON FUNCTIONS
============================================================================= */

void on_pause_release(){
	// The only working button right now lol
	
	if (start_fprintf){
		// Disable recording and balancing
		start_fprintf = 0;
		rc_led_set(RC_LED_GREEN, 0);
		rc_led_set(RC_LED_RED, 0);		
		disarm_controller();
		
	}
	else {
		// Enable recording and balancing
		start_fprintf = 1;
		rc_led_set(RC_LED_GREEN, 1);
		rc_led_set(RC_LED_RED, 1);
	}
	
	
	
	
	return;
}
void on_mode_release(){
	return;
}

/*==============================================================================
	MATH FUNCTIONS
==============================================================================*/
double angle_difference(double x1, double x2){
	// Determine angle difference between x1 and x2 within [-PI, PI].
	
	double diff = x1 - x2;
	if (diff > PI) {
		while (diff > PI){
			diff -= 2*PI;
		}
	}			
	else if (diff < -PI){
		while (diff < -PI){
			diff += 2*PI;
		}
	}
	
	return diff;
}
double abs_d(double x){
	// Return the absolute value for x (|x|)
	
	if (x < 0)	return -x;
	else 		return x;
}
double sign_d(double x){
	// Return the sign of x
	
	if (x > 0)			return 1;
	else if (x < 0)		return -1;
	else				return 0;
}
double saturate_d(double x, double min, double max){
	// Saturate x with the bounds [min,max]
	
	if (x > max)		return max;
	else if (x < min)	return min;
	else				return x;
}
double theta_xy_mag(rc_vector_t theta){
	// Calculate the vector length of [x;y]
	
	return sqrt(theta.d[0]*theta.d[0] + theta.d[1]*theta.d[1]);
}

double sign_d_friction(double x, double r){
	// Return the sign of x if |x| > r, else 0
	
	if (x > r)			return 1;
	else if (x < -r)	return -1;
	else				return 0;
}

double friction_comp_exp(double x, double a, double c){
	// Use exponent function to compensate friction
	// x = speed
	// a = exponent rate
	// c = coulomb friction coefficient
	// c and a must be positive
	
	if (x > 0)			return (1 - exp(-a*x))*c;
	else if (x < 0)		return (-1 + exp(a*x))*c;
	else				return 0;
	
}

double tap_filter_diff(rc_matrix_t A, rc_vector_t* f, double fk){
	
	// push back older values of f and add the new fk
	for (int k = N_TAP; k > 0; k--){
		f->d[k] = f->d[k-1];
	}
	f->d[0] = fk;
	
	// Calculate the A*fk
	rc_vector_t temp = rc_vector_empty();
	rc_matrix_times_col_vec(A, *f ,&temp);
	
	double output = temp.d[1];
	
	rc_vector_free(&temp);
	
	return output;
}

double filter_2nd(lp2_filter* lp, double u){
	// 2nd order low pass filter
	
	// Push back older values
	lp->yk.d[2] = lp->yk.d[1];
	lp->yk.d[1] = lp->yk.d[0];
	lp->uk.d[2] = lp->uk.d[1];
	lp->uk.d[1] = lp->uk.d[0];
	
	// New values
	lp->uk.d[0] = u;
	lp->yk.d[0] = lp->num0*lp->uk.d[0] + lp->num1*lp->uk.d[1] 
		+ lp->num2*lp->uk.d[2] - lp->den1*lp->yk.d[1] - lp->den2*lp->yk.d[2];
	
	return lp->yk.d[0];	
}

void filter_2nd_coef_update(lp2_filter* lp, double wc){
	// 1st order filter only
	
	lp->num1 = 1+wc;
	lp->den1 = wc;
	
	return;
}


void init_filter_2nd(lp2_filter* lp, 
			double n0, double n1, double n2, double d1, double d2){
	
	lp->yk = rc_vector_empty();
	lp->uk = rc_vector_empty();
	rc_vector_zeros(&lp->yk, 3);
	rc_vector_zeros(&lp->uk, 3);
	
	lp->num0 = n0;
	lp->num1 = n1;
	lp->num2 = n2;
	lp->den1 = d1;
	lp->den2 = d2;
	
	return;
}

void reset_filter_2nd(lp2_filter* lp){
	rc_vector_zero_out(&lp->yk);
	rc_vector_zero_out(&lp->uk);
	
	return;
}

void exit_filter_2nd(lp2_filter* lp){
	rc_vector_free(&lp->yk);
	rc_vector_free(&lp->uk);
	
	return;
}


/*==============================================================================
	INTERPOLATION FUNCTIONS
==============================================================================*/


// int interpolate_controller_gain(float wz, float uz, float wz_max, float uz_max){
	// // Generate the 2x10 controller gain matrix (K_lqr) for the ball balancer (extended state for the integration control)
	
	// // Normalize the parameters (N must be odd number)
	// int mid = floor(N_INT/2.0);			// 2 in case of N_INT = 5
	// float xn = wz/wz_max*mid + mid;		// Yaw speed parameter, [-2,2) + 2 -> [0,4)
	// float yn = uz/uz_max*mid + mid; 	// Yaw torque parameter, same as above
	
	// // Saturate xn and yn => e.g. min(max(xn,0),N-1.00001);
	// if (xn > N_INT-1.00001) xn = N_INT-1.00001;	if (xn < 0) xn = 0;
	// if (yn > N_INT-1.00001) yn = N_INT-1.00001;	if (yn < 0) yn = 0;
	
	// // Interpolate the data for all the gains
	// int i,j;
	// for (i=0; i<2; i++){
		// for (j=0; j<10; j++){
			// rc_set_matrix_entry(&K_lqr,i,j,interpolate_mesh(xn, yn, Ki[i][j]));
			
		// }
	// }
	
	// return 0;
// }




/*==============================================================================
	INITIATION, RESET AND EXIT ROUTINES
==============================================================================*/
void init_mbbr_states(mbbr_states* s, int Nx, int Ny, int Nu, double Pi){
	
	// Initialize the vectors
	s->x = rc_vector_empty();
	s->f = rc_vector_empty();
	s->y = rc_vector_empty();
	s->h = rc_vector_empty();
	s->u = rc_vector_empty();
	
	rc_vector_zeros(&s->x, Nx);
	rc_vector_zeros(&s->f, Nx);
	rc_vector_zeros(&s->y, Ny);
	rc_vector_zeros(&s->h, Ny);
	rc_vector_zeros(&s->u, Nu);
	
	// Initialize the matrices
	s->F = rc_matrix_empty();
	s->H = rc_matrix_empty();
	s->Q = rc_matrix_empty();
	s->R = rc_matrix_empty();
	s->Pi = rc_matrix_empty();
	
	rc_matrix_zeros(&s->F, Nx, Nx);
	rc_matrix_zeros(&s->H, Ny, Nx);
	rc_matrix_zeros(&s->Q, Nx, Nx);
	rc_matrix_zeros(&s->R, Ny, Ny);
	
	rc_matrix_zeros(&s->Pi, Nx, Nx);
	rc_matrix_identity(&s->Pi, Nx);
	rc_matrix_times_scalar(&s->Pi, Pi);
	
	// Initialize the scalars
	s->Nx = Nx;
	s->Ny = Ny;
	s->Nu = Nu;	
	
	return;
}

void init_input_output_data(io_data* uy){
	// Initialize the io struct
	
	// Initialize the vectors
	uy->theta = rc_vector_empty();
	uy->gyro = rc_vector_empty();
	uy->accel = rc_vector_empty();
	uy->magnet = rc_vector_empty();
	
	uy->phi = rc_vector_empty();
	uy->phi_old = rc_vector_empty();
	uy->phid = rc_vector_empty();
	uy->phi_w = rc_vector_empty();
	uy->phi_w_old = rc_vector_empty();
	uy->phid_w = rc_vector_empty();
	
	uy->xc = rc_vector_empty();
	uy->xr = rc_vector_empty();
	uy->u = rc_vector_empty();
	uy->uw = rc_vector_empty();
	uy->uw_dc = rc_vector_empty();
	
	// Setup vector length
	rc_vector_zeros(&uy->theta, 3);
	rc_vector_zeros(&uy->gyro, 3);
	rc_vector_zeros(&uy->accel, 3);
	rc_vector_zeros(&uy->magnet, 3);
	
	rc_vector_zeros(&uy->phi, 3);
	rc_vector_zeros(&uy->phi_old, 3);
	rc_vector_zeros(&uy->phid, 3);
	rc_vector_zeros(&uy->phi_w, 3);
	rc_vector_zeros(&uy->phi_w_old, 3);
	rc_vector_zeros(&uy->phid_w, 3);
	
	rc_vector_zeros(&uy->xc, NX_C);
	rc_vector_zeros(&uy->xr, NX_C);
	rc_vector_zeros(&uy->xe, NX_C);
	rc_vector_zeros(&uy->u, 3);
	rc_vector_zeros(&uy->uw, 3);
	rc_vector_zeros(&uy->uw_dc, 3);
	
	// Initialize the matrices
	uy->K = rc_matrix_empty();
	uy->T_b2w = rc_matrix_empty();
	uy->T_w2b = rc_matrix_empty();
	uy->C = rc_matrix_empty();
	uy->R_accel = rc_matrix_empty();
	
	// init_controller_gain(&uy->K_drive);
	// init_controller_gain_spin(&uy->K_spin);
	// rc_matrix_duplicate(uy->K_drive, &uy->K);
	
	// Initial target controller gain
	uy->K_target = rc_matrix_empty();
	rc_matrix_zeros(&uy->K_target, NU_C, NX_C);
	set_controller_gain(&uy->K_target,0);	
	rc_matrix_duplicate(uy->K_target, &uy->K);
	
	init_transformation_matrix(&uy->T_b2w, &uy->T_w2b);
	//init_drive_coast_surface(&uy->C, N_DCS);
	
	
	// Accelerometer tilt correction
	// R_accel = Rz*Ry*Rx
	rc_matrix_t Rx = rc_matrix_empty();
	rc_matrix_t Ry = rc_matrix_empty();
	rc_matrix_t Rz = rc_matrix_empty();
	
	rc_matrix_identity(&Rx,3);
	rc_matrix_identity(&Ry,3);
	rc_matrix_identity(&Rz,3);
	
	// x rotation
	Rx.d[1][1] = cos(ACCEL_ANGLE_X);
	Rx.d[1][2] = -sin(ACCEL_ANGLE_X);
	Rx.d[2][1] = sin(ACCEL_ANGLE_X);
	Rx.d[2][2] = cos(ACCEL_ANGLE_X);
	
	// y rotation
	Ry.d[0][0] = cos(ACCEL_ANGLE_Y);
	Ry.d[0][2] = sin(ACCEL_ANGLE_Y);	
	Ry.d[2][0] = -sin(ACCEL_ANGLE_Y);
	Ry.d[2][2] = cos(ACCEL_ANGLE_Y);
	
	// z rotation
	Rz.d[0][0] = cos(ACCEL_ANGLE_Z);
	Rz.d[0][1] = -sin(ACCEL_ANGLE_Z);
	Rz.d[1][0] = sin(ACCEL_ANGLE_Z);
	Rz.d[1][1] = cos(ACCEL_ANGLE_Z);
	
	// Acceleration rotation matrix
	rc_matrix_duplicate(Rx, &uy->R_accel);
	rc_matrix_left_multiply_inplace(Ry,&uy->R_accel);
	rc_matrix_left_multiply_inplace(Rz,&uy->R_accel);
	
	// Scalar values
	uy->w_max = WR_MAX;	// Maximum no load speed
	
	// Tap filter variables
	uy->A_tap = rc_matrix_empty();
	uy->tr_x = rc_vector_empty();
	uy->tr_y = rc_vector_empty();
	uy->w_1 = rc_vector_empty();
	uy->w_2 = rc_vector_empty();
	uy->w_3 = rc_vector_empty();	
	
	init_tap_filter_matrix(&uy->A_tap,N_TAP,P_TAP);
	rc_vector_zeros(&uy->tr_x, N_TAP+1);
	rc_vector_zeros(&uy->tr_y, N_TAP+1);
	rc_vector_zeros(&uy->w_1, N_TAP+1);
	rc_vector_zeros(&uy->w_2, N_TAP+1);
	rc_vector_zeros(&uy->w_3, N_TAP+1);
	
	uy->Ru = rc_matrix_empty();
	rc_matrix_zeros(&uy->Ru, 3, 3);
	
	uy->Tu = rc_matrix_empty();
	rc_matrix_zeros(&uy->Tu, 3, 3);
	init_phi_transformation(&uy->Tu);
	
		
	return;
}

void exit_mbbr_states(mbbr_states* s){
	
	rc_vector_free(&s->x);
	rc_vector_free(&s->f);
	rc_vector_free(&s->h);
	rc_vector_free(&s->y);
	rc_vector_free(&s->u);
	
	rc_matrix_free(&s->F);
	rc_matrix_free(&s->H);
	
	rc_matrix_free(&s->Q);
	rc_matrix_free(&s->R);
	rc_matrix_free(&s->Pi);
	
	return;
}

void exit_io_data(io_data* uy){
	
	rc_vector_free(&uy->theta);
	rc_vector_free(&uy->gyro);
	rc_vector_free(&uy->accel);
	rc_vector_free(&uy->magnet);
	
	rc_vector_free(&uy->phi);
	rc_vector_free(&uy->phi_old);
	rc_vector_free(&uy->phid);
	rc_vector_free(&uy->phi_w);
	rc_vector_free(&uy->phi_w_old);
	rc_vector_free(&uy->phid_w);
	
	rc_vector_free(&uy->xc);
	rc_vector_free(&uy->xr);
	rc_vector_free(&uy->xe);
	rc_vector_free(&uy->u);
	rc_vector_free(&uy->uw);
	rc_vector_free(&uy->uw_dc);
	
	rc_matrix_free(&uy->K);
	rc_matrix_free(&uy->T_b2w);
	rc_matrix_free(&uy->T_w2b);
	rc_matrix_free(&uy->C);
	
	rc_matrix_free(&uy->K_target);
	
	rc_matrix_free(&uy->A_tap);
	rc_vector_free(&uy->tr_x);
	rc_vector_free(&uy->tr_y);
	rc_vector_free(&uy->w_1);
	rc_vector_free(&uy->w_2);
	rc_vector_free(&uy->w_3);
	
	rc_matrix_free(&uy->Ru);
	rc_matrix_free(&uy->Tu);
	
	
	
	return;
}

void reset_mbbr_states(mbbr_states* s){
	
	// Reset the Kalman Filter vectors
	rc_vector_zero_out(&s->x);
	rc_vector_zero_out(&s->f);
	rc_vector_zero_out(&s->y);
	rc_vector_zero_out(&s->h);
	rc_vector_zero_out(&s->u);
	
	// Reset the Kalman Filter matrices
	rc_matrix_zero_out(&s->F);
	rc_matrix_zero_out(&s->H);
	
	return;
}

void reset_io_data(io_data* uy){
	
	// Reset encoder measurements
	rc_encoder_eqep_write(ENC_CH_1,0);
	rc_encoder_eqep_write(ENC_CH_2,0);
	rc_encoder_eqep_write(ENC_CH_3,0);
	
	// Reset encoder measurement values
	rc_vector_zero_out(&uy->phi);
	rc_vector_zero_out(&uy->phi_old);
	rc_vector_zero_out(&uy->phid);
	rc_vector_zero_out(&uy->phi_w);
	rc_vector_zero_out(&uy->phi_w_old);
	rc_vector_zero_out(&uy->phid_w);
	
	
	// Reset controller vectors
	rc_vector_zero_out(&uy->xc);
	rc_vector_zero_out(&uy->xr);
	rc_vector_zero_out(&uy->xe);
	rc_vector_zero_out(&uy->u);
	rc_vector_zero_out(&uy->uw);
	rc_vector_zero_out(&uy->uw_dc);
	
	set_controller_gain(&uy->K_target,0);	
	rc_matrix_duplicate(uy->K_target, &uy->K);
	
	uy->phid_i_x = 0;
	uy->phid_i_y = 0;
	
	rc_vector_zero_out(&uy->tr_x);
	rc_vector_zero_out(&uy->tr_y);
	rc_vector_zero_out(&uy->w_1);
	rc_vector_zero_out(&uy->w_2);
	rc_vector_zero_out(&uy->w_3);
	
	input_transformation_matrix(&uy->Ru,0,0);
	
	return;
}

void reset_variables(){
	
	// Input/output variables
	reset_io_data(&uy);
	
	// EKF variables
	reset_mbbr_states(&s);
	reset_mbbr_states(&sl);
	rc_kalman_reset(&ekf);
	rc_kalman_reset(&ekfl);
	
	// Filter variables
	reset_filter_2nd(&phid_fx);
	reset_filter_2nd(&phid_fy);
	
	// Temporary variables
	rc_vector_zero_out(&temp_vec_1);
	rc_vector_zero_out(&temp_vec_2);
	rc_vector_zero_out(&temp_vec_3);
	
	// Others
	
	
	bt_x = 0;
	bt_y = 0;
	bt_button = 0;
	bt_start = 0;
	
	temp_d_1 = 0;
	temp_d_2 = 0;
	temp_d_3 = 0;
	
	
	driving_state = STOP;
	spinning_state = NOT_SPINNING;
	spinning_state_old = NOT_SPINNING;
	
	rc_x = 0;
	rc_y = 0;
	rc_z = 0;
	
	
	
	
	slc_theta_x = 0;
	slc_theta_y = 0;
	slc_theta_x_old = 0;
	slc_theta_y_old = 0;	
	slc_thetaD_x = 0;
	slc_thetaD_y = 0;
	phiD_e_x_old = 0;
	phiD_e_y_old = 0;
	
	spinning_flag = 0;
	spinning_flag_old = 0;
	
	phi_in_x = 0;
	phi_in_y = 0;
	phidot_in_x = 0;
	phidot_in_y = 0;
	
	
	
	slc_spd_gain   = SLC_GAIN[0];
	slc_spd_coef_2 = SLC_COEF_2[0];
	
	
	
	count_test_1 = 0;
	count_test_2 = 0;
	
	if (CONTROLLER_TYPE == 1) {
		est_st = EKF;
		offset_yaw = 0;
	}
	else if (CONTROLLER_TYPE == 2) {
		est_st = KF;
		offset_yaw =  uy.theta.d[2];
	}
	else {
		est_st = RAW;
		offset_yaw =  uy.theta.d[2];
	}
	
	
	
	
	spin_rate = 0;
	drive_rate_x = 0;
	drive_rate_y = 0;
	
	yaw_forward = 0;
	yaw_0 = uy.theta.d[2];
	
	
	s.x.d[0] =  uy.theta.d[0];
	s.x.d[1] =  uy.theta.d[1];
	sl.x.d[0] =  uy.theta.d[0];
	sl.x.d[1] =  uy.theta.d[1];
	

	
	flag_spin_imbalance = 0;
	rc_z2 = 0;
	
	return;
}

/* =============================================================================
	MAIN FUNCTION
============================================================================= */

int main(){
	/* -------------------------------------------------------------------------
		Initialize robotics cape:
		- Kill existing cape process
		- Enable signal handler
		- Project ID
		- Buttons
		- Encoders
		- ADC
		- Motors
		- I2C
		- printf
		- frpintf
	------------------------------------------------------------------------- */

	/* -------------------------------------------------------------------------
		Cape initialization
	------------------------------------------------------------------------- */

	// Kill existing cape process
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// Start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// Make PID file to indicate your project is running
	rc_make_pid_file();

	// Initialize buttons (on release only)
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize mode button\n");
		return -1;
	}
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,NULL,on_pause_release);
	rc_button_set_callbacks(RC_BTN_PIN_MODE,NULL,on_mode_release);

	// LED setup
	if(rc_led_set(RC_LED_GREEN, 0)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_GREEN\n");
		return -1;
	}
	if(rc_led_set(RC_LED_RED, 1)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_RED\n");
		return -1;
	}

	printf("Test1\n");
	// Initialize encoders. PRU for encoder 4
	if(rc_encoder_eqep_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
		return -1;
	}

	// Initialize adc
	if(rc_adc_init()==-1){
		fprintf(stderr, "failed to initialize adc\n");
	}

	// Initialize motors
	if(rc_motor_init_freq(PWM_FREQ)) return -1;

	// Setup IMU interrupt
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLING_RATE_HZ;
	mpu_config.orient = ORIENTATION_Y_UP;
	mpu_config.dmp_fetch_accel_gyro = 1;
	mpu_config.enable_magnetometer = 1;
	
	// if gyro isn't calibrated, run the calibration routine
	if(!rc_mpu_is_gyro_calibrated()){
		printf("Gyro not calibrated, automatically starting calibration routine\n");
		printf("Let your MiP sit still on a firm surface\n");
		rc_mpu_calibrate_gyro_routine(mpu_config);
	}
	
	// start mpu
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
		rc_led_blink(RC_LED_RED, 5, 5);
		return -1;
	}
	
	// Initialize Bluetooth
	init_bluetooth();
	
	// -------------------------------------------------------------------------
	// 		Initializate variables
	// -------------------------------------------------------------------------
	// rc_usleep(1000000);	// Sleep for 1 seconds
	
	// Initial time stamps
	clock_gettime(CLOCK_MONOTONIC, &t0);
	clock_gettime(CLOCK_MONOTONIC, &t1);
	t_ms = 0;
	t_mode = 0;
	t_start = 0;
	t_end = RECORD_TIME_END*1000*60;

	// Allocate fprintf data memory
	data_row = t_end/1000*SAMPLING_RATE_HZ/(1+SKIP_REC_STEP) + 100;
	data_col = DATA_COL_LENGTH;
	data = (double*) malloc(data_row*data_col*sizeof(double));
	if (data == NULL){
		fprintf(stderr,"ERROR: failed to allocate recording data memory.\n");
		return -1;
	}
	
	// Initialize 3d and linear ekf model struct
	init_mbbr_states(&s, NX, NY, NU, P0);
	init_mbbr_states(&sl, NX_L, NY_L, NU_L, P0);
	init_noise_data_3d(&s);
	init_noise_data_lin(&sl);
	
	// Initialize IO data
	init_input_output_data(&uy);

	// Initialize Kalman Filter structs
	ekf = rc_kalman_empty();
	ekfl = rc_kalman_empty();
	rc_kalman_alloc_ekf(&ekf, s.Q, s.R, s.Pi);
	rc_kalman_alloc_ekf(&ekfl, sl.Q, sl.R, sl.Pi);
	
	// Override James' original singular matrix determinant tolerance
	if (USE_CUSTOM_ZERO_TOLERANCE){
		rc_algebra_set_zero_tolerance(MY_ZERO_TOLERANCE);
	}
	
	// 1st order LP filter
	init_filter_2nd(&phid_fx,0,1+PHIDOT_LP_WC[0],0,PHIDOT_LP_WC[0],0);
	init_filter_2nd(&phid_fy,0,1+PHIDOT_LP_WC[0],0,PHIDOT_LP_WC[0],0);
	
	// Other variables
	v_battery = 0;
	
	// Initialize other variables
	rec_count = 0;
	rec_left = 0;
	n_skip = 0;
	
	temp_vec_1 = rc_vector_empty();
	temp_vec_2 = rc_vector_empty();
	temp_vec_3 = rc_vector_empty();
	rc_vector_zeros(&temp_vec_1, 3);
	rc_vector_zeros(&temp_vec_2, 3);
	rc_vector_zeros(&temp_vec_3, 3);
	
	disarm_controller();
	
	if (START_FPRINTF) 	start_fprintf = 1;
	else	start_fprintf = 0;
	
	
	// -------------------------------------------------------------------------
	// 		Pre-experiment console printouts
	// -------------------------------------------------------------------------
	printf("MBBR setting up!\n");
	
	printf("Sampling rate: %i\n", SAMPLING_RATE_HZ);
	printf("Motor driver PWM frequency: %i\n", PWM_FREQ);
	printf("Output data: %i rows and %i columns\n", data_row, data_col);
	printf("End time = %7.3f seconds\n", t_end/1000.0);	
	
	printf("\nRUNNING: Begin the balancing.\n");
	
	/* -------------------------------------------------------------------------
		Thread Setup:
		- printf thread
		- fprintf thread
		- setpoint thread
		- imu interrupt thread
	------------------------------------------------------------------------- */
	pthread_t th1 = 0;
	pthread_t th2 = 0;
	pthread_t th3 = 0;

	// printf thread
	if(isatty(fileno(stdout)) && !DISABLE_PRINTF_THREAD){
		if(rc_pthread_create(&th1, printf_loop, (void*) NULL, SCHED_OTHER, 0)){
			fprintf(stderr, "failed to start printf thread\n");
			return -1;
		}
	}
	
	// fprintf thread
	if(rc_pthread_create(&th2, fprintf_loop, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start dyno thread\n");
		return -1;
	}
	
	// Setpoint thread
	if(rc_pthread_create(&th3, setpoint_loop, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start dyno thread\n");
		return -1;
	}
	
	// this should be the last step in initialization
	// to make sure other setup functions don't interfere
	rc_mpu_set_dmp_callback(&mbbr_imu_interrupt_loop);
	
	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		rc_usleep(100000);
	}

	/* -------------------------------------------------------------------------
		Exit routine
	------------------------------------------------------------------------- */
	// Wait until done printing data
	printf("EXPERIMENT DONE, writing the rest of the data to text file.\n");
	printf("Need to write %i more data.\n",rec_left);
	//fprintf_data(data, data_row, data_col);
	while(fprintf_not_done){
		// Wait until done printing.
		rc_usleep(100000);
	}
	printf("DONE!\n");

	// Cleanup
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	
	rc_mpu_power_off();
	rc_button_cleanup();
	rc_encoder_eqep_cleanup();
	rc_motor_cleanup();
	
	// Free memory
	free(data);
	rc_kalman_free(&ekf);
	rc_kalman_free(&ekfl);
	exit_mbbr_states(&s);
	exit_mbbr_states(&sl);
	exit_io_data(&uy);
	rc_vector_free(&temp_vec_1);
	rc_vector_free(&temp_vec_2);
	rc_vector_free(&temp_vec_3);
	
	
	exit_filter_2nd(&phid_fx);
	exit_filter_2nd(&phid_fy);
	
	// -------------------------------------------------------------------------
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}
