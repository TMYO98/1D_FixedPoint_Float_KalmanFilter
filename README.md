Just copy the .h and .c on your directory, initialize the kalman and/or mean variables, and call the functions, done!, if Fixed point used, R and Q values must be the equivalent of float * 65536
you can change this scale if necessary, just keep it constant
i.e: 

	FixedKalmanFilter_t *Fxkf = (FixedKalmanFilter_t *)malloc(sizeof(FixedKalmanFilter_t));
	Median_t *Median = (Median_t *)malloc(sizeof(Median_t));
	Median_init(Median);
	int32_t Q = (int32_t)(.003f * 65536);
	int32_t R = (int32_t)(.015f * 65536);
	Fixedkalman_init(Fxkf, 0, 0, Q, R);


