/*
 *
 *
 * //Da vedere l'ordine con printf
    //Speeds_quad sono le velocità angolari al quadrato
    // Speeds_quad[0] = ESC 3
    // Speeds_quad[1] = ESC 2
    // Speeds_quad[2] = ESC 1
    // Speeds_quad[3] = ESC 4
     *
     *
     *

void stabilizeMotors()
{
	float virtualInputs[4];
	readImu();

	//prev_rif_yaw non parte azzerato ad ogni ciclo, perché è dichiarato come static,
	//quindi mantiene il suo valore tra le chiamate alla funzione
	static int prev_rif_yaw = 0; // Memorizza il valore precedente della levetta
	static int flag_control = 0; // Indica se il PID deve continuare a controllare

	// Controllo cambio stato della levetta dell'imbardata
	if (rif_yaw != prev_rif_yaw)
	{
		prev_rif_yaw = rif_yaw; // Aggiorna lo stato precedente
		yaw_setpoint = yaw + yaw_angolo;

		// Assicura che yaw_setpoint sia nel range [0, 360]
		if (yaw_setpoint >= 360)
			yaw_setpoint -= 360;
		else if (yaw_setpoint < 0)
			yaw_setpoint += 360;

		flag_control = 1; // Attiva il controllo PID
	}

	// Se il PID ha raggiunto lo yaw_setpoint o yaw_angolo è 0, ferma il controllo
	//fabs() restituisce il valore assoluto di un numero in virgola mobile
	if ((flag_control && fabs(yaw - yaw_setpoint) < 1.0) || rif_yaw == 0) // Tolleranza di 1 grado
	{
		flag_control = 0;
	}

	virtualInputs[0] = 9; // Peso del drone
	virtualInputs[1] = PID_Controller(&RollPID, roll, 0);
	virtualInputs[2] = PID_Controller(&PitchPID, pitch, 0);
	virtualInputs[3] = (flag_control) ? PID_Controller(&YawPID, yaw, yaw_setpoint) : 0;
	//virtualInputs[3] = 0;

	float* Speeds;
	Speeds = SpeedCompute(virtualInputs);

	//Sono stati messi degli offset nel tentativo di bilanciare se spinte dei due motori
	float avgMotor1 = map(*(Speeds+0)); //+ 0.04+ 0.019;
	float avgMotor2 = map(*(Speeds+1)); //+ 0.0295;
	float avgMotor3 = map(*(Speeds+2)); //- 0.0295;
	float avgMotor4 = map(*(Speeds+3)); //- 0.0295;

	if(flag_print)
	{
	  printf("avgMotor1: %.2f\r\n"
			  "avgMotor2: %.2f\r\n"
			  "avgMotor3: %.2f\r\n"
			  "avgMotor4: %.2f\r\n"
			  "roll: %f\r\n"
			  "pitch: %f\r\n"
			  "yaw: %f\r\n"
			  "rif_yaw: %d\r\n"
			  "yaw_setpoint: %.2f\r\n"
			  "Virtual Inputs: %f, %f, %f, %f\r\n\n",
			  avgMotor1, avgMotor2, avgMotor3, avgMotor4, roll, pitch, yaw, rif_yaw, yaw_setpoint,
			  virtualInputs[0],virtualInputs[1],virtualInputs[2],virtualInputs[3]);
	  flag_print=0;
	}

	setPwm(avgMotor1, avgMotor2, avgMotor3, avgMotor4);
}




*/






/*

 if(flagTim1)
	  {
		if(calibrateFlag)
		{
			calibrateFlag = 0;
			ESC_Calibrate(); //ATTACCA LA BATTERIA DOPO AVER SETTATO IL PWM PER GLI ESC = LIMIT_DUTY
		}

		if(provaAvvioMotori)
		{
			//MAI METTERE 0 COME DUTY
			//PROVA ANTONIO: fARE AVVIO SOLO DEGLI ESC 1 E 4
			HAL_Delay(3000);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //Led verde
			setPwm(MIN_CALIB_DUTY, MIN_CALIB_DUTY, MIN_CALIB_DUTY, MIN_CALIB_DUTY); //Inizio avvio motori
			HAL_Delay(2000);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); //Led giallo
			setPwm(MIN_CALIB_DUTY, MIN_CALIB_DUTY, ARM_DUTY, MIN_CALIB_DUTY); //Motori in armo
			HAL_Delay(4000);
			setPwm(MIN_CALIB_DUTY, MIN_CALIB_DUTY, 6.5, MIN_CALIB_DUTY); //Motori ad un certo duty > duty di armamento
			HAL_Delay(5000);
			stopMotors();
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //Led rosso
			provaAvvioMotori = 0;
		}

		//RADIOCOMANDO PROVA
		if(levaCH5)
		{
			if(flag_print)
			{
				//printf("\033[2J\033[H"); // Pulisce lo schermo di PuTTY
				printf("Stato i = %d\r\n", i);
				flag_print = 0;
			}
			// Switch-case per gestire gli stati del motore
			switch (i) //Vedere come mettere più istruzioni in un singolo case dello switch
			{
				case 0: // Stato 0: Motore spento

					stopMotors(); //Levetta giù
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //Led rosso
					break;

				case 1: // Stato 1: Motore in armamento
					armMotors(); //Levetta su
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); //Led giallo
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //Led rosso
					break;

				case 2: // Stato 2: Motore in esecuzione
					stabilizeMotors(); //Levetta al centro //motorRunning() mette 5.4% di duty
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //Led verde
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //Led rosso
					break;

				default: // Motori fermi
					stopMotors();
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //Led rosso
					break;
			}
		}


		if(levaCH2)
		{

			if(flag_print)
			{
				printf("Stato roll = %d\n", rif_roll);
				flag_print = 0;
			}
			switch (rif_roll)
			 {
				  case -1:
				  case -2:
				  case -3:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //Led rosso
				  break;

				  case 0: // Stato 1: Motore in armamento
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); //Led giallo
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //Led rosso
				  break;

				  case 1:
				  case 2:
				  case 3:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //Led verde
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //Led rosso
				  break;

				  default: // Motori fermi
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //Led rosso
				  break;
			 }
		}

		if(levaCH1)
		{

			if(flag_print)
			{
				printf("Stato pitch = %d\n", rif_pitch);
				flag_print = 0;
			}
			switch (rif_pitch)
			 {
				  case -1:
				  case -2:
				  case -3:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //Led rosso
				  break;

				  case 0: // Stato 1: Motore in armamento
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); //Led giallo
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //Led rosso
				  break;

				  case 1:
				  case 2:
				  case 3:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //Led verde
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //Led rosso
				  break;

				  default: // Motori fermi
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
				  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //Led rosso
				  break;
			 }
		}


		if(IMUcode)
		{
			stabilizeMotors();


			accel_data = bno055_getVector(BNO055_LIA_DATA_X_LSB);
			gyro_data = bno055_getVector(BNO055_GYR_DATA_X_LSB);
			mag_data = bno055_getVector(BNO055_MAG_DATA_X_LSB);
			euler_data = bno055_getVectorEuler();


			if(flag_print)
			{
				// Stampa dei dati su seriale
				printf("Ax: %.2f, Ay: %.2f, Az: %.2f [m/s^2]\r\n", accel_data.x, accel_data.y, accel_data.z);
				printf("Gx: %.2f, Gy: %.2f, Gz: %.2f [degree/s]\r\n", gyro_data.x, gyro_data.y, gyro_data.z);
				printf("Mx: %.2f, My: %.2f, Mz: %.2f [uT]\r\n", mag_data.x, mag_data.y, mag_data.z);
				printf("Ex: %.2f, Ey: %.2f, Ez: %.2f [gradi]\r\n", euler_data.x, euler_data.y, euler_data.z);
				printf("\r\n");
				flag_print = 0;
			}
			HAL_Delay(5000);

		}


		if(final)
		{
			switch (i) //Vedere come mettere più istruzioni in un singolo case dello switch
			{
				case 0: // Stato 0: Motore spento
					stopMotors(); //Levetta giù
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //Led rosso
					break;

				case 1: // Stato 1: Motore in armamento
					armMotors(); //Levetta su
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); //Led giallo
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //Led rosso
					break;

				case 2: // Stato 2: Motore in esecuzione
					stabilizeMotors(); //Levetta al centro
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //Led verde
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //Led rosso
					break;

				default: // Motori fermi
					stopMotors();
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //Led verde
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); //Led giallo
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //Led rosso
					break;
			}
		}

	 }
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
// radiocomando

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim5) //TIM5 controlla solo le azioni di armamento, esecuzione e fermata dei motori
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel1
        {
            if (chFrequency.firstCaptured == 0) // if the first value is not captured
            {
                chFrequency.firstCaptured = 1; // set the first captured as true
                __HAL_TIM_SET_COUNTER(htim, 0); // reset the counter
            }
            else // if the first is already captured
            {
                chFrequency.val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read value
                chFrequency.frequency = (float)(refClock / chFrequency.val); //Calcolo frequenza
                chFrequency.frequency = floorf(chFrequency.frequency * 100) / 100; //Arrotondamento a 2 cifre decimali

                chFrequency.firstCaptured = 0; // set it back to false
                //chFrequency.flagFirstFrequency = 0; //NON SERVE?
            }
        }

        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // if the interrupt source is channel4
        {
            if (chDuty[IC_CHANNEL5].firstCaptured == 0) // if the first value is not captured
            {
                chDuty[IC_CHANNEL5].firstCaptured = 1; // set the first captured as true
                __HAL_TIM_SET_COUNTER(htim, 0); // reset the counter
            }
            else // if the first is already captured
            {
                chDuty[IC_CHANNEL5].val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
                chDuty[IC_CHANNEL5].usWidth = (float)(chDuty[IC_CHANNEL5].val) / 1000000; // refClock; //Converte il valote in tempo (microsecondi)
                chDuty[IC_CHANNEL5].duty = chDuty[IC_CHANNEL5].usWidth * chFrequency.frequency * 100;
                chDuty[IC_CHANNEL5].duty = floorf(chDuty[IC_CHANNEL5].duty * 100) / 100;

                levetta = chDuty[IC_CHANNEL5].duty;

                //Livelli di DUTY
                //LEVEL1 con duty cycle da 4,75% a 6,2% (nel caso nostro da 5,3 a 6,2)
                //LEVEL2 con duty cycle da 6,2% a 6,6% (nel caso nostro da 6,2 a 6,6)
                // Stato della levetta
                if (levetta>OFF_RECEIVER_DUTY && levetta < LEVEL1_DUTY) //No errore, LV1 = 6.2, LV2 = 6.6)
				{
					// ARMAMENTO o STOP
					if (i == 0 || i == 1) //Fino a 6,2& di duty stanno sempre in armamento (stato 1)
					{
						i=1;
					}
					else if (i == 2 || i == 3) //Sennò si fermano i motori poichè non possono lavorare ad un duty cycle minore
					{
						stopMotors();
					}
				}
				else if (levetta >= LEVEL1_DUTY && levetta <= LEVEL2_DUTY)
				{
					i=2;
				}
				else
				{
				 stopMotors(); //Operazione di default -> Ferma i motori sopra ad un duty cycle di 6,6% (max testato)
				}

                chDuty[IC_CHANNEL5].firstCaptured = 0;
            }
        }
    }

    //TIM2 controlla solo le azioni di rollio e beccheggio per ora
    //Probabilmente dovremo attivare un altro channel del TIM2 per definire il Yaw (imbardata da implementare)
	if (htim == &htim2)
	{
		//channel1 per il rollio
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if (chDuty[IC_CHANNEL1].firstCaptured == 0) // if the first value is not captured
			{
				chDuty[IC_CHANNEL1].firstCaptured = 1; // set the first captured as true
				__HAL_TIM_SET_COUNTER(htim, 0); // reset the counter
			}
			else //gestione calcolo duty cycle e rollio
			{
				chDuty[IC_CHANNEL1].val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				chDuty[IC_CHANNEL1].usWidth = (float)(chDuty[IC_CHANNEL1].val) / 1000000; //refClock;
				chDuty[IC_CHANNEL1].duty = chDuty[IC_CHANNEL1].usWidth * chFrequency.frequency * 100;
				chDuty[IC_CHANNEL1].duty = floorf(chDuty[IC_CHANNEL1].duty * 100) / 100;

				calcolo_roll = chDuty[IC_CHANNEL1].duty; //livello del rollio (in percentuale) del segnale PWM

				//In generale le condizioni seguenti servono a discretizzare il livello di rollio
				//associandogli un certo valore da -3 a 3 in base alla fascia di appartenenza
				//Controlla se il valore del duty cycle (calcolo_roll) rientra in un intervallo predefinito (tra 5% e 15%)
				if(calcolo_roll < 15 && calcolo_roll > 5)
				{
				    if(calcolo_roll < 9.8 && calcolo_roll > 8.5)
				        rif_roll=-1;
				    if(calcolo_roll <= 8.5 && calcolo_roll > 7.5)
				        rif_roll =-2;
				    if(calcolo_roll <= 7.5 && calcolo_roll > 6.3)
				        rif_roll =-3;
				    if(calcolo_roll <= 10.3 && calcolo_roll >= 9.8)
				        rif_roll = 0;
				    if(calcolo_roll > 10.3 && calcolo_roll <= 11.3)
				        rif_roll = 1;
				    if(calcolo_roll > 11.3 && calcolo_roll <= 12.3)
				        rif_roll = 2;
				    if(calcolo_roll > 12.3 && calcolo_roll <= 13.5)
				        rif_roll = 3;
				}

				// provaroll = chDuty[IC_CHANNEL1].duty;
				chDuty[IC_CHANNEL1].firstCaptured = 0;
			}
		}

		//Stessa implementazione ma per il pitch (beccheggio)
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
		    if(chDuty[IC_CHANNEL3].firstCaptured == 0) // if the first value is not captured
		    {
		        chDuty[IC_CHANNEL3].firstCaptured = 1; // set the first captured as true
		        __HAL_TIM_SET_COUNTER(htim, 0);        // reset the counter
		    }
		    else
		    {
		        chDuty[IC_CHANNEL3].val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		        chDuty[IC_CHANNEL3].usWidth = (float) (chDuty[IC_CHANNEL3].val) / 1000000; // refClock;
		        chDuty[IC_CHANNEL3].duty = chDuty[IC_CHANNEL3].usWidth * chFrequency.frequency * 100;
		        chDuty[IC_CHANNEL3].duty = floorf(chDuty[IC_CHANNEL3].duty * 100) / 100;

		        calcolo_pitch = chDuty[IC_CHANNEL3].duty;

		        if(calcolo_pitch < 15 && calcolo_pitch > 5){
		            if(calcolo_pitch < 9.8 && calcolo_pitch > 8.5)
		                rif_pitch=-1;
		            if(calcolo_pitch <= 8.5 && calcolo_pitch > 7.5)
		                rif_pitch =-2;
		            if(calcolo_pitch <= 7.5 && calcolo_pitch > 6.3)
		                rif_pitch =-3;
		            if(calcolo_pitch <= 10.3 && calcolo_pitch >= 9.8)
		                rif_pitch = 0;
		            if(calcolo_pitch > 10.3 && calcolo_pitch <= 11.3)
		                rif_pitch = 1;
		            if(calcolo_pitch > 11.3 && calcolo_pitch <= 12.3)
		                rif_pitch = 2;
		            if(calcolo_pitch > 12.3 && calcolo_pitch <= 13.5)
		                rif_pitch = 3;
		        }

		        chDuty[IC_CHANNEL3].firstCaptured = 0;
		    }
		}
	}

}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim5){
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // If the interrupt is triggered by channel 1
		{
			// Read the IC value
			uint32_t ICValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

			if (ICValue != 0)
			{
				// calculate the Duty Cycle
				uint32_t duty_received = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) *100)/ICValue;
				if(duty_received >= 11){
					mode = 0;
				}
				else if (duty_received <= 8){
					mode = 2;
				}
				else {
					mode = 1;
				}
			}
		}
	}
}


// radiocomando
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim5) //TIM5 controlla solo le azioni di armamento, esecuzione e fermata dei motori
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //Led verde
    	float chFrequency = 0;
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel 1
        {

        	chFrequency = refClock/(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)); // read value (refClock = 64000000)
        	chFrequency = floorf(chFrequency * 100) / 100;
        	printf("Frequenza = ", chFrequency, " \r\n");
        }

        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // if the interrupt source is channel 2
        {
        	float upWidth = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4))/1000000;
        	float chDuty = upWidth * chFrequency * 100; //Calcolo duty
        	float levetta = floorf(chDuty * 100) / 100;
        	printf("Levetta = ", levetta, " \r\n");

			//Livelli di duty cycle radiocomando: <7, tra 7 e 11, >11
			//Stato della levetta del channel 5 del radiocomando
        	//Stati della i gestiti nel while(1)
			if(levetta > 7 && levetta < 11) //armamento
			{
				i = 1;
			}
			else if (levetta <= 7) //esecuzione
			{
				i = 2;
			}
			else //levetta > 11
			{
				i = 0;
			}
        }
    }
}

*/
































