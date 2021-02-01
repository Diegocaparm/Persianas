#include "main.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

//Funciones servo
void servo_write(int); //Escribir angulo
int map(int, int, int, int, int); //Mapeado
void subir_persiana(); //Subir al máximo
void bajar_persiana(); //Bajar al mínimo

//Variables servo
int i; //Valor que irá al servo
int max; //Valor que nos indicará si está en un máximo nuestro servo

//Funciones ADC
int16_t LDR(void); //Leer valor LDR
int16_t pot(void); //Leer valor potenciometro
void lector(int16_t); //Paso de 0-255 a 0-180 en formato int

//Variables ADC
int16_t potval, potv;		//Valor del potenciometro y variable que igualaremos a esta
int16_t j; 					//Variable que indicará el tramo del rango de valores donde estamos
int16_t luminosidad, lum;	//Valor del LDR y variable que igualaremos a esta

//Funciones ultrasonidos
uint32_t hcsr04_read (void); //Leer tiempo de vuelta

//Variables ultrasonidos
uint32_t local_time; 	//Variables que indican el tiempo que tardamos en detectar el objeto
uint32_t sensor_time;	//tanto dentro como fuera de la función
uint32_t distance; 		//Variable distancia del objeto detectado al sensor

//Funciones contadores
void waitmin(int); 		//Contador de ms
void wait(int); 		//Delay discreto en uS
int h; 					//Variable para el contador wait


//Variables interrupciones
volatile int flag = 0; 	//Bandera que nos lleva al cambio de modo
volatile int modo = 0;	//Variable que nos indica el modo actual
volatile int t = -1;	//Variable asignada al contador TIM2

//Funciones interrupciones
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==GPIO_PIN_0) 	//Si se activa el pin PA0
		flag = 1;				//Encender la bandera
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim ->Instance == TIM2) //A cada interrupción del TIM2
		t++; //Aumentar el número periodos de 20 minutos, iniciado a las 9:00
	if (modo == 0) //Si estamos en reposo
	{
		if (t == 36)
			bajar_persiana(); //A las 21:00, bajar las persianas
		if (t == 72)
		{
			subir_persiana();//A las 9:00 subir las persianas
			t = 0; //Y reiniciar el contador
		}
	}
	if (t == 72) //En caso de que haya que reiniciar el contador en otro modo
		t = 0;
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {
	  if (flag) 		//Si se activa la bandera de interrupción
	  {
		  modo = 0; 	//Estado de reposo y selección de modo
		  if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) && (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 0) && (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == 0)))
			  modo = 1; //Modo sensor de presencia si sólo hemos usado el PD1
		  else if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) && (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == 0) && (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == 0)))
			  modo = 2; //Modo sensor de luz si sólo hemos usado el PD2
		  else if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) && (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 0) && (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == 0)))
			  modo = 3; //Modo manual ajustable si sólo hemos usado el PD3
		  else 			//Si usamos varios pulsadores externos a la vez, ninguno o hay algún problema
			  modo = 0; //Entramos en modo de reposo
		  flag = 0; 	//E igualamos la bandera a 0 nuevamente
	  }

	  while (modo == 1){
		  //Hcsr04
		  if (flag == 0) // Mientras no haya una interrupcion que se ejecute el siguiente programa
		  {
		      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1); // Enciende el LED verde que nos indica el modo que estamos usando
		      sensor_time = hcsr04_read();				// Igualamos sensor_time a la salida de la función
		      distance  = sensor_time /17;				// Dividimos el tiempo entre 17 para hallar la distancia
		      if(distance<30)							// Si esta baja de 30 cm hay presencia
			      subir_persiana();						// Y subimos las persianas automáticamente
		      else										// De no haber nadie en la sala
			      bajar_persiana();						// Las bajamos de nuevo
		      waitmin(3000);							// Vuelve a comprobar la proximidad cada 3 segundos
		  }
		  else 											// Si hay una interrupción o pasa cualquier cosa, volvemos al modo 0
              modo = 0;
	  }

	  while (modo == 2){
		  //LDR
		  if (flag == 0) //Mientras no haya una interrupcion que se ejecute el siguiente programa
		  {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);	// Enciende el LED naranja que nos indica el modo que usaremos
			  lum = LDR();								// Igualamos lum al valor medido por el LDR
			  if (lum > 80)								// Si esta supera el umbral de 80, habrá mucha luz
				  subir_persiana();						// Subimos las persianas
			  else if (lum < 30)						// Si lum baja de 30, habrá muy poca luz
				  bajar_persiana();						// Bajamos las persianas
			  waitmin(300000); 							// Recarga el valor cada 5 minutos
		  }
		  else 											// Si hay una interrupción o pasa cualquier cosa, volvemos al modo 0
			  modo = 0;
	  }

	  while (modo == 3){
		  //Potenciometro
		  if (flag == 0) //Ejecutamos el programa mientras no haya interrupciones
		  {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1); // Enciende el LED rojo que nos indica el modo que estamos usando
			  potv = pot();								// Igualamos potv a el valor medido en el potenciometro
			  lector(potv);								// Cargamos ese valor para dividirlo en tramos y cargarlo al servo
			  waitmin(20); 								// Recargamos el valor cada 20 mS para que sea algo continuo y fluido
		  }
		  else 											// Vuelta al modo 0 en caso de pulsar el PA0 o de fallo
			  modo = 0;
	  }

	  // En caso de que se salga de cualquier modo y no se selecciones otro
	  // Apagamos todos los leds y mantenemos el modo a 0 (Reposo)
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	  modo = 0;
  }

}

int16_t pot(void)
{
	HAL_ADC_Start(&hadc3);					// Inicializamos el canal ADC3
	HAL_ADC_PollForConversion(&hadc3, 100); // Medimos el valor del potenciometro de 0 a 255
	potval = HAL_ADC_GetValue(&hadc3);		// Igualamos ese valor a la variable
	return potval;							// Y devolvemos esa variable
}

void subir_persiana(void)
{
	if (max != 2) //Si la persiana no está ya subida
		servo_write (180); //Subirla al máximo
}

void bajar_persiana(void)
{
	if (max != 1) //Si la persiana no está ya bajada
		servo_write (0); //Bajarla al máximo
}

int16_t LDR(void)
{
	HAL_ADC_Start(&hadc1);					// Inicializamos el canal ADC1
	HAL_ADC_PollForConversion(&hadc1, 100); // Medimos el valor del LDR de 0 a 255
	luminosidad = HAL_ADC_GetValue(&hadc1);	// Igualamos la variable luminosidad al valor medido
	return luminosidad;						// Devolvemos ese valor
}

void lector(int16_t val)
{
	i = 0; 				// Iniciamos un int a 0 que será los grados que haremos girar al servo
	j = 14;				// Igualamos la variable que compararemos a 14 (primer tramo avanzado)
	while (val > j)		// Mientras el valor del potenciometro sea mayor que j
	{
		i = i + 10;		// Aumentamos en 10 los grados que giramos
		j = j + 14;		// Aumentamos en 14 la variable a comparar (avanzamos un tramo)
		if (i > 180)	// Si nos pasamos de 180
			j = 0;		// Volvemos a empezar porque hay un fallo
	}					// Saldremos de este bucle cuando el valor del potenciometro esté por
						// debajo de j (habremos encontrado el tramo de valores en el que está)
	servo_write(i);		// Giramos en el servo 10 grados * número de tramos avanzado
}

void servo_write(int angle)
{
	htim1.Instance->CCR1 = map(0,180,50,250,angle); //Mandamos un pulso del tamaño que nos indique map
	if (angle == 0)									//en función del ángulo que introduzcamos
		max = 1; //Si el ángulo es 0 que registre que la persiana queda bajada al tope
	else if (angle == 180)
		max = 2; //Si el ángulo es 180 que registre que la persiana queda subida al tope
	else
		max = 0; //Si el ángulo no es ningún extremo
}

int map(int st1, int fn1, int st2, int fn2, int value)
{			//0, 180, 50, 250, ángulo deseado
    return (1.0*(value-st1))/((fn1-st1)*1.0) * (fn2-st2)+st2;
}			//Nos devuelve el pulso entre 0,5 ms y 2,5 ms equivante al ángulo deseado

void wait(int time) //Contador en uS
{
	h = 0;			//Iniciamos la cuenta a 0
	while(h<time)  	//Mientras h no llegue al tiempo pedido
	{
		if (flag)  	//En caso de interrupción
			break; 	//Salir del bucle
		h++;	   	//Aumentar h si no las hay
	}
}

void waitmin(int timemin) //Contador en mS
{
	int tickstart = HAL_GetTick();  //Iniciamos igualandola al contador interno del reloj
	while((HAL_GetTick() - tickstart) < timemin) //Mientras reloj avance el tiempo que le pedimos
	{								//Que siga esperando
      if(flag)						//Y en caso de interrupcion
			break;					//Salir del bucle
	}
}

uint32_t hcsr04_read (void)
{
 local_time=0; 											// Igualamos el contador a 0
 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);  // Ponemos el TRIGGER a 0
 wait(2);  												// Esperamos por 2 us
 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);  	// Ponemos el TRIGGER a 1
 wait(10);   											// Esperamos por 10 us
 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);  // Ponemos el TRIGGER a 0

 while (!(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)));  		// Esperamos a que se active ECHO
 while (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10))    		// Mientras el ECHO esté activo
  {
  local_time++;   										// Aumentamos la cuenta de cuanto tiempo tardan en volver las ondas
  }
 return local_time;										// Devolvemos el valor de la cuenta
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_8B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 159;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);

}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 299999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif