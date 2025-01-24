/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef enum {
    WAITING_BUTTON,
	lavando,
    MOVING_MOTOR_2,
	EMERGENCY  // Nuevo estado
} SystemState;

typedef enum {
    EJECUTANDO,
	ESPERANDO // Nuevo estado
} pepe;
/* USER CODE END PTD */
volatile SystemState currentState = WAITING_BUTTON;
volatile  pepe currentState2 = ESPERANDO;

#include "i2c-lcd.h"
#include "DHT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Definiciones de pines
#define BUTTON_PIN GPIO_PIN_15
#define BUTTON_PORT GPIOC

#define EMERGENCY_BUTTON_PIN GPIO_PIN_3  // Botón de emergencia en PC15
#define EMERGENCY_PORT GPIOE             // Puerto del botón de emergencia

#define USER_BUTTON_PIN GPIO_PIN_0       // Botón de usuario (rearme) en PA0
#define USER_BUTTON_PORT GPIOA          // Puerto del botón de usuario

#define LED_RED_PIN GPIO_PIN_13
#define LED_GREEN_PIN GPIO_PIN_14
#define LED_PORT GPIOC

#define MOTOR1_PIN GPIO_PIN_3
#define MOTOR2_PIN GPIO_PIN_7
#define MOTOR1_PORT GPIOB
#define MOTOR2_PORT GPIOA

#define ECHO2_PIN GPIO_PIN_1
#define TRIG2_PIN GPIO_PIN_2
#define ECHO_PIN GPIO_PIN_13
#define TRIG_PIN GPIO_PIN_10
#define ULTRASONIC_PORT GPIOD
#define ULTRASONIC_PORT2 GPIOA

#define MODE_BUTTON_PIN GPIO_PIN_2      // Botón para seleccionar el modo
#define MODE_BUTTON_PORT GPIOB

#define MODE2_BUTTON_PIN GPIO_PIN_7     // Botón para seleccionar el modo
#define MODE2_BUTTON_PORT GPIOE

#define INICIO_PIN GPIO_PIN_8     // Botón para seleccionar el modo
#define INICIO_PORT GPIOE

#define POTENTIOMETER_PIN GPIO_PIN_3    // Potenciómetro conectado al ADC1
#define POTENTIOMETER_PORT GPIOA

#define TEMP_MIN 40.0f
#define TEMP_MAX 60.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t previousTime = 0;
uint32_t interval = 500;  // Intervalo de parpadeo en milisegundos
volatile uint8_t emergencyFlag = 0;
volatile uint8_t selectedMode = 0;      // 1: Económico, 2: Ajuste de temperatura
float currentTemperature = 0.0f;       // Temperatura ajustada en modo 2
float selectedTemperature = 0.0;
int isTemperatureSet = 0;  // Bandera para indicar si la temperatura está fijad

int modo = 0;
int leido=0;
int flag=0;
int inicio=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
DHT_DataTypedef DHT22_Data; //Estructura que guarda la temeperatura y la humedad
float Humidity; //Variable para guardar la humedad
float Temperature; //Variable para guardar la temperatura

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void delayMicroseconds(uint16_t microseconds);

void delayMicroseconds(uint16_t microseconds) {
    __HAL_TIM_SET_COUNTER(&htim4, 0); // Reinicia el contador
    while (__HAL_TIM_GET_COUNTER(&htim4) < microseconds); // Espera
}

/* USER CODE END 0 */

// Interrupción GPIO
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == EMERGENCY_BUTTON_PIN) {
        emergencyFlag = 1; // Activar la bandera de emergencia
    }
    else{
    	if (GPIO_Pin == MODE_BUTTON_PIN){
    		modo =1;
    	}
    	else if (GPIO_Pin == MODE2_BUTTON_PIN){
    		modo =2;
    	}
    	else if (GPIO_Pin==INICIO_PIN){
    		inicio=1;
    	}
    	}
    }

float convertirTemperatura(uint32_t lecturaADC, float Tmin, float Tmax) {
    return Tmin + ((float)lecturaADC / 4095.0f) * (Tmax - Tmin);
}
float readTemperature(void) {
    uint32_t adcValue = 0;

    // Leer el ADC (potenciómetro)
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        adcValue = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    // Convertir a temperatura (30°C a 60°C)
    return convertirTemperatura(adcValue, 30.0f, 60.0f);
}


float readUltrasonicDistance1(void) {
    // Emitir pulso TRIG
	    uint32_t startTime = 0, endTime = 0;
	    HAL_TIM_Base_Start(&htim4);
	    // Genera el pulso TRIG (10µs)
	    HAL_GPIO_WritePin(ULTRASONIC_PORT, TRIG_PIN, GPIO_PIN_SET);
	    delayMicroseconds(10);
	    HAL_GPIO_WritePin(ULTRASONIC_PORT, TRIG_PIN, GPIO_PIN_RESET);

	    // Espera el inicio del pulso ECHO
	    while (HAL_GPIO_ReadPin(ULTRASONIC_PORT, ECHO_PIN) == GPIO_PIN_RESET);

	    startTime = __HAL_TIM_GET_COUNTER(&htim4); // Marca el tiempo inicial

	    // Espera el final del pulso ECHO
	    while (HAL_GPIO_ReadPin(ULTRASONIC_PORT, ECHO_PIN) == GPIO_PIN_SET);

	    endTime = __HAL_TIM_GET_COUNTER(&htim4); // Marca el tiempo final

	    // Calcula la duración del pulso (en microsegundos)
	    uint32_t duration = endTime - startTime;

	    // Calcula la distancia en cm (velocidad del sonido: 34300 cm/s)
	    return (duration * 0.0343) / 2;
}
float readUltrasonicDistance2(void) {
    // Emitir pulso TRIG
	    uint32_t startTime = 0, endTime = 0;
	    HAL_TIM_Base_Start(&htim5);
	    // Genera el pulso TRIG (10µs)
	    HAL_GPIO_WritePin(ULTRASONIC_PORT2, TRIG2_PIN, GPIO_PIN_SET);
	    delayMicroseconds(10);
	    HAL_GPIO_WritePin(ULTRASONIC_PORT2, TRIG2_PIN, GPIO_PIN_RESET);

	    // Espera el inicio del pulso ECHO
	    while (HAL_GPIO_ReadPin(ULTRASONIC_PORT2, ECHO2_PIN) == GPIO_PIN_RESET);

	    startTime = __HAL_TIM_GET_COUNTER(&htim5); // Marca el tiempo inicial

	    // Espera el final del pulso ECHO
	    while (HAL_GPIO_ReadPin(ULTRASONIC_PORT2, ECHO2_PIN) == GPIO_PIN_SET);

	    endTime = __HAL_TIM_GET_COUNTER(&htim5); // Marca el tiempo final

	    // Calcula la duración del pulso (en microsegundos)
	    uint32_t duration = endTime - startTime;

	    // Calcula la distancia en cm (velocidad del sonido: 34300 cm/s)
	    return (duration * 0.0343) / 2;
}

void moveMotor1(void) {
	   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
	     HAL_Delay(5000);  // Tiempo para subir la barrera
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2000);
}

void moveMotor2(void) {
	   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);
	     HAL_Delay(5000);  // Tiempo para subir la barrera
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
}

void toggleLEDs(int presenceDetected) {
    if (presenceDetected) {
        HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET); // Apagar LED rojo
        HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET); // Encender LED verde
    } else {
        HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);   // Encender LED rojo
        HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET); // Apagar LED verde
    }
}
void handleEmergency(void) {
    // Parpadeo de ambos LEDs (rojo y verde) en el estado de emergencia
	 lcd_clear();
	 lcd_put_cur(0,0);
     lcd_send_string("PAUSA DE");
     lcd_send_string("EMERGENCIA ");
	 while (1) {
	        // Parpadeo de los LED
	        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14); // LED verde
	        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // LED rojo
	        HAL_Delay(500);

	        // Verificar si el botón de rearme (PA0) fue presionado
	       if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
	    	    inicio=0;
	            currentState2 = ESPERANDO;
	            emergencyFlag = 0; // Restablecer la bandera de emergencia
	           break; // Salir de la función de emergencia
	        }
	    }
}

void process() {
    toggleLEDs(1);

    float distance = 0.0;  // Para el ultrasonido 1
    float distance2 = 0.0; // Para el ultrasonido 2

    // Manejo de estados
    switch (currentState) {
        case WAITING_BUTTON:
        	lcd_clear();
        	lcd_put_cur(0,0);
        	lcd_send_string("PULSE PARA INICIO");
            // Verificar la bandera de emergencia
            if (emergencyFlag) return;

            distance = readUltrasonicDistance1();

            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET) {
                if (distance > 10.0) {
                	lcd_clear();
                	lcd_put_cur(0,0);
                    lcd_send_string("SUBIENDO BARRERA");
                    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Iniciar motor 2
                    moveMotor1();
                    currentState = lavando; // Cambiar al estado de lavado
                }
            }
            break;

        case lavando:
        	if (emergencyFlag) return;
        	lcd_clear();
        	lcd_put_cur(0,0);
        	lcd_send_string("Esperando Coche");
            distance = readUltrasonicDistance1();
            // Verificar la bandera de emergencia

            if (distance < 10.0) {
            	 lcd_clear();
            	 lcd_put_cur(0,0);
            	 lcd_send_string("<LAVANDO>");
                for (int i = 0; i < 5; i++) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
                    HAL_Delay(1000); // Simula tiempo de lavado
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
                    HAL_Delay(1000);
                    // Verificar la bandera de emergencia en cada iteración
                    if (emergencyFlag) return;
                }
                lcd_send_string("<FIN DEL LAVADO >");
                currentState = MOVING_MOTOR_2; // Cambiar al estado de mover motor 2
            }
            break;

        case MOVING_MOTOR_2:
            // Verificar la bandera de emergencia
            if (emergencyFlag) return;

            distance2 = readUltrasonicDistance2();
            if (distance2 < 7.0) {
            	lcd_clear();
            	lcd_put_cur(0,0);
                lcd_send_string("<BAJANDO BARRERA ⬇️>");
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Iniciar motor 2
                moveMotor2();                             // Mover motor 2
                toggleLEDs(1);                            // Encender LED rojo
                currentState = WAITING_BUTTON;
                leido=0;// Volver al estado inicial
                flag=0;
                modo=0;

            }
            break;

        default:
            currentState = WAITING_BUTTON; // Reiniciar estado por seguridad
            break;
    }
}
void process2() {
    toggleLEDs(1);

    float distance = 0.0;  // Para el ultrasonido 1
    float distance2 = 0.0; // Para el ultrasonido 2

    // Manejo de estados
    switch (currentState) {
        case WAITING_BUTTON:
        	lcd_clear();
        	lcd_put_cur(0,0);
        	lcd_send_string("PULSE PARA INICIO");
            // Verificar la bandera de emergencia
            if (emergencyFlag) return;

            distance = readUltrasonicDistance1();

            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET) {
                if (distance > 10.0) {
                	lcd_clear();
                	lcd_put_cur(0,0);
                    lcd_send_string("SUBIENDO BARRERA");
                    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Iniciar motor 2
                    moveMotor1();
                    currentState = lavando; // Cambiar al estado de lavado
                }
            }
            break;

        case lavando:
            // Verificar la bandera de emergencia
            if (emergencyFlag) return;
            if (distance < 10.0) {
            	 lcd_clear();
            	 lcd_put_cur(0,0);
            	 lcd_send_string("<LAVANDO>");
                for (int i = 0; i < 5; i++) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
                    HAL_Delay(1000); // Simula tiempo de lavado
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
                    HAL_Delay(1000);
                    // Verificar la bandera de emergencia en cada iteración
                    if (emergencyFlag) return;
                }
                lcd_send_string("<FIN DEL LAVADO >");
                currentState = MOVING_MOTOR_2; // Cambiar al estado de mover motor 2
            }
            break;

        case MOVING_MOTOR_2:
            // Verificar la bandera de emergencia
            if (emergencyFlag) return;

            distance2 = readUltrasonicDistance2();
            if (distance2 < 7.0) {
            	lcd_clear();
            	lcd_put_cur(0,0);
                lcd_send_string("<BAJANDO BARRERA ⬇️>");
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Iniciar motor 2
                moveMotor2();                             // Mover motor 2
                toggleLEDs(1);                            // Encender LED rojo
                currentState = WAITING_BUTTON;            // Volver al estado inicial
                leido=0;// Volver al estado inicial
                flag=0;
                modo=0;
            }
            break;

        default:
            currentState = WAITING_BUTTON; // Reiniciar estado por seguridad
            break;
    }
}
int seleccionaTemp(void) {

while(1){

        // Leer el valor del potenciómetro y convertirlo a temperatura
        currentTemperature = readTemperature();

        // Mostrar la temperatura en la pantalla LCD
        lcd_clear();
        lcd_put_cur(1,0);
        lcd_send_string("TEMP. SELEC:");
        lcd_put_cur(0, 0);
        Display_Temp(currentTemperature*10 , 0);



        // Verificar si se presionó el botón de inicio para confirmar la selección
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET) {

            selectedTemperature = currentTemperature; // Guardar la temperatura seleccionada
            // Marcar que la temperatura ha sido fijada
            HAL_Delay(500); // Pequeño debounce

            // Mensaje de confirmación
            lcd_clear();
            lcd_put_cur(0, 0);
            lcd_send_string("TEMP. FIJADA:");
            lcd_put_cur(1, 0);
            HAL_Delay(2000);
            Display_Temp(currentTemperature*10 , 0);
            HAL_Delay(2000);
            return 1;
            break;

            // Pasar al proceso principal
        }

        HAL_Delay(500); // Actualización cada 500ms

}
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("<<<<INICIANDO>>>>");
  HAL_Delay(2000);



  while (1) {
 if (emergencyFlag) {
	 handleEmergency();
 }
 else{
    switch (currentState2) {
      case ESPERANDO:

        lcd_clear();
        lcd_put_cur(0, 0);
        lcd_send_string("Pulse para inicio:");
        HAL_Delay(2000);

        if(inicio == 1){//if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET) {

          lcd_clear();
          lcd_put_cur(0, 0);
          lcd_send_string("INICIANDO...");
          HAL_Delay(2000);
          currentState2 = EJECUTANDO;
        }
        break;

      case EJECUTANDO:
        //if (modo == 1) {

          if (flag ==0){
          lcd_clear();
          lcd_put_cur(0, 0);
        	            lcd_send_string("Seleccione");
        	            lcd_put_cur(1, 0);
        	            lcd_send_string("temperatura");
        	            HAL_Delay(2000);

          leido = seleccionaTemp();  // Llamar a la función para seleccionar temperatura
          flag=1;
          }
          else{
        	 lcd_clear();
        	 lcd_put_cur(0, 0);
        	 lcd_send_string("Seleccione Modo");


          if (modo==1){
        	  lcd_clear();
        	   lcd_put_cur(0, 0);
        	   lcd_send_string("Modo 1 ");
        	   HAL_Delay(2000);

          while (leido == 1) {          // Si la temperatura ha sido fijada
            process();
          }// Ejecutar el proceso del sistema
             // Permanecer en el estado EJECUTANDO
          }
          else if (modo == 2){
        	  lcd_clear();
        	         	   lcd_put_cur(0, 0);
        	         	   lcd_send_string("Modo 2 ");
        	         	   HAL_Delay(2000);
        	while (leido == 1) {          // Si la temperatura ha sido fijada
        	            process();
          }
          }
          currentState2 = EJECUTANDO;

          }
       /* } else {
          lcd_clear();
          lcd_put_cur(0, 0);
          lcd_send_string("Modo invalido");
          HAL_Delay(2000);
          currentState2 = ESPERANDO;  // Volver al estado ESPERANDO en caso de error
        }*/
        break;

      default:
        lcd_clear();
        lcd_put_cur(0, 0);
        lcd_send_string("Reiniciando...");
        HAL_Delay(2000);
        currentState2 = ESPERANDO;  // Reiniciar al estado ESPERANDO por seguridad
        break;
    }
  }
  }
}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 100-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_ROJO_Pin|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_2_GPIO_Port, TRIG_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_1_GPIO_Port, TRIG_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 MODO_2_Pin INICIOE8_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|MODO_2_Pin|INICIOE8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_ROJO_Pin PC14 */
  GPIO_InitStruct.Pin = LED_ROJO_Pin|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : INICIO_Pin */
  GPIO_InitStruct.Pin = INICIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INICIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_2_Pin */
  GPIO_InitStruct.Pin = TRIG_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MODO_1_Pin */
  GPIO_InitStruct.Pin = MODO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MODO_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_1_Pin */
  GPIO_InitStruct.Pin = TRIG_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
