/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx.h"
#include "stm32f103xb.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	int id; //Unique id for all delays
    int startTime;  // Time elapsed since the delay started
    int delayTime;    // Duration of the delay
    int activeFlag;   // Flag to indicate if the delay is active
} delay_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Clock_Frequency 8000 //KHz
#define MAX_DELAYS 10
#define BAUD_RATE 115200
#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 128
#define CAN_BAUD_PRESCALER 9
#define CAN_TS1 13
#define CAN_TS2 2
#define CAN_SJW 1
#define PWM_FREQUENCY 800000 // 800 kHz PWM frequency
#define PWM_DUTY_CYCLE 50  // PWM duty cycle percentage
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
volatile int Globalcounter = 0;
delay_t delays[MAX_DELAYS];
volatile uint8_t txBuffer[TX_BUFFER_SIZE];
volatile uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint8_t txHead = 0;
volatile uint8_t txTail = 0;
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;
volatile uint8_t txBusy = 0;
char UART_MSGBUFFER[TX_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SysTick_Init(uint32_t ticks);
void SysTick_Handler(void);
void LED_Init (void);
void LED_Toggle(void);
void Delay_Init(void);
int Delay_Start(int id, int delayTime);
void Delay_Stop(int id);
int Delay_Completed(int id);
void Delay_ErrorHandler(int delayid);
void UART_Init(void);
void UART_Send(uint8_t *data, uint16_t size);
void UART_Receive(uint8_t *data, uint16_t size);
void USART1_IRQHandler(void);
void CAN_Init(void);
bool CAN_Transmit(uint8_t *data, uint8_t length, uint32_t timeout);
bool CAN_Receive(uint8_t *data, uint32_t timeout);
void CAN_ErrorHandler(bool operationSuccess);
void PWM_Init(void);
void PWM_SetDutyCycle(uint8_t dutyCycle);
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
  SysTick_Init(Clock_Frequency);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  LED_Init();
  Delay_Init();
  UART_Init();
  CAN_Init();
  PWM_Init();
  PWM_SetDutyCycle(30);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int delayId_0 = Delay_Start(0, 5000);
  Delay_ErrorHandler(delayId_0);

  while (1)
  {
      if (Delay_Completed(0))
      {
          LED_Toggle();
          Delay_Start(0, 100);      // Restart the delay
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/* USER CODE BEGIN 4 */
void SysTick_Init(uint32_t ticks)
{

	SysTick->CTRL = 0; // Disable SysTick

	SysTick->LOAD = ticks-1; // Set Reload Register

	// Setting Interrupt Priority to the highest
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1);

	SysTick->VAL = 0; // Reset the SysTick counter value

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // Selecting internal clock source
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Enabling SysTick exception Request when 0


	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable SysTick
}

void SysTick_Handler(void)
{

	if (Globalcounter == 0xffffffff) {
        Globalcounter = 0; // Reset the counter if the maximum value is reached
    } else {
        Globalcounter++; // Increment the counter
    }

}

void LED_Init (void)
{

	// Enable clock for GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

	// Set PC13 as output push-pull
	GPIOC->CRH &= ~GPIO_CRH_CNF13;    // Output Push-Pull
	GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Output mode, max speed 50 MHz
}

void LED_Toggle(void)
{
	GPIOC->ODR ^= GPIO_ODR_ODR13;
}

void Delay_Init(void)
{
    for (int i = 0; i < MAX_DELAYS; i++) {
        delays[i].id = i;               // Initialize each delay with a unique id
        delays[i].startTime = 0;        // Reset the start time
        delays[i].delayTime = 0;        // Reset the delay time
        delays[i].activeFlag = 0;       // Initialize all delays as inactive
    }
}

int Delay_Start(int id, int delayTime)
{
    if (id >= 0 && id < MAX_DELAYS) {
        delays[id].startTime = Globalcounter; // Capture the current Globalcounter value
        delays[id].delayTime = delayTime;
        delays[id].activeFlag = 1;           // Mark this delay as active
        return id;                           // Return the id of the delay
    }
    return -1; // Invalid id
}

void Delay_Stop(int id)
{
    if (id >= 0 && id < MAX_DELAYS) {
        delays[id].activeFlag = 0; // Stop the specified delay
    }
}

int Delay_Completed(int id)
{
    if (id >= 0 && id < MAX_DELAYS) {
          if (delays[id].activeFlag && (Globalcounter >= delays[id].startTime + delays[id].delayTime)) {
            delays[id].activeFlag = 0; // Deactivate the delay after completion
            return 1; // Delay completed
        }
    }
    return 0; // Delay not yet completed
}

void Delay_ErrorHandler(int delayid)
{
	if (delayid == (-1))
	{
		char errormsg[] = "Invalid Delay Id chosen....\n";
		UART_Send((uint8_t*)errormsg, strlen(errormsg));
	}
}

void UART_Init(void)
{
    // Enable clock for GPIOA and USART1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // USART1 clock

    // Configure PA9 (TX) as Alternate function push-pull
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1); // Output mode, max speed 2 MHz

    // Configure PA10 (RX) as input floating
    GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
    GPIOA->CRH |= GPIO_CRH_CNF10_0; // Input floating

    // Configure baud rate
    USART1->BRR = (Clock_Frequency*1000) / BAUD_RATE;

    // Enable USART, TX, RX and RXNE interrupt
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    // Enable USART1 global interrupt
    NVIC_EnableIRQ(USART1_IRQn);
}

void UART_Send(uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) {
        while (((txHead + 1) % TX_BUFFER_SIZE) == txTail); // Wait if buffer is full
        txBuffer[txHead] = data[i];
        txHead = (txHead + 1) % TX_BUFFER_SIZE;
    }

    // Enable TXE interrupt to start transmitting
    if (!txBusy) {
        txBusy = 1;
        USART1->CR1 |= USART_CR1_TXEIE;
    }
}

void UART_Receive(uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) {
        while (rxHead == rxTail); // Wait until data is received
        data[i] = rxBuffer[rxTail];
        rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
    }
}

void USART1_IRQHandler(void)
{
    // Check if data is ready to be read
    if (USART1->SR & USART_SR_RXNE) {
        uint8_t data = USART1->DR; // Read received data
        uint8_t nextHead = (rxHead + 1) % RX_BUFFER_SIZE;
        if (nextHead != rxTail) { // If buffer is not full
            rxBuffer[rxHead] = data;
            rxHead = nextHead;
        }
    }

    // Check if transmit data register is empty
    if (USART1->SR & USART_SR_TXE) {
        if (txTail != txHead) { // If data is available to transmit
            USART1->DR = txBuffer[txTail];
            txTail = (txTail + 1) % TX_BUFFER_SIZE;
        } else {
            // No more data to send, disable TXE interrupt
            USART1->CR1 &= ~USART_CR1_TXEIE;
            txBusy = 0;
        }
    }
}

void CAN_Init(void)
{
    /* Enable clock for GPIOB and CAN */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  // Enable GPIOB clock
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;  // Enable CAN clock

    /* Configure PB8 as CAN_RX (input floating) */
    GPIOB->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);
    GPIOB->CRH |= GPIO_CRH_CNF8_0;

    /* Configure PB9 as CAN_TX (alternate function push-pull) */
    GPIOB->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOB->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0;

    /* Enter initialization mode */
    CAN1->MCR = CAN_MCR_INRQ;
    while (!(CAN1->MSR & CAN_MSR_INAK)); // Wait until initialization mode is entered

    /* Set CAN bit timing */
    CAN1->BTR = (CAN_SJW - 1) << 24 |
                (CAN_TS1 - 1) << 16 |
                (CAN_TS2 - 1) << 20 |
                (CAN_BAUD_PRESCALER - 1);

    /* Set CAN filters */
    CAN1->FMR |= CAN_FMR_FINIT;  // Enter filter initialization mode
    CAN1->FA1R &= ~CAN_FA1R_FACT; // Deactivate all filters

    /* Filter 0: accept all standard IDs */
    CAN1->FS1R |= CAN_FS1R_FSC;  // Single 32-bit scale configuration
    CAN1->FM1R &= ~CAN_FM1R_FBM; // Identifier mask mode
    CAN1->sFilterRegister[0].FR1 = 0x00000000; // Filter ID
    CAN1->sFilterRegister[0].FR2 = 0x00000000; // Filter mask
    CAN1->FA1R |= CAN_FA1R_FACT;  // Enable filter 0

    CAN1->FMR &= ~CAN_FMR_FINIT;  // Leave filter initialization mode

    /* Leave initialization mode, enter normal mode */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while (CAN1->MSR & CAN_MSR_INAK); // Wait until normal mode is entered

    // Check CAN mode status and send status over UART
    if (CAN1->MSR & CAN_MSR_INAK)
    {
        UART_Send((uint8_t *)"CAN is still in Initialization mode\r\n", 38);
    }
    else if (CAN1->MSR & CAN_MSR_SLAK)
    {
        UART_Send((uint8_t *)"CAN is in Sleep mode\r\n", 23);
    }
    else
    {
        UART_Send((uint8_t *)"CAN is in Normal mode\r\n", 24);
    }
}

bool CAN_Transmit(uint8_t *data, uint8_t length, uint32_t timeout)
{
    uint32_t startTime = 0;

    // Check that the length is valid (should be between 1 and 8 bytes)
    if (length == 0 || length > 8)
    {
        return false;  // Invalid data length
    }

    // Wait for an empty transmit mailbox
    while (!(CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)))
    {
        if (startTime++ >= timeout)
        {
            return false;  // Timeout
        }
    }

    // Select an empty mailbox (lowest numbered mailbox first)
    uint32_t mailbox = 0;
    if (CAN1->TSR & CAN_TSR_TME0)
    {
        mailbox = 0;
    }
    else if (CAN1->TSR & CAN_TSR_TME1)
    {
        mailbox = 1;
    }
    else if (CAN1->TSR & CAN_TSR_TME2)
    {
        mailbox = 2;
    }

    // Set up the mailbox with the data to be sent
    CAN1->sTxMailBox[mailbox].TIR = 0;  // Standard Identifier, no RTR, no Extended ID
    CAN1->sTxMailBox[mailbox].TDTR = (length & 0x0F);  // Set the data length code (DLC)

    // Copy data into the mailbox
    CAN1->sTxMailBox[mailbox].TDLR = ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[0];
    CAN1->sTxMailBox[mailbox].TDHR = ((uint32_t)data[7] << 24) | ((uint32_t)data[6] << 16) | ((uint32_t)data[5] << 8) | (uint32_t)data[4];

    // Request transmission
    CAN1->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;

    // Wait for transmission to complete or timeout
    startTime = 0;
    while (!(CAN1->TSR & (CAN_TSR_TXOK0 << (mailbox * 8))))
    {
        if (CAN1->TSR & (CAN_TSR_TERR0 << (mailbox * 8)) || CAN1->TSR & (CAN_TSR_ALST0 << (mailbox * 8)))
        {
            return false;  // Transmission error or arbitration lost
        }

        if (startTime++ >= timeout)
        {
            return false;  // Timeout
        }
    }

    return true;  // Transmission successful
}

bool CAN_Receive(uint8_t *data, uint32_t timeout)
{
    uint32_t startTime = 0;

    // Wait until a message is received in FIFO 0
    while (!(CAN1->RF0R & CAN_RF0R_FMP0))
    {
        if (startTime++ >= timeout)
        {
            return false;  // Timeout
        }
    }

    // Read the received message from the FIFO
    uint32_t receivedLow = CAN1->sFIFOMailBox[0].RDLR;
    uint32_t receivedHigh = CAN1->sFIFOMailBox[0].RDHR;

    data[0] = (uint8_t)(receivedLow & 0xFF);
    data[1] = (uint8_t)((receivedLow >> 8) & 0xFF);
    data[2] = (uint8_t)((receivedLow >> 16) & 0xFF);
    data[3] = (uint8_t)((receivedLow >> 24) & 0xFF);
    data[4] = (uint8_t)(receivedHigh & 0xFF);
    data[5] = (uint8_t)((receivedHigh >> 8) & 0xFF);
    data[6] = (uint8_t)((receivedHigh >> 16) & 0xFF);
    data[7] = (uint8_t)((receivedHigh >> 24) & 0xFF);

    // Release the FIFO
    CAN1->RF0R |= CAN_RF0R_RFOM0;

    return true;  // Reception successful
}

void CAN_ErrorHandler(bool operationSuccess)
{
    if (operationSuccess)
    {
        UART_Send((uint8_t *)"CAN Operation Successful\r\n", 27);
        return;
    }

    uint32_t esr = CAN1->ESR;  // Read the CAN Error Status Register
    char errorMsg[128];  // Buffer to hold the error message

    if (esr & CAN_ESR_BOFF)
    {
        snprintf(errorMsg, sizeof(errorMsg), "CAN Error: Bus-Off\r\n");
    }
    else if (esr & CAN_ESR_EPVF)
    {
        snprintf(errorMsg, sizeof(errorMsg), "CAN Error: Error Passive\r\n");
    }
    else if (esr & CAN_ESR_EWGF)
    {
        snprintf(errorMsg, sizeof(errorMsg), "CAN Error: Error Warning\r\n");
    }
    else if ((esr & CAN_ESR_LEC) != 0)
    {
        // Last Error Code (LEC) bits indicate the type of error
        switch (esr & CAN_ESR_LEC)
        {
            case (0x01 << 4):
                snprintf(errorMsg, sizeof(errorMsg), "CAN Error: Stuff Error\r\n");
                break;
            case (0x02 << 4):
                snprintf(errorMsg, sizeof(errorMsg), "CAN Error: Form Error\r\n");
                break;
            case (0x03 << 4):
                snprintf(errorMsg, sizeof(errorMsg), "CAN Error: Acknowledgment Error\r\n");
                break;
            case (0x04 << 4):
                snprintf(errorMsg, sizeof(errorMsg), "CAN Error: Bit Recessive Error\r\n");
                break;
            case (0x05 << 4):
                snprintf(errorMsg, sizeof(errorMsg), "CAN Error: Bit Dominant Error\r\n");
                break;
            case (0x06 << 4):
                snprintf(errorMsg, sizeof(errorMsg), "CAN Error: CRC Error\r\n");
                break;
            default:
                snprintf(errorMsg, sizeof(errorMsg), "CAN Error: Unknown Error\r\n");
                break;
        }
    }
    else
    {
        snprintf(errorMsg, sizeof(errorMsg), "CAN Error: Unknown Status\r\n");
    }

    UART_Send((uint8_t *)errorMsg, strlen(errorMsg));  // Send the error message over UART
}

void PWM_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // Enable GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Enable TIM2 clock

    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
    GPIOA->CRL |= (GPIO_CRL_CNF0_1 | GPIO_CRL_MODE0_1); // AF push-pull, 10 MHz output

    uint32_t timerClock = Clock_Frequency * 1000; // Timer clock in Hz
    uint32_t period = (timerClock / PWM_FREQUENCY) - 1; // Calculate period for 800 kHz
    TIM2->PSC = 0; // Prescaler set to 0
    TIM2->ARR = period;

    TIM2->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // PWM mode 1: OC1M = 110
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;  // Enable preload register on TIM2_CH1
    TIM2->CCR1 = period / 2; // Set the initial duty cycle to 50%

    TIM2->CCER |= TIM_CCER_CC1E; // Enable TIM2_CH1 output
    TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2 counter
    TIM2->EGR |= TIM_EGR_UG; // Force update generation
}

void PWM_SetDutyCycle(uint8_t dutyCycle)
{
    uint32_t period = TIM2->ARR;
    uint32_t pulse = (period * dutyCycle) / 100; // Calculate pulse length
    TIM2->CCR1 = pulse; // Set the duty cycle
}

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
