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
#define CAN_BAUD_PRESCALER 2
#define CAN_TS1 13
#define CAN_TS2 2
#define CAN_SJW 1
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
int CAN_Transmit(uint32_t stdId, uint8_t *data, uint8_t len);
int CAN_Receive(uint8_t *data, uint8_t *len);
void CAN_ErrorHandler(void);
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int delayId_0 = Delay_Start(0, 500);
  Delay_ErrorHandler(delayId_0);

  char startmsg[] = "Starting UART....\n";
  UART_Send((uint8_t*)startmsg, strlen(startmsg));

  /* Buffer to store received data and formatted message for uart */
//  uint8_t receivedData[RX_BUFFER_SIZE];
//  char messageBuffer[RX_BUFFER_SIZE];

  uint8_t txData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  uint8_t rxData[8];
  uint8_t len;

  while (1)
  {
      if (Delay_Completed(0))
      {
          LED_Toggle();
          Delay_Start(0, 500);      // Restart the delay
      }

      // Transmit a CAN message
      if (CAN_Transmit(0x123, txData, 8) == 0)
      {
          // Transmission was successful
          snprintf(UART_MSGBUFFER, sizeof(UART_MSGBUFFER), "CAN Transmit Successful\r\n");
          UART_Send((uint8_t*)UART_MSGBUFFER, strlen(UART_MSGBUFFER));
      }
      else
      {
          // Transmission failed
          CAN_ErrorHandler();
      }

      // Receive a CAN message
      if (CAN_Receive(rxData, &len) == 0)
      {
          // Process received message
          snprintf(UART_MSGBUFFER, sizeof(UART_MSGBUFFER), "CAN Receive: ID=0x%03X, Length=%d, Data=", 0x123, len);

          for (uint8_t i = 0; i < len; i++) {
              snprintf(UART_MSGBUFFER + strlen(UART_MSGBUFFER), sizeof(UART_MSGBUFFER) - strlen(UART_MSGBUFFER), " %02X", rxData[i]);
          }

          snprintf(UART_MSGBUFFER + strlen(UART_MSGBUFFER), sizeof(UART_MSGBUFFER) - strlen(UART_MSGBUFFER), "\r\n");
          UART_Send((uint8_t*)UART_MSGBUFFER, strlen(UART_MSGBUFFER));
      }
      else
      {
          // Reception failed or no message available
          CAN_ErrorHandler();
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
	 RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  // GPIOB clock
	 RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;  // CAN clock

	 /* Configure PB8 as CAN_RX (input floating) */
	 GPIOB->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);
	 GPIOB->CRH |= GPIO_CRH_CNF8_0;  // Input floating

	 /* Configure PB9 as CAN_TX (alternate function push-pull) */
	 GPIOB->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
	 GPIOB->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0;  // Alternate function push-pull, 10 MHz

    /* Enter initialization mode */
    CAN1->MCR = CAN_MCR_INRQ;
    while (!(CAN1->MSR & CAN_MSR_INAK)); // Wait until initialization mode is entered

    // Check if the CAN peripheral entered initialization mode
    if (CAN1->MSR & CAN_MSR_INAK) {
        snprintf(UART_MSGBUFFER, sizeof(UART_MSGBUFFER), "CAN Initialization successful\r\n");
        UART_Send((uint8_t*)UART_MSGBUFFER, strlen(UART_MSGBUFFER));
    } else {
        snprintf(UART_MSGBUFFER, sizeof(UART_MSGBUFFER), "CAN Initialization failed\r\n");
        UART_Send((uint8_t*)UART_MSGBUFFER, strlen(UART_MSGBUFFER));
    }

    /* Set CAN bit timing */
    CAN1->BTR = (CAN_SJW - 1) << 24 |
                (CAN_TS1 - 1) << 16 |
                (CAN_TS2 - 1) << 20 |
                (CAN_BAUD_PRESCALER - 1);

    /* Leave initialization mode, enter normal mode */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while (CAN1->MSR & CAN_MSR_INAK); // Wait until normal mode is entered

    // Check if the CAN peripheral entered normal mode
    if (!(CAN1->MSR & CAN_MSR_INAK)) {
        snprintf(UART_MSGBUFFER, sizeof(UART_MSGBUFFER), "CAN Entered Normal mode\r\n");
        UART_Send((uint8_t*)UART_MSGBUFFER, strlen(UART_MSGBUFFER));
    } else {
        snprintf(UART_MSGBUFFER, sizeof(UART_MSGBUFFER), "CAN Failed to enter Normal mode\r\n");
        UART_Send((uint8_t*)UART_MSGBUFFER, strlen(UART_MSGBUFFER));
    }

    /* Set CAN filters to accept all messages */
    CAN1->FMR |= CAN_FMR_FINIT;  // Enter filter initialization mode
    CAN1->FA1R &= ~CAN_FA1R_FACT; // Disable all filters

    /* Filter 0: accept all standard IDs */
    CAN1->FS1R |= CAN_FS1R_FSC;  // Single 32-bit scale configuration
    CAN1->FM1R &= ~CAN_FM1R_FBM; // Identifier mask mode
    CAN1->sFilterRegister[0].FR1 = 0x00000000; // Filter ID
    CAN1->sFilterRegister[0].FR2 = 0x00000000; // Filter mask
    CAN1->FA1R |= CAN_FA1R_FACT;  // Enable filter 0

    CAN1->FMR &= ~CAN_FMR_FINIT;  // Leave filter initialization mode
}

int CAN_Transmit(uint32_t stdId, uint8_t *data, uint8_t len)
{
    uint32_t timeout = 100000;  // Arbitrary timeout value

    // Wait for an empty transmit mailbox
    while (!(CAN1->TSR & CAN_TSR_TME0))
    {
        if (--timeout == 0) {
            snprintf(UART_MSGBUFFER, sizeof(UART_MSGBUFFER), "CAN Transmit Timeout\r\n");
            UART_Send((uint8_t*)UART_MSGBUFFER, strlen(UART_MSGBUFFER));
            return -1;
        }
    }

    // Configure the TX mailbox with the standard ID and data length
    CAN1->sTxMailBox[0].TIR = stdId << 21;
    CAN1->sTxMailBox[0].TDTR = len & 0x0F;
    CAN1->sTxMailBox[0].TDLR = *((uint32_t *)data); // First 4 bytes of data
    if (len > 4) {
        CAN1->sTxMailBox[0].TDHR = *((uint32_t *)(data + 4)); // Remaining data
    }

    // Request transmission
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

    timeout = 100000;  // Reset timeout

    // Wait for transmission complete
    while (!(CAN1->TSR & CAN_TSR_RQCP0))
    {
        if (--timeout == 0) {
            snprintf(UART_MSGBUFFER, sizeof(UART_MSGBUFFER), "CAN Transmit Complete Timeout\r\n");
            UART_Send((uint8_t*)UART_MSGBUFFER, strlen(UART_MSGBUFFER));
            return -1;
        }
    }

    // Clear the transmission request complete flag
    CAN1->TSR |= CAN_TSR_RQCP0;

    // Check if transmission was successful
    if (CAN1->TSR & CAN_TSR_TXOK0) {
        return 0; // Transmission OK
    } else {
        return -1; // Transmission error
    }
}


int CAN_Receive(uint8_t *data, uint8_t *len)
{
    // Check if there is a message in FIFO 0
    if (!(CAN1->RF0R & CAN_RF0R_FMP0)) {
        return -1; // No message available
    }

    // Get the received message length
    *len = CAN1->sFIFOMailBox[0].RDTR & 0x0F;

    // Read the data
    *((uint32_t *)data) = CAN1->sFIFOMailBox[0].RDLR;
    if (*len > 4) {
        *((uint32_t *)(data + 4)) = CAN1->sFIFOMailBox[0].RDHR;
    }

    // Release the FIFO
    CAN1->RF0R |= CAN_RF0R_RFOM0;

    return 0; // Receive OK
}

void CAN_ErrorHandler(void)
{
    char error_msg[64];

    // Read CAN error status register
    uint32_t esr = CAN1->ESR;

    // Check for different error conditions
    if (esr & CAN_ESR_BOFF) {
        snprintf(error_msg, sizeof(error_msg), "CAN Bus-Off Error\r\n");
    } else if (esr & CAN_ESR_EPVF) {
        snprintf(error_msg, sizeof(error_msg), "CAN Error Passive\r\n");
    } else if (esr & CAN_ESR_EWGF) {
        snprintf(error_msg, sizeof(error_msg), "CAN Error Warning\r\n");
    } else if (esr & CAN_ESR_LEC) {
        // Last Error Code (LEC) interpretation
        switch ((esr >> 4) & 0x07) {
            case 0x01: snprintf(error_msg, sizeof(error_msg), "CAN Stuff Error\r\n"); break;
            case 0x02: snprintf(error_msg, sizeof(error_msg), "CAN Form Error\r\n"); break;
            case 0x03: snprintf(error_msg, sizeof(error_msg), "CAN Acknowledge Error\r\n"); break;
            case 0x04: snprintf(error_msg, sizeof(error_msg), "CAN Bit recessive Error\r\n"); break;
            case 0x05: snprintf(error_msg, sizeof(error_msg), "CAN Bit dominant Error\r\n"); break;
            case 0x06: snprintf(error_msg, sizeof(error_msg), "CAN CRC Error\r\n"); break;
            case 0x07: snprintf(error_msg, sizeof(error_msg), "CAN Set by software\r\n"); break;
            default: snprintf(error_msg, sizeof(error_msg), "Unknown CAN Error\r\n"); break;
        }
    } else {
        snprintf(error_msg, sizeof(error_msg), "No CAN Error Detected\r\n");
    }

    // Send error message via UART
    UART_Send((uint8_t*)error_msg, strlen(error_msg));
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
