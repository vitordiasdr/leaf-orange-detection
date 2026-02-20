/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : H723 + OV7725 (DCMI/DMA) + ST7735 + X-CUBE-AI (U8 96x96x3)
  ******************************************************************************
  * Notas:
  * - Buffers do DCMI em SRAM D2 (NON-cache) -> MPU D2 >= 128KB @ 0x3000_0000.
  * - Buffers da IA (activations/in/out) em DTCM (sem cache).
  * - Modelo (network.h): input U8 96x96x3, output U8 com 5 classes.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

#include <string.h>
#include <stdio.h>


/* USER CODE BEGIN Includes */
#include "board.h"
#include "lcd.h"
#include "camera.h"

/* X-CUBE-AI */
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef TFT96
  /* QQVGA (160x120) */
  #define FrameWidth   160
  #define FrameHeight  120
#elif defined(TFT18)
  /* QQVGA2 (128x160) - ST7735 1.8" */
  #define FrameWidth   128
  #define FrameHeight  160
#else
  /* Default para ST7735 1.8" (128x160) */
  #define FrameWidth   128
  #define FrameHeight  160
#endif

#define FRAME_PIXELS    (FrameWidth * FrameHeight)
#define TARGET_FPS      60
#define CAMERA_FPS_MAX  75
#define FRAME_SKIP      (CAMERA_FPS_MAX / TARGET_FPS)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* --- DCMI/LCD --- */
/* Buffers de frame em SRAM D2 (NON-cache). Requer .ld com .RAM_D2 mapeado para 0x30000000 */
__attribute__((section(".RAM_D2"))) __attribute__((aligned(32)))
static uint16_t frame_dma[FRAME_PIXELS];  /* destino do DMA do DCMI */
__attribute__((section(".RAM_D2"))) __attribute__((aligned(32)))
static uint16_t frame_lcd[FRAME_PIXELS];  /* cópia estável para o LCD */

volatile uint8_t  frame_ready = 0;
volatile uint32_t DCMI_FrameIsReady = 0;
static   uint32_t Camera_FPS = 0;

/* --- X-CUBE-AI --- */
/* Sua rede é quantizada U8: in/out são U8 conforme network.h */
AI_ALIGNED(4) __attribute__((section(".DtcmRam")))
static ai_u8 ai_in[AI_NETWORK_IN_1_SIZE_BYTES];       /* 96*96*3 = 27648 */
AI_ALIGNED(4) __attribute__((section(".DtcmRam")))
static ai_u8 ai_out[AI_NETWORK_OUT_1_SIZE_BYTES];     /* 5 */
AI_ALIGNED(4) __attribute__((section(".DtcmRam")))
static ai_u8 ai_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

/* Instância da rede e buffers ai_buffer (padrão X-CUBE-AI) */
static ai_handle s_network = AI_HANDLE_NULL;

static ai_buffer s_ai_input[AI_NETWORK_IN_NUM];
static ai_buffer s_ai_output[AI_NETWORK_OUT_NUM];



/* Rótulos das classes (ajuste conforme seu dataset) */
static const char* kClasses[5] = {
  "Canker_Diseases", "Nutrient_Def_Yellow", "Healthy", "Multiple_Diseases", "Young_Healthy"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);

static int  ai_init(void);
static void preprocess_rgb565_to_u8(const uint16_t* src, int src_w, int src_h, ai_u8* dst);
static int  argmax_u8(const ai_u8* v, int n, ai_u8* pmax);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Habilita caches (I + D). Deixe D2 NON-cache via MPU para o DMA do DCMI. */
static void CPU_CACHE_Enable(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();
}

/* MPU: mapeia DTCM e D2. MUITO IMPORTANTE: D2 >= 128KB! */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();

  /* Region 0: DTCM (0x2000_0000) - usar como non-cache */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress      = 0x20000000; /* DTCM base */
  MPU_InitStruct.Size             = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Region 1: SRAM D2 (0x3000_0000) - NON-cache para DCMI/DMA */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress      = 0x30000000; /* D2 SRAM base */
  MPU_InitStruct.Size             = MPU_REGION_SIZE_128KB; /* >>> PRECISA >= 128KB para 2 frames */
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* DCMI -> FPS e sinaliza frame completo */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
  static uint32_t tick = 0, frames = 0;
  UNUSED(hdcmi);

  frames++;
  if (HAL_GetTick() - tick >= 1000) {
    tick = HAL_GetTick();
    Camera_FPS = frames;
    frames = 0;
  }
  frame_ready = 1;
}

/* IA: create -> init (weights/activations) -> link I/O */

static int ai_init(void)
{
  __HAL_RCC_CRC_CLK_ENABLE(); /* recomendado em H7 antes de create/init */

  ai_error err = ai_network_create(&s_network, AI_NETWORK_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) return -1;

  /* use o macro oficial para evitar warning de chaves */
  ai_network_params params = AI_NETWORK_PARAMS_INIT(
      AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
      AI_NETWORK_DATA_ACTIVATIONS(ai_activations)
  );

  if (!ai_network_init(s_network, &params)) {
    err = ai_network_get_error(s_network);
    return -2;
  }

  /* >>> AQUI: pegue os descritores da rede e ligue aos seus vetores <<< */
  ai_buffer* in_desc  = ai_network_inputs_get(s_network, NULL);
  ai_buffer* out_desc = ai_network_outputs_get(s_network, NULL);

  /* Clona formato/tamanho/shape para s_ai_input/s_ai_output */
  memcpy(&s_ai_input[0],  &in_desc[0],  sizeof(ai_buffer));
  memcpy(&s_ai_output[0], &out_desc[0], sizeof(ai_buffer));

  /* Agora só troque o ponteiro de dados pelos seus buffers U8 */
  s_ai_input[0].data  = AI_HANDLE_PTR(ai_in);
  s_ai_output[0].data = AI_HANDLE_PTR(ai_out);

  return 0;
}


/* Pré-processa RGB565 128x160 -> U8 96x96x3 intercalado (R,G,B) */
static void preprocess_rgb565_to_u8(const uint16_t* src, int src_w, int src_h, ai_u8* dst)
{
  const int dst_w = AI_NETWORK_IN_1_WIDTH;   /* 96 */
  const int dst_h = AI_NETWORK_IN_1_HEIGHT;  /* 96 */
  const int ch    = AI_NETWORK_IN_1_CHANNEL; /* 3 */

  for (int y = 0; y < dst_h; ++y) {
    int sy = (y * src_h) / dst_h;
    for (int x = 0; x < dst_w; ++x) {
      int sx = (x * src_w) / dst_w;
      uint16_t p = src[sy * src_w + sx];

      uint8_t r5 = (p >> 11) & 0x1F;
      uint8_t g6 = (p >> 5)  & 0x3F;
      uint8_t b5 =  p        & 0x1F;

      uint8_t r = (r5 << 3) | (r5 >> 2);
      uint8_t g = (g6 << 2) | (g6 >> 4);
      uint8_t b = (b5 << 3) | (b5 >> 2);

      if (ch == 3) { *dst++ = r; *dst++ = g; *dst++ = b; }
      else {
        uint8_t gray = (uint8_t)(0.299f*r + 0.587f*g + 0.114f*b);
        *dst++ = gray;
      }
    }
  }
}

/* Argmax para vetor U8 (retorna índice e opcionalmente o valor máximo) */
static int argmax_u8(const ai_u8* v, int n, ai_u8* pmax)
{
  int im = 0; ai_u8 m = v[0];
  for (int i = 1; i < n; ++i) {
    if (v[i] > m) { m = v[i]; im = i; }
  }
  if (pmax) *pmax = m;
  return im;
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  MPU_Config();
  CPU_CACHE_Enable();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  board_button_init();
  board_led_init();

  /* LCD de teste / splash */
  LCD_Test();

  	uint8_t texto[30];

    sprintf((char *)&texto, "Camera Not Found");
    LCD_ShowString(0, 58, ST7735Ctx.Width, 16, 16, texto);

    #ifdef TFT96
  	Camera_Init_Device(&hi2c1, FRAMESIZE_QQVGA);
  	#elif TFT18
  	Camera_Init_Device(&hi2c1, FRAMESIZE_QQVGA2);
  	#endif
  	//clean Ypos 58
  	ST7735_LCD_Driver.FillRect(&st7735_pObj, 0, 58, ST7735Ctx.Width, 16, BLACK);

    while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
      sprintf((char *)&texto, "Camera id:0x%x", hcamera.device_id);
      LCD_ShowString(4, 58, ST7735Ctx.Width, 16, 12, texto);

  		HAL_Delay(500);

      sprintf((char *)&texto, "LongPress K1 to Run");
      LCD_ShowString(4, 58, ST7735Ctx.Width, 16, 12, texto);

  		HAL_Delay(500);
    }

    HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS,
                       (uint32_t)frame_dma,
                       FRAME_PIXELS / 2);
  /* Inicializa IA */
  if (ai_init() != 0) {
    LCD_ShowString(2, 2, ST7735Ctx.Width, 16, 12, (uint8_t*)"AI init error");
  }

  /* Buffer de texto */
  char text[48] = {0};

  /* Controle de Throttle de FPS para IA */
  static uint8_t skip = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (frame_ready)
    {
      frame_ready = 0;

      /* Throttle opcional: pule frames para manter CPU folgada */
      skip++;
      if (skip < FRAME_SKIP) {
        /* ainda assim atualize a tela com o último frame estável */
        memcpy(frame_lcd, frame_dma, FRAME_PIXELS * 2);
        ST7735_FillRGBRect(&st7735_pObj, 0, 0,
                           (uint8_t*)frame_lcd, ST7735Ctx.Width, ST7735Ctx.Height);
        continue;
      }
      skip = 0;

      /* 1) Copia para frame_lcd (estável) e desenha no display */
      memcpy(frame_lcd, frame_dma, FRAME_PIXELS * 2);
      ST7735_FillRGBRect(&st7735_pObj, 0, 0,
                         (uint8_t*)frame_lcd, ST7735Ctx.Width, ST7735Ctx.Height);

      /* 2) Pré-processa 128x160 -> 96x96x3 U8 */
      preprocess_rgb565_to_u8(frame_lcd, FrameWidth, FrameHeight, ai_in);

      /* 3) Inferência: ai_network_run() com buffers ligados no ai_init() */
      s_ai_input[0].data  = AI_HANDLE_PTR(ai_in);
      s_ai_output[0].data = AI_HANDLE_PTR(ai_out);

      ai_i32 nb = ai_network_run(s_network, s_ai_input, s_ai_output);
      if (nb == 1) {
        /* 4) Top-1 da saída U8 (5 canais) e exibe */
        ai_u8 vmax = 0;
        int cls = argmax_u8(ai_out, AI_NETWORK_OUT_1_SIZE, &vmax);

        /* Se quiser em porcentagem: (vmax*100)/255 */
        snprintf(text, sizeof(text), "%s  %u", kClasses[cls], ((unsigned)vmax/255));
        LCD_ShowString(2, 2, ST7735Ctx.Width, 16, 12, (uint8_t*)text);

        /* Mostra FPS da câmera (do callback) */
        snprintf(text, sizeof(text), "%i",cls);
        LCD_ShowString(2, 18, ST7735Ctx.Width, 16, 12, (uint8_t*)text);

        snprintf(text, sizeof(text), "%lu FPS", Camera_FPS);
        LCD_ShowString(2, 34, ST7735Ctx.Width, 16, 12, (uint8_t*)text);
      }

      board_led_toggle();
    }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Mantenha sua configuração existente (gerada pelo Cube) */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /* Exemplo com HSE + PLL. Ajuste conforme seu projeto .ioc */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State     = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 2;
  RCC_OscInitStruct.PLL.PLLN       = 44;
  RCC_OscInitStruct.PLL.PLLP       = 1;
  RCC_OscInitStruct.PLL.PLLQ       = 46;
  RCC_OscInitStruct.PLL.PLLR       = 2;
  RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN   = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                   | RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) Error_Handler();

  /* MCO opcional: HSI48/4 na MCO1 */
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_4);
}

/* USER CODE BEGIN 4 */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    board_led_toggle();
    HAL_Delay(100);
  }
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
