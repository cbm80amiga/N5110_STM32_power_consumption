/*
  STM32 LowPower modes testing
  (c)2019 Pawel A. Hernik
  YouTube video: 
  https://youtu.be/ThO8t149KmM
*/

// *** CONNECTIONS ***
/*
 N5110 LCD pinout from left:
 #1 RST      - PA0
 #2 CS/CE    - PA4
 #3 DC       - PA1
 #4 MOSI/DIN - PA7
 #5 SCK/CLK  - PA5
 #6 VCC      - 3.3V
 #7 LIGHT    - 200ohm to GND
 #8 GND

 STM32 SPI1 pins:
  PA4 CS1
  PA5 SCK1
  PA6 MISO1
  PA7 MOSI1
*/

#include <libmaple/pwr.h>
#include <libmaple/scb.h>
#include <RTClock.h>
RTClock rtclock(RTCSEL_LSE);

#define N5110_RST       PA0
#define N5110_CS        PA4
#define N5110_DC        PA1

// define USESPI in LCD driver header for HW SPI version
#include "N5110_SPI.h"
#if USESPI==1
#include <SPI.h>
#endif
N5110_SPI lcd(N5110_RST, N5110_CS, N5110_DC); // RST,CS,DC

#include "small4x7_font.h"

void disableClocks() 
{
    rcc_clk_disable(RCC_ADC1);
    rcc_clk_disable(RCC_ADC2);
    rcc_clk_disable(RCC_ADC3);
    rcc_clk_disable(RCC_AFIO);
    rcc_clk_disable(RCC_BKP);
    rcc_clk_disable(RCC_CRC);
    rcc_clk_disable(RCC_DAC);
    rcc_clk_disable(RCC_DMA1);
    rcc_clk_disable(RCC_DMA2);
    rcc_clk_disable(RCC_FLITF);
    rcc_clk_disable(RCC_FSMC);
    //rcc_clk_disable(RCC_GPIOA); // needed by N5110
    rcc_clk_disable(RCC_GPIOB);
    rcc_clk_disable(RCC_GPIOC);
    rcc_clk_disable(RCC_GPIOD);
    rcc_clk_disable(RCC_GPIOE);
    rcc_clk_disable(RCC_GPIOF);
    rcc_clk_disable(RCC_GPIOG);
    rcc_clk_disable(RCC_I2C1);
    rcc_clk_disable(RCC_I2C2);
    //rcc_clk_disable(RCC_PWR); // needed by standby
    rcc_clk_disable(RCC_SDIO);
    //rcc_clk_disable(RCC_SPI1); // needed by N5110
    rcc_clk_disable(RCC_SPI2);
    rcc_clk_disable(RCC_SPI3);
    rcc_clk_disable(RCC_SRAM);
    rcc_clk_disable(RCC_TIMER1);
    rcc_clk_disable(RCC_TIMER2);
    rcc_clk_disable(RCC_TIMER3);
    rcc_clk_disable(RCC_TIMER4);
    rcc_clk_disable(RCC_TIMER5);
    rcc_clk_disable(RCC_TIMER6);
    rcc_clk_disable(RCC_TIMER7);
    rcc_clk_disable(RCC_TIMER8);
    rcc_clk_disable(RCC_TIMER9);
    rcc_clk_disable(RCC_TIMER10);
    rcc_clk_disable(RCC_TIMER11);
    rcc_clk_disable(RCC_TIMER12);
    rcc_clk_disable(RCC_TIMER13);
    rcc_clk_disable(RCC_TIMER14);
    rcc_clk_disable(RCC_USART1);
    rcc_clk_disable(RCC_USART2);
    rcc_clk_disable(RCC_USART3);
    rcc_clk_disable(RCC_UART4);
    rcc_clk_disable(RCC_UART5);
    rcc_clk_disable(RCC_USB);
}

void setGPIOmode(gpio_pin_mode mode) 
{
  for(int i = 0; i < 16; i++) {
    gpio_set_mode(GPIOA, i, mode);
    gpio_set_mode(GPIOB, i, mode);
    gpio_set_mode(GPIOC, i, mode);
  }
}

static void int_fun() {};

// standby=true for deep sleep
void sleepMode(bool standby, uint8_t seconds)
{ 
  rtclock.createAlarm(&int_fun, rtclock.getTime()+seconds);  // wakeup int
  PWR_BASE->CR &= PWR_CR_LPDS | PWR_CR_PDDS | PWR_CR_CWUF;
  PWR_BASE->CR |= PWR_CR_CWUF;
  PWR_BASE->CR |= PWR_CSR_EWUP;
  SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;
  if(standby) {
    PWR_BASE->CR |= PWR_CR_PDDS;
    PWR_BASE->CR &= ~PWR_CR_LPDS;
  } else {
    adc_disable(ADC1);
    adc_disable(ADC2);
    PWR_BASE->CR &= ~PWR_CR_PDDS;
    PWR_BASE->CR |= PWR_CR_LPDS;
  }
  asm("    wfi");
  SCB_BASE->SCR &= ~SCB_SCR_SLEEPDEEP;
}

void setPLL(rcc_pll_multiplier mult) 
{
  rcc_switch_sysclk(RCC_CLKSRC_HSI);
  rcc_turn_off_clk(RCC_CLK_PLL);
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , mult);
}

void setup()
{
  Serial.begin(115200);
  lcd.init();
  lcd.clrScr();
  lcd.setFont(Small4x7PL);
  lcd.printStr(ALIGN_CENTER, 1, "Starting ...");
  delay(2000);
}

unsigned long delTime = 10000;

void loop()
{
  lcd.setFont(Small4x7PL);
  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "Regular mode");
  delay(delTime);

  pinMode(PC13,   OUTPUT); 
  digitalWrite(PC13,0); // on
  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "Regular mode");
  lcd.printStr(ALIGN_CENTER, 2, "PC13 LED on");
  delay(delTime);
  digitalWrite(PC13,1); // off

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC disabled");
  adc_disable_all();
  delay(delTime);

  setGPIOmode(GPIO_INPUT_ANALOG); 
  lcd.init();
  lcd.setFont(Small4x7PL);
  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC disabled");
  lcd.printStr(ALIGN_CENTER, 2, "GPIO_INPUT_ANALOG");
  adc_disable_all();
  delay(delTime);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC disabled");
  lcd.printStr(ALIGN_CENTER, 2, "GPIO_INPUT_ANALOG");
  lcd.printStr(ALIGN_CENTER, 3, "Disable all clocks");
  disableClocks();
  delay(delTime);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC disabled");
  lcd.printStr(ALIGN_CENTER, 2, "GPIO_INPUT_ANALOG");
  lcd.printStr(ALIGN_CENTER, 3, "Disable all clocks");
  lcd.printStr(ALIGN_CENTER, 4, "<STOP>");
  sleepMode(false,delTime/1000);  // stop
  setPLL(RCC_PLLMUL_9);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC disabled");
  lcd.printStr(ALIGN_CENTER, 2, "GPIO_INPUT_ANALOG");
  lcd.printStr(ALIGN_CENTER, 3, "Disable all clocks");
  lcd.printStr(ALIGN_CENTER, 4, "<STOP>");
  lcd.printStr(ALIGN_CENTER, 5, "N5110 sleep in 2s");
  delay(2000); lcd.sleep(true);
  sleepMode(false,delTime/1000);  // stop
  lcd.sleep(false);
  setPLL(RCC_PLLMUL_9);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "Prescaler");
  lcd.printStr(ALIGN_CENTER, 3, "DIV_1");
  lcd.printStr(ALIGN_CENTER, 4, "72MHz");
  //rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_1);
  delay(delTime);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "Prescaler");
  lcd.printStr(ALIGN_CENTER, 3, "DIV_2");
  lcd.printStr(ALIGN_CENTER, 4, "36MHz");
  rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_2);
  delay(delTime/2);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "Prescaler");
  lcd.printStr(ALIGN_CENTER, 3, "DIV_4");
  lcd.printStr(ALIGN_CENTER, 4, "18MHz");
  rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_4);
  delay(delTime/4);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "Prescaler");
  lcd.printStr(ALIGN_CENTER, 3, "DIV_8");
  lcd.printStr(ALIGN_CENTER, 4, "9MHz");
  rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_8);
  delay(delTime/8);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "Prescaler");
  lcd.printStr(ALIGN_CENTER, 3, "DIV_16");
  lcd.printStr(ALIGN_CENTER, 4, "4.5MHz");
  rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_16);
  delay(delTime/16);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "Prescaler");
  lcd.printStr(ALIGN_CENTER, 3, "DIV_256");
  lcd.printStr(ALIGN_CENTER, 4, "281.25kHz");
  rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_256);
  delay(delTime/256);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "Stop @ DIV_256");
  sleepMode(false,delTime/1000);  // stop

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "After");
  lcd.printStr(ALIGN_CENTER, 2, "Stop @ DIV_256");
  lcd.printStr(ALIGN_CENTER, 3, "HSI 8MHz");
  delay(1+delTime/256/9); // 72/8, we are in 8MHz mode

  rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_1);
  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "RCC_PLLMUL_2");
  lcd.printStr(ALIGN_CENTER, 3, "16MHz");
  setPLL(RCC_PLLMUL_2);
  delay(delTime*2/9);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "RCC_PLLMUL_3");
  lcd.printStr(ALIGN_CENTER, 3, "24MHz");
  setPLL(RCC_PLLMUL_3);
  delay(delTime*3/9);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "RCC_PLLMUL_4");
  lcd.printStr(ALIGN_CENTER, 3, "32MHz");
  setPLL(RCC_PLLMUL_4);
  delay(delTime*4/9);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "RCC_PLLMUL_6");
  lcd.printStr(ALIGN_CENTER, 3, "48MHz");
  setPLL(RCC_PLLMUL_6);
  delay(delTime*6/9);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "RCC_PLLMUL_8");
  lcd.printStr(ALIGN_CENTER, 3, "64MHz");
  setPLL(RCC_PLLMUL_8);
  delay(delTime*8/9);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "RCC_PLLMUL_9");
  lcd.printStr(ALIGN_CENTER, 3, "72MHz (default)");
  setPLL(RCC_PLLMUL_9);
  delay(delTime);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "RCC_PLLMUL_10");
  lcd.printStr(ALIGN_CENTER, 3, "80MHz (OC)");
  setPLL(RCC_PLLMUL_10);
  delay(delTime*10/9);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "RCC_PLLMUL_12");
  lcd.printStr(ALIGN_CENTER, 3, "96MHz (OC)");
  setPLL(RCC_PLLMUL_12);
  delay(delTime*12/9);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "ADC/GPIO/clocks off");
  lcd.printStr(ALIGN_CENTER, 2, "RCC_PLLMUL_16");
  lcd.printStr(ALIGN_CENTER, 3, "128MHz (OC)");
  setPLL(RCC_PLLMUL_16);
  delay(delTime*16/9);
  setPLL(RCC_PLLMUL_9);

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "Regular mode");
  lcd.printStr(ALIGN_CENTER, 2, "<STANDBY 10s>");
  lcd.printStr(ALIGN_CENTER, 3, "in 2s");
  delay(2000);
  sleepMode(true,delTime/1000);  // standby

  lcd.clrScr();
  lcd.printStr(ALIGN_CENTER, 1, "Regular mode");
  lcd.printStr(ALIGN_CENTER, 2, "After Standby");
  delay(delTime);
}

