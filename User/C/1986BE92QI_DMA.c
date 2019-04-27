#include "1986BE92QI_DMA.h"
#include "math.h" 
#include "main.h" 

uint16_t BufferData [1024*2];                                             //Этот буффер будем выдавать циклично.

struct DAC_ST
{
    uint32_t Destination_end_pointer;                                     //Указатель конца данных приемника.
    uint32_t Source_end_pointer;                                          //Указатель конца данных источника
    uint32_t channel_cfg;                                                 //Конфигурация канала.
    uint32_t NULL;                                                        //Пустая ячейка. 
} 

__align(1024) DAC_ST;                                                   //Выравниваем массив структур по 1024 байта. 
struct DAC_ST DAC_ST_ADC[32+32];                                        //Создаем массив структур для всех каналов. 

//Источник/приемник = 16 бит, отправляем/принимаем = 16 бит, защиты нет, 50 передач, пинг-понг.
uint32_t DMA_DAC_InitST_PR  = dst_src|src_inc|src_size|dst_size|n_minus_1|cycle_ctrl;
uint32_t DMA_DAC_InitST_ALT = dst_src|src_inc|src_size|dst_size|n_minus_1|cycle_ctrl;

//-------------------------------------------------
//Настраиваем DMA для связки с DAC.
//-------------------------------------------------
void DMA_to_DAC_and_TIM1 (void) 
{
  //Настраиваем первичную структуру.
  DAC_ST_ADC[10-1].Destination_end_pointer = (uint32_t)BufferData + (sizeof(BufferData))/2 - 1;       //Указатель на последний элемент середины массива (C_4 - массив значений синусоидального сигнала в 100 значений).
  DAC_ST_ADC[10-1].Source_end_pointer = (uint32_t)&(DAC->DAC2_DATA);                                  //Указатель на последний (не меняется) адрес приемника (регистр данных DAC).
  DAC_ST_ADC[10-1].channel_cfg = (uint32_t)(DMA_DAC_InitST_PR);                                       //Структура настройки первичной структуры.
  DAC_ST_ADC[10-1].NULL = (uint32_t)0;                                                                //Пустая ячейка.
  //Настройка альтернативной структуры. 
  DAC_ST_ADC[10-1+32].Destination_end_pointer = (uint32_t)BufferData + sizeof(BufferData) - 1;        //Указатель на последний элемент массива (C_4 - массив значений синусоидального сигнала в 100 значений).
  DAC_ST_ADC[10-1+32].Source_end_pointer = (uint32_t)&(DAC->DAC2_DATA);                               //Указатель на последний (не меняется) адрес приемника (регистр данных DAC).
  DAC_ST_ADC[10-1+32].channel_cfg = (uint32_t)(DMA_DAC_InitST_ALT);                                   //Структура настройки альтернативной структуры.
  DAC_ST_ADC[10-1+32].NULL = (uint32_t)0;                                                             //Пустая ячейка.
  
  //Настраиваем контроллер DMA.
  RST_CLK->PER_CLOCK|=PCLK_EN_DMA;                                                      //Включаем тактирование DMA.
  DMA->CTRL_BASE_PTR = (uint32_t)&DAC_ST_ADC;                                           //Указываем адрес массива структур. 
  DMA->CFG = CFG_master_enable;                                                         //Разрешаем работу DMA.
    
  //Настраиваем канал. 
  DMA->CHNL_ENABLE_SET   = 1<<10;                                                       //Разрешаем работу 10 канала.
}

#define ST_Play_P           (DAC_ST_ADC[10-1].channel_cfg    & (1023<<4))               //Для проверки колличества оставшихся передачь в первичной сруктуре.
#define ST_Play_ALT         (DAC_ST_ADC[10-1+32].channel_cfg & (1023<<4))               //Для проверки колличества оставшихся передачь в альтернативной сруктуре.

uint16_t LoopBUFData = 0;                                                               //Колличество готовых к перезаписи ячеек.       
void Timer2_IRQHandler (void)                                                           //Меняем структуры.
{
    if ((ST_Play_P == 0) && (ST_Play_ALT <= ((1024-2)<<4)))                             //Если прошли первую половину и уже начата передача второй - переключиться на 2-ю.	
        {
  	      DAC_ST_ADC[10-1].channel_cfg = (uint32_t)(DMA_DAC_InitST_PR);                 //Заново заполняем структуру первой. 
        };
    if ((ST_Play_ALT == 0) && (ST_Play_P <= ((1024-2)<<4)))
        {
        	DAC_ST_ADC[10-1+32].channel_cfg = (uint32_t)(DMA_DAC_InitST_ALT);             //Альтернативной.                            
        }    
		LoopBUFData=512;                                                                    //Разрешаем перезапись 512 ячеек. 
		DMA->CHNL_ENABLE_SET   = 1<<10;                                                     //Разрешаем передачу дальше.                         
		TIMER2->STATUS=0;                                                                   //Сбрасываем флак, чтобы не зайти в это прерывание снова.
}

extern const uint16_t MesKOL [37];
extern const uint16_t MesSM [37];
extern const uint8_t SinMES [3849];

uint16_t BufferIndex = 0;                                             //Указатель на первый пустой элемент массива.
uint16_t LoopSin0, LoopSin1, LoopSin2, LoopSin3;                      //Указатель на передаваемую ячейку значения периода волны.
uint8_t  WavTon_BF0, WavTon_BF1, WavTon_BF2, WavTon_BF3;              //Для "красивой" синусоиды. Чтобы в случае, если несколько шестнадцатых подряд идет одна и так же нота - ее синусоида не "рвалась", а продолжала.
void NoteBUF (uint8_t NambeN0, uint8_t NambeN1, uint8_t NambeN2, uint8_t NambeN3)
{
    
    //Частота выдачи 32000 Гц. 32000 передач в секунде. 
    //Скорость воспроизведения 120 ударов в минуту. В минуте 60 секунд. 
    //=> у нас 600/60 ударов в секунду. 
    //У нас 1 такт - 4/4 (четыре четверти). В такте 4 удара.
    //=>1 такт = 4 секунды. 
	  //Мы играем 128-тыми. 32000 в секунду * 4 / 128 доли 
    // = 1000 передачь за одну шестнадцатую. 
 
    if (WavTon_BF0!=NambeN0) {WavTon_BF0=NambeN0; LoopSin0=0;};                                //Если передаем новую волну - то с начала пириода.
    if (WavTon_BF1!=NambeN1) {WavTon_BF1=NambeN1; LoopSin1=0;};  
    if (WavTon_BF2!=NambeN2) {WavTon_BF2=NambeN2; LoopSin2=0;};  
		if (WavTon_BF3!=NambeN3) {WavTon_BF3=NambeN3; LoopSin3=0;}; 
		
  for (uint16_t Del16 = 0; Del16<1000; Del16++)                                                //Передаем 64-ю.
    { 
      while (LoopBUFData == 0) {};                                                						//Ждем, пока предыдущая часть массива передастся. 
			BufferData[BufferIndex]=SinMES[LoopSin0+MesSM[NambeN0]]+SinMES[LoopSin1+MesSM[NambeN1]]+SinMES[LoopSin2+MesSM[NambeN2]]+SinMES[LoopSin3+MesSM[NambeN3]]; //Заполняем буффер. 
			LoopSin0++;  LoopSin1++; LoopSin2++; LoopSin3++;                                        //Приготовиться к показу следущего элемента.
      if (LoopSin0==MesKOL[NambeN0]) LoopSin0=0;                                              //Если прошел весь период 1-й волны - начать с начала.
      if (LoopSin1==MesKOL[NambeN1]) LoopSin1=0;                                              //Тоже самое со второй.
			if (LoopSin2==MesKOL[NambeN2]) LoopSin2=0;
      if (LoopSin3==MesKOL[NambeN3]) LoopSin3=0;				
      BufferIndex++; if (BufferIndex == (1024*2)) BufferIndex = 0;                            //Если счетчик буфера вышел за пределы - записывать с начала. 
      LoopBUFData--;                                                                          //Указываем, что мы передали один элемент.
    }
}

void SM_Ton (uint32_t TN)                           //Меняем тон таймера. 
{
    TIMER1->CNTRL = 0;                              //Останавливаем таймеры.
    TIMER2->CNTRL = 0;
    TIMER1->CNT   = 0;                              //Сбрасываем значение таймера.
    TIMER2->CNT   = 0;
    TIMER1->ARR = TN;                               //Заполняем новые значения. 
    TIMER2->ARR = TN*25;
    DAC_ST_ADC[10-1].channel_cfg = (uint32_t)(DMA_DAC_InitST_PR);      //Перенастраиваем структуру DMA.
    DAC_ST_ADC[10-1+32].channel_cfg = (uint32_t)(DMA_DAC_InitST_ALT); 
    TIMER1->CNTRL = CNTRL_CNT_EN;                   //Разрешаем работу таймеров.
    TIMER2->CNTRL = CNTRL_CNT_EN;      
}

void Ton_off (void)
{
    TIMER1->CNTRL = 0;                              //Останавливаем таймеры.
    TIMER2->CNTRL = 0;
}