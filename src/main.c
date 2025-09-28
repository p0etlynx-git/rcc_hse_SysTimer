#include "stm32f1xx.h"

// макросы CMSIS
//SET_BIT( REG, BIT)    <-- установить бит
//READ_BIT( REG, BIT)   <-- получить бит
//CLEAR_BIT( REG, BIT)  <-- сбросить бит

//MODIFY_REG( REG, CLEARMASK, SETMASK)  <-- Изменить регистр
//CLEAR_REG( REG)                       <-- Очистить регистр
//WRITE_REG( REG, VAL)                  <-- Записать регистр

// Настроить системный таймер
void InitSysTick(void);
// включить внешний генератор и настроить частоту 72 000 000
uint32_t InitHSE(void);
// Обработчик прерывания системного таймера
void SysTick_Handler();

// Глобальный переменные системного таймера
__IO uint32_t StartUpCounter;
__IO uint32_t SysTick_CNT = 0;
__IO uint32_t SysTick_val = 0;

// Функция задержки в милисекундах
// На системном таймере
void sys_delay(uint32_t msec)
{
    // Записать в регистр текущего значения системного таймера число полной милисекунды
    MODIFY_REG( SysTick->VAL, SysTick_VAL_CURRENT_Msk, SysTick_val);
    // Записать в счётчик по таймеру количество милисекунд
    SysTick_CNT = msec;
    // Ждать пока счётчик по таймеру отработает
    while(SysTick_CNT) {    }
}

// Тело основной программы
int main( void)
{
    InitHSE();
    InitSysTick();
    // по умолчанию RCC от внутреннего генератора
    // Enable Port A RCC 
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Port A

    // CRH(15..8) CRL(7..0)
    GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);                            // Push Pull mode PA5 Reset register
    GPIOA->CRL |= (0b00 <<GPIO_CRL_CNF5_Pos) | (0b10 << GPIO_CRL_MODE5_Pos);    // Push Pull mode PA5 Set register

    while(1)
    {

        // Turn Led On
        if((GPIOA->ODR & GPIO_ODR_ODR5) == 0){  // если светодиод не горит
            GPIOA->BSRR = GPIO_BSRR_BS5;        // включить его
        } else {                                // иначе
            GPIOA->BSRR = GPIO_BSRR_BR5;        // выключить его
        }
        //delay(100); // ждём
        sys_delay(500);
    }

    return 0;
}

// функция настройки системного таймера
// Ставим задержку в 1-ну милисекунду от текущей частоты ядра
void InitSysTick(void)
{
    // получаем текущую частоту ядра в SystemCoreClock
    SystemCoreClockUpdate();
    // получить предделитель системной частоты до одной милисекунды
    SysTick_val =  SystemCoreClock / 1000 - 1;
    // заносим число тиков в предделитель системного таймера
    MODIFY_REG( SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, SysTick_val);
    // сбросить счетчик
    CLEAR_BIT( SysTick->VAL, SysTick_VAL_CURRENT_Msk);
    // SysTick_CTRL_CLKSOURCE_Msk   - текущая частота ядра
    // SysTick_CTRL_ENABLE_Msk      - включить системный таймер
    // SysTick_CTRL_TICKINT_Msk     - включить прерывания от системного таймера
    SET_BIT( SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
}


// Обработчик прерывания от системного таймера
// подсчитывает прошедшее время в мс.
void SysTick_Handler()
{
    if(SysTick_CNT > 0)  SysTick_CNT--;
}

//Настраиваем тактирование системы от внешнего кварца
//через PLL на максимально возможных частотах.
//Внешний кварц должен быть на 8МГц
//Возвращает:
//  0 - завершено успешно
//  1 - не запустился кварцевый генератор
//  2 - не запустился PLL
uint32_t InitHSE(void)
{
    // ---- Настройка HSE на 72 МГц платы Nucleo F103RB
    // Кварц HSE на 8 МГц
    // PLLXTPRE: без деления
    // PLLSRC: HSE генератор
    // PLLMUL = 9
    // SW = PLLCLK
    // AHB Prescaler = 1
    // APB1 Prescaler = 2
    // APB2 Prescaler = 1    

    // ---- порядок запуска
    // Запустить генератор HSE
    // Настроить PLL
    // Запустить PLL
    // Настроить количество циклов ожидания FLASH
    // Настроить делители шин
    // Переключиться на работу от PLL

    // Запустить генератор HSE      RCC->CR |= RCC_CR_HSEON;
    SET_BIT( RCC->CR, RCC_CR_HSEON);
    // Ждем успешного запуска или окончания тайм-аута
    for( StartUpCounter=0;; StartUpCounter++)
    {
        //Если успешно запустилось, то 
        //выходим из цикла         
        //if( READ_BIT( RCC->CR, RCC_CR_HSERDY) == RCC_CR_HSERDY ) break;
        if( RCC->CR & RCC_CR_HSERDY) break;

        //Если не запустилось, то
        //отключаем все, что включили
        //и возвращаем ошибку
        if(StartUpCounter > 0x1000)
        {
            //Останавливаем HSE             RCC->CR &= ~RCC_CR_HSEON; 
            CLEAR_BIT( RCC->CR, RCC_CR_HSEON);
            return 1;
        }           
    }
    //Настраиваем PLL
    // обнулить                 RCC->CFGR &= ~RCC_CFGR_PLLMULL16; 
    // записать PLL множитель равен 9
    //Тактирование PLL от HSE   RCC->CFGR |= RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC;
    MODIFY_REG( RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL, 
                RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9);
        
    //Запускаем PLL     RCC->CR |= RCC_CR_PLLON; 
    SET_BIT( RCC->CR, RCC_CR_PLLON);
    for( StartUpCounter=0;; StartUpCounter++)
    {
        //Если успешно запустилось, то 
        //выходим из цикла
        //if( READ_BIT( RCC->CR, RCC_CR_PLLRDY) != RESET) break;
        if( RCC->CR & RCC_CR_PLLRDY) break;

        //Если не запустилось, то
        //отключаем все, что включили
        //и возвращаем ошибку
        if(StartUpCounter > 0x1000)
        {
            //RCC->CR &= ~RCC_CR_HSEON; // Останавливаем HSE
            //RCC->CR &= ~RCC_CR_PLLON; // Останавливаем PLL
            CLEAR_BIT( RCC->CR, RCC_CR_HSEON);
            CLEAR_BIT( RCC->CR, RCC_CR_PLLON);
            return 2
        }           
    }
    //Вот здесь кроется один неочевидный момент. 
    //Дело в том, что FLASH-память микроконтроллера, 
    //в которой хранится управляющая программа, 
    //может работать на максимальной частоте 24 МГц. 
    //Обмен данными с FLASH осуществляется через шину AHB. 
    //А если частота шины AHB выше 24 МГц, 
    //то необходимо ввести циклы ожидания обращений к этой памяти, 
    //примем, чем выше частота, тем больше этих циклов надо:

    //ноль циклов ожидания, если 0 < SYSCLK ≤ 24 MHz
    //один цикл ожидания, если 24 MHz < SYSCLK ≤ 48 MHz
    //два цикла ожидания, если 48 MHz < SYSCLK ≤ 72 MHz

    //Выключаем буфер предварительной выборки 
    //и включаем его.
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
    //так как частота ядра у нас будет 48 MHz < SYSCLK <= 72 MHz
    //Устанавливаем 2 цикла ожидания для Flash     FLASH->ACR |= FLASH_ACR_LATENCY_2; 
    MODIFY_REG( FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);
  
    //Делители
    //RCC->CFGR |= RCC_CFGR_PPRE2_0   //Делитель шины APB2 отключен (оставляем 0 по умолчанию)
    //           | RCC_CFGR_PPRE1_DIV2 //Делитель нишы APB1 равен 2
    //           | RCC_CFGR_HPRE_0;   //Делитель AHB отключен (оставляем 0 по умолчанию)
    MODIFY_REG( RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);
    MODIFY_REG( RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);
    MODIFY_REG( RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);

    //RCC->CFGR |= RCC_CFGR_SW_PLL; //Переключаемся на работу от PLL
    MODIFY_REG( RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
      //Ждем, пока переключимся
    //while((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_1)
    while( READ_BIT( RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {    }
  
    //После того, как переключились на
    //внешний источник такирования
    //отключаем внутренний RC-генератор
    //для экономии энергии
    //RCC->CR &= ~RCC_CR_HSION;
    CLEAR_BIT( RCC->CR, RCC_CR_HSION);
    return 0;
}