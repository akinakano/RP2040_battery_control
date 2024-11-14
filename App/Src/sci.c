/*******************************
    Debug用UART通信ライブラリー

 *******************************/
#include    <stdio.h>
#include    <ctype.h>
#include    <stdarg.h>
#include    <stm32h747xx.h>
#include    "tim.h"
#include    "sci.h"
#include    "util.h"
#include    "gpio.h"

#define     ABS_I(x)    (x < 0 ? (-x) :(x))

static volatile uint8_t rx_buff[RX_BUFF_SIZE];
static volatile int rx_tail = 0;
static          int rx_head = 0;
static          uint8_t tx_buff[TX_BUFF_SIZE];
static          int tx_tail = 0;
static volatile int tx_head = 0;

void SCI_Init(void) { // DebugUART

  // Peripheral clock enable
  RCC->APB1LENR |= RCC_APB1LENR_USART2EN;
  // PD5      ------> AF7:UART2_TX
  // PD6      ------> AF7:UART2_RX
  GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL5_Msk ) | ( 7 << GPIO_AFRL_AFSEL5_Pos ) );
  GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE5_Msk ) | ( 2 << GPIO_MODER_MODE5_Pos ) );
  GPIOD->AFR[0] = ( ( GPIOD->AFR[0] & ~GPIO_AFRL_AFSEL6_Msk ) | ( 7 << GPIO_AFRL_AFSEL6_Pos ) );
  GPIOD->MODER = ( ( GPIOD->MODER & ~GPIO_MODER_MODE6_Msk ) | ( 2 << GPIO_MODER_MODE6_Pos ) );

  // baudrate : 115200
  // parity   : none
  // word     : 8bit
  // stop     : 1bit
  // flow ctrl: disable
  SCI_UART->CR1 &= ~USART_CR1_UE;
  SCI_UART->CR1 = USART_CR1_TXFEIE | USART_CR1_FIFOEN | USART_CR1_RTOIE | USART_CR1_TE | USART_CR1_RE;
  SCI_UART->CR2 = USART_CR2_RTOEN;
  SCI_UART->CR3 = USART_CR3_RXFTIE | (0b100 << USART_CR3_RXFTCFG_Pos);
  SCI_UART->GTPR = 0; // prescaller x1
  SCI_UART->RTOR = SCI_BAUDRATE * 20 / 1000; // 20ms
  uint32_t usart_div = APB_CLK / SCI_BAUDRATE;
  SCI_UART->BRR = (usart_div & (USART_BRR_DIV_FRACTION_Msk|USART_BRR_DIV_MANTISSA_Msk)) << USART_BRR_DIV_FRACTION_Pos; //baudrate
  SCI_UART->CR1 |= USART_CR1_UE;
}

void SCI_IRQHandler(void) {

  uint32_t isr = SCI_UART->ISR;
  if(isr & USART_ISR_TXFE) {
    while (tx_tail != tx_head) {
      if( SCI_UART->ISR & USART_ISR_TXE_TXFNF ) {
        SCI_UART->TDR = tx_buff[tx_head];
        tx_head = (tx_head + 1) % TX_BUFF_SIZE;
      }
    }
    SCI_UART->CR1 &= ~USART_CR1_TXFEIE;
  }
  if(isr & (USART_ISR_RXFT | USART_ISR_RTOF)) {
    while(SCI_UART->ISR & USART_ISR_RXNE_RXFNE) {
      uint8_t d = (uint8_t)SCI_UART->RDR;
      int rx_tail_tmp = (rx_tail + 1) % RX_BUFF_SIZE;
      if(rx_tail_tmp != rx_head) {
        rx_buff[rx_tail] = d;
        rx_tail = rx_tail_tmp;
      }
    }
    SCI_UART->ICR = USART_ICR_RTOCF;
  }
}

int SCI_checkc(void) {
  if(rx_tail != rx_head) {
    int d = rx_buff[rx_head];
    rx_head = (rx_head + 1) % RX_BUFF_SIZE;
    return d;
  }
  return -1;
}

int SCI_getc(void) {
  int c;
  while((c = SCI_checkc()) < 0);
  if(c == 0x0d) c = '\n';
  return c;
}

void SCI_putc(uint8_t c) {
    int txTailTmp = (tx_tail + 1) % TX_BUFF_SIZE;
    if(txTailTmp == tx_head) return;
    tx_buff[tx_tail] = c;
    tx_tail = txTailTmp;
    if(!(SCI_UART->CR1 & USART_CR1_TXFEIE)) SCI_UART->CR1 |= USART_CR1_TXFEIE;
}

int SCI_puts(const char *str) {

  const char *s;
  for(s = str; *s; s++) {
    if(*s == 0x0a) SCI_putc(0x0d);
    SCI_putc(*s);
  }
  return s - str;
}

/**********************************
  指定されたフォーマットで表示する
 **********************************/
static void printFmt(char *p, uint16_t order, uint16_t alignLeft, uint16_t fillZero, uint16_t minus)
{
    char *s = p;
    char pad = ' ';
    uint16_t len = 0;
    int32_t i;

    /* 文字数のカウント */
    for(len=0; *s != '\0'; len++, s++);

    /* マイナスなら文字数調整 */
    if(minus)
        len++;

    /* 文字数の調整 */
    if(order){
        if(order > len){
            order -= len;
        }else{
            order = 0;
        }
    }

    /* 右詰め */
    if( ! alignLeft){
        /* 詰め文字の設定 */
        if(fillZero){
            pad = '0';
            
            /* マイナス表示 */
            if(minus){
                SCI_putc('-');
                minus = 0;
            }
        }

        for(i=0; i<order; i++)
            SCI_putc(pad);
    }

    /* マイナス表示 */
    if(minus)
        SCI_putc('-');

    /* データの表示 */
    SCI_puts(p);

    /* 左詰め */
    if(alignLeft)
        for(i=0; i<order; i++)
            SCI_putc(' ');
}

/**********************************
  フォーマットを解釈する
 **********************************/
static const char *parseFmt(const char *s, void *value)
{
    char buf[12];
    //char buf_f[6];
    char *p = buf;
    uint16_t alignLeft = 0;
    uint16_t fillZero = 0;
    uint16_t order = 0;
    uint16_t minus = 0;

    /* 左詰判定 */
    if(*s == '-'){
        alignLeft = 1;
        s++;
    }

    /* ゼロフィル判定 */
    if(*s == '0'){
        fillZero = 1;
        s++;
    }

    /* 文字数指定判定 */
    if(isDec(*s)){
        for(order = 0; isDec(*s); ){
            order *= 10;
            order += (*s - '0');
            s++;
        }
    }

    /* 種類判定、表示準備 */
    switch (*s){
    case 'd':   /* 符号付10進数 */
        /* 正負の判定 */
        if((int32_t)value >= 0){
            uint2Dec((uint32_t)value, buf);
        }else{
            uint2Dec((uint32_t)(-(int32_t)value), buf);
            minus = 1;
        }
        break;
    case 'u':   /* 符号無し10進数 */
        uint2Dec((uint32_t)value, buf);
        break;
    case 'x':   /* 小文字16進数 */
        uint2Hex((uint32_t)value, 0, buf);
        break;
    case 'X':   /* 大文字16進数 */
        uint2Hex((uint32_t)value, 1, buf);
        break;
    case 's':   /* 文字列 */
        p = (char*)value;
        break;
    case 'c':   /* １文字 */
        buf[0] = (char)((uint32_t)value & 0xFF);
        buf[1] = '\0';
        break;
    // case 'f':   /* float型 */

    //     value_int = (int32_t)value;
    //     vf = (float)(*value);
    //     // value_float = (int32_t)((float)value*1000 - value_int * 1000);

    //     break;
    default:
        buf[0] = '\0';
        break;
    }

    /* 表示 */
    printFmt(p, order, alignLeft, fillZero, minus);

    s++;
    return(s);
}

/**********************************
  いわゆる printf
 **********************************/
int SCI_printf(const char *str, ... )
{
    va_list ap;
    const char *s;

    /* 可変引数の初期化 */
    va_start(ap, str);

    for(s = str; *s != '\0'; ){
        /* 特殊文字判定 */
        if(*s == '%'){
            s++;

            /* "%%"なら'%'を表示 */
            if(*s == '%'){
                SCI_putc('%');
                s++;
            }
            /* フォーマットに従って表示 */
            else{
                s = parseFmt(s, va_arg(ap, void *));
                if(s == NULL)
                    return -1;
            }
        }

        /* 1文字ずつ普通に表示 */
        else{
            if(*s == 0x0a) SCI_putc(0x0d);
            SCI_putc(*s);
            s++;
        }
    }

    va_end(ap);
    return(s - str);
}

/**********************************
  文字列を一行受信
 **********************************/
char *SCI_gets(char *s)
{
    int32_t i = 0;
    int32_t c;

    /* 改行まで受信 */
    while((c = SCI_getc()) != '\n'){
        s[i] = c;
        i++;
        if (isprint(c)) {
            SCI_printf("%c", c);
        } else {
                //SCI_printf("[0x%02X]",c);
                // 何も表示しない
        }
    }

    /* 改行をNULLに変換 */
    s[i] = '\0';

    return s;
}

void strtofloat(float *f, char* sign, int* t_i, int* t_f)
{
    *t_i = ABS_I((int32_t)*f);
    *t_f = ABS_I((int32_t)(*f*1000 - (int32_t)*f*1000));
    if (*f < 0) {
        *sign = '-';
    } else {
        *sign = ' ';
    }
}
