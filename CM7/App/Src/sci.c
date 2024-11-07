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

// 送受信バッファ（リングバッファ）(デバッグ通信用)
#define RX_BUFF_SIZE 16
static volatile char  rx_buff[RX_BUFF_SIZE];
static volatile uint32_t rx_tail = 0;  // 受信バッファTailポインタ（格納ポインタ）
static          uint32_t rx_head = 0;  // 受信バッファHeadポインタ（取り出しポインタ）

#define TX_BUFF_SIZE 128
static          char tx_buff[TX_BUFF_SIZE];
static          uint32_t tx_tail = 0;  // 送信バッファTailポインタ（格納ポインタ）
static volatile uint32_t tx_head = 0;  // 送信バッファHeadポインタ（取り出しポインタ）

//タイマ3割込みで処理
void SCI_IrqHandler(void)//tim3
{
    //更新割込み処理終了
    if(TIM3->SR & TIM_SR_UIF){
        TIM3->SR &= ~TIM_SR_UIF;
    }

    //
    SCI_RX_handler();
    SCI_TX_handler();
}

void SCI_RX_handler(void)
{
    USART_TypeDef *UARTx = SCI_DEBUG_UART;

    if(!UARTx) {
        return;
    }

    // 受信FIFOのデータをすべて処理する(受信FIFOのデータ格納段数<RLVL>が0になるまで)
    while (UARTx->ISR & USART_ISR_RXNE_RXFNE) {
        // 受信FIFOから受信データ取り出し
        char read_data = (char)(UARTx->RDR & 0x000000FF);
        uint32_t rx_tail_tmp = (rx_tail+1) % RX_BUFF_SIZE;

        // 受信バッファに空きがあれば格納、なければ破棄
        if (rx_tail_tmp != rx_head) {
            rx_buff[rx_tail] = read_data;
            // 受信バッファ格納ポインタ更新
            rx_tail = rx_tail_tmp;
        }
    }
}

void SCI_TX_handler(void)
{
    USART_TypeDef *UARTx = SCI_DEBUG_UART;

    // 送信バッファが空になるまで繰り返す
    while (tx_tail != tx_head) {
        if( UARTx->ISR & USART_ISR_TXE_TXFNF ) {
            // 送信FIFOに空きがあれば送信データを書き込む
            UARTx->TDR = tx_buff[tx_head];
            // 送信バッファ取り出しポインタ更新
            tx_head = (tx_head+1) % TX_BUFF_SIZE;
        } else {
            // 送信FIFOがフルなので次の割り込みまで処理を延期する
            return;
        }
    }
}

/**********************************
  １バイト受信
 **********************************/
int32_t SCI_read(int32_t *read_char)
{
    if (rx_tail != rx_head) {
        // 受信バッファにデータがあれば取り出す
        *read_char = rx_buff[rx_head];
        // 受信バッファ取り出しポインタ更新
        rx_head = (rx_head+1) % RX_BUFF_SIZE;
        return 1;
    }

    // 受信バッファが空なら何もしないで終了
    *read_char = 0;
    return 0;
}

/*********************************
  １バイト送信
 *********************************/
int32_t SCI_write(int32_t send_char)
{
    uint32_t txTailTmp = (tx_tail+1) % TX_BUFF_SIZE;
    // 送信バッファに空きができるまで待つ
    while (txTailTmp == tx_head) {}
    // 送信バッファに送信データ格納
    tx_buff[tx_tail] = (char)send_char;
    // 送信バッファ格納ポインタ更新
    tx_tail = txTailTmp;

    return 1;
}

/**********************************

  これ以降は汎用関数
  ハードウェアに依存しない

 **********************************/

/**********************************
  １文字表示
 **********************************/
int32_t SCI_putchar(int32_t c)
{
    /* 改行はCR+LFに変換して送信 */
    if(c == '\n'){
        if(!SCI_write(0x0D))
            return(0);
        if(!SCI_write(0x0A))
            return 0;
    }

    /* それ以外はそのまま送信 */
    else{
        if(!SCI_write(c))
            return 0;
    }

    return 1;
}

/**********************************
  文字列表示
 **********************************/
static int32_t printStr(const char *str)
{
    const char *s;

    /* 終端文字まで1文字ずつ表示 */
    for(s=str; *s != '\0'; s++)
        if(!SCI_putchar(*s))
            return 0;

    /* 表示した文字数を返す */
    return (s - str);
}

/**********************************
  文字列表示(最後に改行付き)
 **********************************/
int32_t SCI_puts(const char *str)
{
    int32_t len;

    /* 文字列表示 */
    len = printStr(str);

    /* 最後に改行を表示 */
    SCI_putchar('\n');

    return (len + 1);
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
                SCI_putchar('-');
                minus = 0;
            }
        }

        for(i=0; i<order; i++)
            SCI_putchar(pad);
    }

    /* マイナス表示 */
    if(minus)
        SCI_putchar('-');

    /* データの表示 */
    printStr(p);

    /* 左詰め */
    if(alignLeft)
        for(i=0; i<order; i++)
            SCI_putchar(' ');
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
int32_t SCI_printf(const char *str, ... )
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
                SCI_putchar('%');
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
            SCI_putchar(*s);
            s++;
        }
    }

    va_end(ap);
    return(s - str);
}

/**********************************
  １文字受信
 **********************************/
int32_t SCI_getchar(void)
{
    int32_t c;

    /* データを受信するまで待つ */
    while(!SCI_read(&c));

    /* 改行文字変換 */
    if(c == 0x0D)
        c = '\n';
    return c;
}

/**********************************
  １文字受信(データを待たない)
 **********************************/
int32_t SCI_getchar2(void)
{
    int32_t c;

    /* データを受信 */
    SCI_read(&c);

    /* 改行文字変換 */
    if(c == 0x0D)
        c = '\n';
    return c;
}


/**********************************
  文字列を一行受信
 **********************************/
char *SCI_gets(char *s)
{
    int32_t i = 0;
    int32_t c;

    /* 改行まで受信 */
    while((c = SCI_getchar()) != '\n'){
        s[i] = c;
        i++;
        if (isprint(c)) {
            // SCI_putchar(c);     /* エコーバック */
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
