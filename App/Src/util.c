
#include    <stdio.h>
#include    "util.h"
#include    "sci.h"

/**********************************
  10進数文字かチェック
 **********************************/
int32_t isDec(int32_t c)
{
    if ('0'<=c && c<='9')
        return TRUE;
    
    return FALSE;
}

/**********************************
  16進数文字かチェック
 **********************************/
int32_t isHex(int32_t c)
{
    if ('0'<=c && c<='9')
        return 1;
    else if(c >= 'a' && c <='f')
        return 1;
    else if(c >= 'A' && c <='F')
        return TRUE;

    return FALSE;
}

/***********************************
  10進数文字から数値に変換
 ***********************************/
int32_t dec2Int(const char *s, int32_t *n)
{
    uint16_t flag = 0;
    uint16_t minus = 0;

    /* マイナスチェック */
    if(*s == '-'){
        minus = 1;
        s++;
    }
    
    /* 文字列を数値に変換 */
    for(*n = 0; isDec(*s); s++){
        *n *= 10;
        *n += (*s - '0');
        flag = 1;
    }

    /* マイナス時の処理 */
    if(minus)
        *n = -*n;

    /* 一文字も変換していない場合 */
    if( ! flag)
        return NG;

    return OK;
}

/**********************************
  16進数文字から数値に変換
 **********************************/
int32_t hex2Int(const char *s, int32_t *n)
{
    int flag = 0;
    
    /* 文字列を数値に変換 */
    for(*n = 0; isHex(*s); s++){
        *n <<= 4;
        if(*s >= '0' && *s <='9')
            *n += (*s - '0');
        else if(*s >= 'a' && *s <='f')
            *n += (*s - 'a' + 10);
        else 
            *n += (*s - 'A' + 10);
        flag = 1;
    }

    /* 一文字も変換していない場合 */
    if( ! flag)
        return NG;

    return OK;
}

/**********************************
  1文字入力
 **********************************/
int32_t inputChar(void)
{
    int32_t c;

    /* キー入力 */
    SCI_printf("> ");
    c = SCI_getc();

    /* 押した文字とその文字コードを表示 */
    if(c != '\n')
        SCI_printf("%c (%d)\n", c, c);
    else
        SCI_printf("(%d)\n", c);

    return c;
}

/**********************************
  10進数文字入力
 **********************************/
int32_t inputDec(const int8_t *msg, int32_t *n, int32_t min, int32_t max)
{
    char buf[16];

    SCI_printf("%s [%d - %d] > ", msg, min, max);
    SCI_gets(buf);
    SCI_printf("\n");

    if(OK == dec2Int(buf, n)){
        if(*n >= min &&  *n <= max){
            return OK;
        }
    }

    return NG;
}

/**********************************
  16進数文字入力
 **********************************/
int32_t inputHex(const int8_t *msg, int32_t *n, int32_t min, int32_t max)
{
    char buf[16];

    SCI_printf("%s [0x%X - 0x%X] > ", msg, min, max);
    SCI_gets(buf);
    SCI_printf("\n");

    if(OK == hex2Int(buf, n)){
        if(*n >= min &&  *n <= max){
            return OK;
        }
    }

    return NG;
}

/**********************************
  数値を10進数文字変換
 **********************************/
void uint2Dec(uint32_t n, char *buf)
{
    char c;
    uint16_t len = 0;
    int32_t i, half;

    /* 10進文字列へ変換し文字数をカウント */
    do{
        if(n == 0)
            i = 0;
        else
            i = n % 10;
        buf[len] = (char)(i + '0');
        len++;
        n /= 10;
    }while(n != 0);

    /* 文字の並び順を直す */
    half = len >> 1;
    for(i=0; i < half; i++){
        c = buf[i];
        buf[i] = buf[(len-1)-i];
        buf[(len-1)-i] = c;
    }

    /* 終端文字列の挿入 */
    buf[len]='\0';
}

/**********************************
  数値を16進数文字変換
 **********************************/
void uint2Hex(uint32_t n, uint16_t upper, char *buf)
{
    char c;
    char a = 'a';
    uint16_t len = 0;
    int32_t i, half;

    /* 大文字/小文字の設定 */
    if(upper)
        a = 'A';
    
    /* 16進文字列へ変換し文字数をカウント */
    do{
        i = n & 0x0F;
        if(i > 9)
            buf[len] = (char)(i + a - 10);
        else
            buf[len] = (char)(i + '0');
        len++;
        n >>= 4;
    }while(n != 0);

    /* 文字の並び順を直す */
    half = len >> 1;
    for(i=0; i < half; i++){
        c = buf[i];
        buf[i] = buf[(len-1)-i];
        buf[(len-1)-i] = c;
    }

    /* 終端文字列の挿入 */
    buf[len]='\0';
}

/**********************************
  16進数でダンプ表示
 **********************************/
void dumpHex(const char *msg, const char *s, uint16_t len)
{
    uint16_t i,j;

    if(msg != NULL)
        SCI_printf("*** %s (%d) ***\n", msg, len);
    
    for(i=0; i < len; ){
        SCI_printf("%08X : ", s+i);
        for(j=0; j < 16; j++){
            SCI_printf("%02x ",s[i]);
            i++;
            if(i == len){
                SCI_putc('\n');
                break;
            }
        }
        SCI_putc('\n');
    }
}

