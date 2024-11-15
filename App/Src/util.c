
#include    <stdio.h>
#include    "util.h"
#include    "console.h"

/**********************************
  16進数でダンプ表示
 **********************************/
void dumpHex(const char *msg, const char *s, uint16_t len) {

    uint16_t i,j;

    if(msg != NULL) printf("*** %s (%d) ***\n", msg, len);
    for(i = 0; i < len;) {
        printf("%08X : ", (unsigned int)s + i);
        for(j=0; j < 16; j++) {
            printf("%02x ", s[i]);
            i++;
            if(i == len) {
                putchar('\n');
                break;
            }
        }
        putchar('\n');
    }
}
