#include "app.h"
#include "board.h"
int main(void) {
    board_init();
    app_init();
    for (;;) app_poll();
}
