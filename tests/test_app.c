#include "app.h"
#include "board.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t clock_us,hw_fault;
static int pending,enabled,binary;
static float in1,in2,inbus,duty;
static uint32_t sample_us;
static uint8_t buttons;
static int usb_connected=1;
static uint32_t usb_epoch=1;
static char tx[65536],oled[8][22];
static size_t tx_count;
void board_init(void) {}
void board_poll(void) {}
int board_sample_ready(float *a,float*b,float*c) {
    if(!pending) return 0;
    pending=0;*a=in1;*b=in2;*c=inbus;return 1;
}
void board_set_output(float d,int e,int f){duty=d;enabled=e;binary=f;}
int board_set_sample_us(uint32_t us){if(us<500||us>10000)return 0;sample_us=us;return 1;}
uint32_t board_millis(void){return clock_us/1000;}
uint32_t board_micros(void){return clock_us;}
int board_usb_readline(char *dst,size_t cap){(void)dst;(void)cap;return 0;}
int board_usb_connected(void){return usb_connected;}
uint32_t board_usb_session(void){return usb_epoch;}
int board_usb_write(const char *s,size_t n){
    if(tx_count+n>=sizeof tx)return 0;
    memcpy(tx+tx_count,s,n);tx_count+=n;tx[tx_count]=0;return 1;
}
uint8_t board_buttons(void){return buttons;}
void board_oled_lines(const char *const lines[8]){
    for(int k=0;k<8;++k)(void)snprintf(oled[k],22,"%s",lines[k]);
}
uint32_t board_faults(void){return hw_fault;}
void board_clear_faults(void){if(!enabled)hw_fault=0;}
static AppStatus status(void){AppStatus s;app_status(&s);return s;}
static void advance(uint32_t us,float v1,float v2,float bus){
    clock_us+=us;in1=v1;in2=v2;inbus=bus;pending=1;app_poll();
}
static void reset(void){
    clock_us=hw_fault=0;pending=enabled=binary=0;buttons=0;tx_count=0;tx[0]=0;
    usb_connected=1;usb_epoch=1;
    memset(oled,0,sizeof oled);app_init();
}
int main(void){
    reset();
    assert(!enabled&&!status().armed&&sample_us==5000);
    app_command("ARM");assert(!status().armed);
    advance(1000,0,0,24);
    app_command("ARM");assert(!status().armed); /* HELLO is a session handshake. */
    app_command("HELLO");
    app_command("ARM");assert(status().armed);
    advance(1000,0,0,24);assert(enabled&&isfinite(duty)&&duty>=0&&duty<=1);
    app_command("SET sampling_us 2000");assert(sample_us==5000);
    app_command("SET reference 6");assert(status().reference==6);
    app_command("SET reference NaN");assert(status().reference==6);
    app_command("SET reference inf");assert(status().reference==6);
    app_command("SET reference 10 rubbish");assert(status().reference==6);
    app_command("SET reference 29");assert(status().reference==6);
    app_command("STOP");assert(!enabled&&!status().armed&&status().duty==0);
    app_command("SET sampling_us 2000");assert(sample_us==2000);
    app_command("SET sampling_us 1000.5");assert(sample_us==2000);
    app_command("SET mode 1");
    advance(2000,0,0,24);app_command("ARM");advance(2000,0,0,24);
    assert(enabled&&binary&&(duty==0||duty==1));
    advance(2000,31,1,24);
    assert(!enabled&&(status().fault&APP_FAULT_OVERVOLTAGE));
    app_command("CLEAR");assert(status().fault);
    advance(2000,1,1,24);app_command("CLEAR");assert(!status().fault&&!enabled);
    app_command("ARM");assert(status().armed);
    advance(2000001,1,1,24);
    assert(!enabled&&(status().fault&APP_FAULT_COMMUNICATION));
    app_command("PING");assert(!enabled&&!status().armed);
    app_command("CLEAR");assert(!status().fault);
    app_command("ARM");clock_us+=7000;app_poll();
    assert(!enabled&&(status().fault&APP_FAULT_STALE));
    advance(2000,1,1,24);app_command("CLEAR");app_command("ARM");
    advance(2000,NAN,1,24);assert(!enabled&&(status().fault&APP_FAULT_SENSOR));
    advance(10000,NAN,1,24);assert(strstr(tx,"\"v1\":null"));
    advance(2000,1,1,24);app_command("CLEAR");
    app_command("SET gain1 1.1");app_command("ARM");assert(!status().armed);
    advance(2000,1,1,24);app_command("ARM");assert(status().armed);
    hw_fault=2;advance(2000,1,1,24);assert(!enabled&&(status().fault&APP_FAULT_PLATFORM));
    app_command("CLEAR");assert(!hw_fault&&!status().fault);
    tx_count=0;tx[0]=0;app_command("GET");
    for(int i=0;i<80;++i)advance(1000,1,1,24);
    assert(strstr(tx,"\"type\":\"param\""));
    assert(strstr(tx,"\"name\":\"sampling_us\""));
    assert(strstr(tx,"\"name\":\"gainbus\""));
    assert(strstr(tx,"\"type\":\"params_end\""));
    assert(strstr(tx,"\"type\":\"sample\""));
    /* The default menu field is sampling_us: RIGHT changes it while stopped. */
    buttons=8;advance(250000,1,1,24);buttons=0;advance(1000,1,1,24);
    assert(sample_us==2500&&!enabled);
    /* LEFT+RIGHT always stops, it never arms. */
    app_command("ARM");assert(status().armed);
    buttons=12;advance(1000,1,1,24);assert(!enabled&&!status().armed);
    /* Wrap-safe unsigned times retain the watchdog semantics. */
    reset();clock_us=0xfffff000u;advance(1000,0,0,24);app_command("HELLO");app_command("ARM");
    for(int i=0;i<10;++i){advance(1000,1,1,24);app_command("PING");}
    assert(status().armed&&!status().fault);
    /* Unplug/reconnect inside the 2-second heartbeat cannot preserve arming. */
    usb_connected=0;app_poll();
    assert(!enabled&&!status().armed&&(status().fault&APP_FAULT_COMMUNICATION));
    usb_connected=1;++usb_epoch;app_command("PING");app_command("CLEAR");
    app_command("ARM");assert(!status().armed); /* Stale ARM without new HELLO. */
    app_command("HELLO");app_command("ARM");assert(status().armed);
    /* Even when no disconnected poll occurred, an epoch change stops the stage. */
    ++usb_epoch;app_command("PING");assert(!status().armed&&!enabled);
    advance(1000,1,1,24);app_command("CLEAR");app_command("ARM");assert(!status().armed);
    app_command("HELLO");app_command("ARM");assert(status().armed);
    puts("app integration: startup, protocol, calibration, all interlocks, menu and wrap tests passed");
    return 0;
}
