#include "app.h"
#include "board.h"
#include "control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define COMMAND_TIMEOUT_US 2000000u
#define TELEMETRY_PERIOD_US 10000u
#define SENSOR_MIN_DEADLINE_US 5000u
#define SENSOR_MIN_BUS 1.0f
#define CAL_COUNT 6u

static Controller controller;
static ControlResult result;
static AppStatus state;
static uint32_t last_sample_us,last_command_us,last_telemetry_us,last_oled_ms;
static uint32_t last_button_ms;
static uint8_t last_buttons,have_sample;
static uint8_t hello_ready;
static uint32_t usb_session;
static int param_cursor=-1;
static size_t menu_index;
static float calibration[CAL_COUNT]={1,1,1,0,0,0};
static const char *const cal_names[CAL_COUNT]={
    "gain1","gain2","gainbus","offset1","offset2","offsetbus"};

static uint32_t sensor_deadline(void) {
    uint32_t d=(uint32_t)controller.cfg.sampling_us*3u;
    return d>SENSOR_MIN_DEADLINE_US?d:SENSOR_MIN_DEADLINE_US;
}
static int send_line(const char *text) {
    return board_usb_write(text,strlen(text));
}
static void error(const char *message) {
    char text[192];
    /* Callers pass fixed local strings, never interpolated input. */
    (void)snprintf(text,sizeof text,"{\"type\":\"error\",\"message\":\"%s\"}\n",message);
    (void)send_line(text);
}
static void acknowledge(const char *command) {
    char text[96];
    (void)snprintf(text,sizeof text,"{\"type\":\"ack\",\"command\":\"%s\"}\n",command);
    (void)send_line(text);
}
static void stop(void) {
    state.armed=0;
    state.duty=0;
    board_set_output(0,0,0);
    ctrl_reset(&controller);
}
static void trip(uint32_t bits) {
    state.fault|=bits;
    stop();
}
static int refresh_usb_session(void) {
    uint32_t session=board_usb_session();
    int connected=board_usb_connected();
    if (session!=usb_session || !connected) {
        if (state.armed) trip(APP_FAULT_COMMUNICATION);
        usb_session=session;
        hello_ready=0;
        param_cursor=-1;
    }
    return connected && hello_ready;
}
static uint32_t measurements_fault(void) {
    if (!have_sample || !isfinite(state.v1) || !isfinite(state.v2) ||
        !isfinite(state.vbus) || state.v1<-.5f || state.v2<-.5f || state.vbus<-.5f)
        return APP_FAULT_SENSOR;
    if (state.v1>controller.cfg.max_voltage || state.v2>controller.cfg.max_voltage ||
        state.vbus>controller.cfg.max_bus) return APP_FAULT_OVERVOLTAGE;
    return 0;
}
static int parameter(size_t index,const char **name,const char **unit,
                     float *value,float *minimum,float *maximum,int *integer) {
    if (index<ctrl_param_count) {
        const ControlParam *p=&ctrl_params[index];
        *name=p->name; *unit=p->unit; *minimum=p->minimum; *maximum=p->maximum;
        *integer=p->integer;
        return ctrl_config_get(&controller.cfg,p->name,value);
    }
    index-=ctrl_param_count;
    if (index>=CAL_COUNT) return 0;
    *name=cal_names[index]; *unit=index<3?"ratio":"V"; *value=calibration[index];
    *minimum=index<3?.5f:-5; *maximum=index<3?2:5; *integer=0;
    return 1;
}
static int update_parameter(const char *name,float value) {
    ControlConfig next=controller.cfg;
    char reason[96];
    size_t k;
    if (!isfinite(value)) { error("value must be finite"); return 0; }
    if (state.armed && strcmp(name,"reference")) {
        error("STOP before changing this parameter"); return 0;
    }
    for (k=0;k<CAL_COUNT;++k) if (!strcmp(name,cal_names[k])) {
        float minimum=k<3?.5f:-5,maximum=k<3?2:5;
        if (value<minimum || value>maximum) { error("calibration out of range"); return 0; }
        calibration[k]=value;
        have_sample=0; /* A fresh sample must use the new calibration before ARM. */
        return 1;
    }
    if (!ctrl_config_set(&next,name,value)) { error("unknown parameter or value out of range"); return 0; }
    if (!ctrl_validate(&next,reason,sizeof reason)) { error("parameter combination is invalid"); return 0; }
    if (state.armed) {
        float available=controller.cfg.low_drop+
            controller.cfg.duty_max*(state.vbus-controller.cfg.high_drop-controller.cfg.low_drop);
        if (value>available) { error("reference exceeds available bridge voltage"); return 0; }
        controller.cfg.reference=next.reference;
    } else {
        /* No PWM is active while rebuilding the model/QP workspace. */
        if (!strcmp(name,"sampling_us") && !board_set_sample_us((uint32_t)next.sampling_us)) {
            error("platform rejected sampling period"); return 0;
        }
        if (!ctrl_init(&controller,&next)) {
            trip(APP_FAULT_CONFIG); error("controller initialization failed"); return 0;
        }
    }
    state.reference=controller.cfg.reference;
    return 1;
}
static const char *skip_space(const char *p) {
    while (*p && isspace((unsigned char)*p)) ++p;
    return p;
}
static int token(const char **p,char *out,size_t cap) {
    size_t n=0;
    *p=skip_space(*p);
    while (**p && !isspace((unsigned char)**p)) {
        if (n+1>=cap) return 0;
        out[n++]=*(*p)++;
    }
    out[n]=0;
    return n!=0;
}
void app_command(const char *line) {
    char command[16],name[32],text[192];
    const char *p=line;
    char *end;
    float value;
    (void)refresh_usb_session();
    if (!line || !token(&p,command,sizeof command)) { error("invalid command"); return; }
    p=skip_space(p);
    if (!strcmp(command,"SET")) {
        if (!token(&p,name,sizeof name)) { error("expected SET name value"); return; }
        p=skip_space(p);
        value=strtof(p,&end);
        if (end==p || *skip_space(end)) { error("expected one numeric value"); return; }
        if (!update_parameter(name,value)) return;
        (void)snprintf(text,sizeof text,
            "{\"type\":\"ack\",\"command\":\"SET\",\"name\":\"%s\",\"value\":%.9g}\n",name,(double)value);
        (void)send_line(text);
    } else {
        if (*p) { error("unexpected arguments"); return; }
        if (!strcmp(command,"STOP")) { stop(); acknowledge(command); }
        else if (!strcmp(command,"PING")) acknowledge(command);
        else if (!strcmp(command,"HELLO")) {
            if (!board_usb_connected()) { error("USB session is disconnected"); return; }
            hello_ready=1;
            (void)send_line("{\"type\":\"hello\",\"protocol\":1,\"device\":\"RCRC STM32H723\",\"firmware\":\"1.0\",\"transport\":\"native USB CDC\"}\n");
        }
        else if (!strcmp(command,"GET")) param_cursor=0;
        else if (!strcmp(command,"ARM")) {
            uint32_t now=board_micros();
            if (!refresh_usb_session()) { error("HELLO required for the current USB session"); return; }
            if (state.fault) { error("latched fault: remove cause then CLEAR"); return; }
            if (board_faults()) { trip(APP_FAULT_PLATFORM); error("platform fault"); return; }
            if (measurements_fault()) { error("invalid or overvoltage sensor reading"); return; }
            if ((uint32_t)(now-last_sample_us)>sensor_deadline()) { error("fresh ADC samples required"); return; }
            if (state.vbus<SENSOR_MIN_BUS) { error("DC bus below 1 V"); return; }
            if (state.v1>state.vbus+1 || state.v2>state.vbus+1) { error("state voltage above bus: wait for discharge"); return; }
            float available=controller.cfg.low_drop+controller.cfg.duty_max*
                (state.vbus-controller.cfg.high_drop-controller.cfg.low_drop);
            if (controller.cfg.reference>available) { error("reference exceeds available bridge voltage"); return; }
            ctrl_reset(&controller);
            state.armed=1;
            acknowledge(command);
        } else if (!strcmp(command,"CLEAR")) {
            if (state.armed) { error("STOP before CLEAR"); return; }
            if (measurements_fault() || (uint32_t)(board_micros()-last_sample_us)>sensor_deadline()) {
                error("fault condition remains or sensors stale"); return;
            }
            board_clear_faults();
            if (board_faults()) { error("platform fault remains; reset board after diagnosis"); return; }
            state.fault=0;
            acknowledge(command);
        } else { error("unknown command"); return; }
    }
    last_command_us=board_micros();
}
static void stream_parameter(void) {
    char text[256];
    const char *name,*unit;
    float value,minimum,maximum;
    int integer;
    if (param_cursor<0) return;
    if (!parameter((size_t)param_cursor,&name,&unit,&value,&minimum,&maximum,&integer)) {
        if (send_line("{\"type\":\"params_end\"}\n")) param_cursor=-1;
        return;
    }
    (void)snprintf(text,sizeof text,
        "{\"type\":\"param\",\"name\":\"%s\",\"value\":%.9g,\"min\":%.9g,\"max\":%.9g,\"unit\":\"%s\",\"integer\":%s}\n",
        name,(double)value,(double)minimum,(double)maximum,unit,integer?"true":"false");
    if (send_line(text)) ++param_cursor;
}
static void sample(void) {
    float v1,v2,bus;
    if (!board_sample_ready(&v1,&v2,&bus)) return;
    last_sample_us=board_micros();
    ++state.sample_count;
    state.v1=v1*calibration[0]+calibration[3];
    state.v2=v2*calibration[1]+calibration[4];
    state.vbus=bus*calibration[2]+calibration[5];
    have_sample=1;
    uint32_t f=measurements_fault();
    if (f) { trip(f); return; }
    if (state.armed && state.vbus<SENSOR_MIN_BUS) { trip(APP_FAULT_BUS); return; }
    if (!state.armed) { board_set_output(0,0,0); return; }
    if (!refresh_usb_session()) { trip(APP_FAULT_COMMUNICATION); return; }
    if (ctrl_step(&controller,state.v1,state.v2,state.vbus,&result)!=CTRL_OK ||
        !isfinite(result.duty) || result.duty<0 || result.duty>1) {
        trip(APP_FAULT_CONTROL); return;
    }
    /* USB IRQs may change the session while the QP is being solved. */
    if (!refresh_usb_session()) { trip(APP_FAULT_COMMUNICATION); return; }
    state.duty=result.duty;
    board_set_output(result.duty,1,(int)controller.cfg.mode==CTRL_FCS);
}
static void telemetry(uint32_t now) {
    char text[384];
    if ((uint32_t)(now-last_telemetry_us)<TELEMETRY_PERIOD_US) return;
    last_telemetry_us=now;
    /* JSON has no NaN/Inf. Invalid sensors are represented by null. */
    char a[24],b[24],c[24];
    if (isfinite(state.v1)) (void)snprintf(a,sizeof a,"%.6g",(double)state.v1); else strcpy(a,"null");
    if (isfinite(state.v2)) (void)snprintf(b,sizeof b,"%.6g",(double)state.v2); else strcpy(b,"null");
    if (isfinite(state.vbus)) (void)snprintf(c,sizeof c,"%.6g",(double)state.vbus); else strcpy(c,"null");
    (void)snprintf(text,sizeof text,
        "{\"type\":\"sample\",\"t_us\":%lu,\"v1\":%s,\"v2\":%s,\"vbus\":%s,\"reference\":%.6g,\"duty\":%.6g,\"mode\":%d,\"armed\":%lu,\"fault\":%lu,\"qp_residual\":%.6g,\"qp_iterations\":%lu,\"sample_count\":%lu,\"platform_fault\":%lu}\n",
        (unsigned long)now,a,b,c,(double)state.reference,(double)state.duty,
        (int)controller.cfg.mode,(unsigned long)state.armed,(unsigned long)state.fault,
        (double)(isfinite(result.qp_residual)?result.qp_residual:0),
        (unsigned long)result.iterations,(unsigned long)state.sample_count,(unsigned long)board_faults());
    (void)send_line(text);
}
static void menu(uint32_t now_ms) {
    const char *name,*unit;
    float value,minimum,maximum;
    int integer;
    uint8_t buttons=board_buttons();
    uint8_t pressed=(uint8_t)(buttons & (uint8_t)~last_buttons);
    if (buttons && (uint32_t)(now_ms-last_button_ms)>=250u) pressed=buttons;
    last_buttons=buttons;
    if ((buttons&12u)==12u) { stop(); pressed=0; }
    if (pressed) {
        last_button_ms=now_ms;
        size_t n=ctrl_param_count+CAL_COUNT;
        if (pressed&1u) menu_index=(menu_index+n-1u)%n;
        if (pressed&2u) menu_index=(menu_index+1u)%n;
        if ((pressed&12u) && parameter(menu_index,&name,&unit,&value,&minimum,&maximum,&integer)) {
            float step=integer?1.0f:fmaxf(fabsf(value)*.05f,.001f);
            if (!strcmp(name,"reference")) step=.1f;
            if (!strcmp(name,"sampling_us")) step=500;
            if (!strcmp(unit,"F")) step=fmaxf(value*.05f,1e-7f);
            float next=value+((pressed&8u)?step:-step);
            if (next<minimum) next=minimum;
            if (next>maximum) next=maximum;
            (void)update_parameter(name,next);
        }
    }
    if ((uint32_t)(now_ms-last_oled_ms)<200u) return;
    last_oled_ms=now_ms;
    char lines[8][22]; const char *ptrs[8];
    static const char *const modes[]={"PI/PID","FCS-MPC","CS-MPC QP"};
    (void)snprintf(lines[0],22,"RCRC %s",modes[(int)controller.cfg.mode]);
    (void)snprintf(lines[1],22,"%s FAULT:%lu",state.armed?"RUN":"STOP",(unsigned long)state.fault);
    (void)snprintf(lines[2],22,"Out %.2f Ref %.2f",(double)state.v2,(double)state.reference);
    (void)snprintf(lines[3],22,"C1 %.2f Bus %.2f",(double)state.v1,(double)state.vbus);
    (void)snprintf(lines[4],22,"D %.3f Ts %.0fus",(double)state.duty,(double)controller.cfg.sampling_us);
    if (parameter(menu_index,&name,&unit,&value,&minimum,&maximum,&integer)) {
        (void)snprintf(lines[5],22,">%.19s",name);
        (void)snprintf(lines[6],22,"%.6g %.8s",(double)value,unit);
    } else { lines[5][0]=0; lines[6][0]=0; }
    (void)snprintf(lines[7],22,"U/D:menu L/R:edit");
    for (unsigned i=0;i<8;++i) ptrs[i]=lines[i];
    board_oled_lines(ptrs);
}
void app_init(void) {
    ControlConfig cfg;
    ctrl_defaults(&cfg);
    memset(&state,0,sizeof state); memset(&result,0,sizeof result);
    calibration[0]=calibration[1]=calibration[2]=1;
    calibration[3]=calibration[4]=calibration[5]=0;
    have_sample=0; param_cursor=-1; menu_index=0; last_buttons=0;
    hello_ready=0; usb_session=board_usb_session();
    last_sample_us=last_command_us=last_telemetry_us=board_micros();
    last_oled_ms=last_button_ms=board_millis();
    state.reference=cfg.reference;
    board_set_output(0,0,0);
    if (!ctrl_init(&controller,&cfg) || !board_set_sample_us((uint32_t)cfg.sampling_us)) trip(APP_FAULT_CONFIG);
}
void app_poll(void) {
    char line[128];
    (void)refresh_usb_session();
    if (board_faults()) trip(APP_FAULT_PLATFORM);
    sample();
    uint32_t now=board_micros();
    if (state.armed && (uint32_t)(now-last_sample_us)>sensor_deadline()) trip(APP_FAULT_STALE);
    if (state.armed && (uint32_t)(now-last_command_us)>COMMAND_TIMEOUT_US) trip(APP_FAULT_COMMUNICATION);
    board_poll();
    (void)refresh_usb_session();
    if (board_usb_readline(line,sizeof line)>0) app_command(line);
    stream_parameter();
    telemetry(now);
    menu(board_millis());
}
void app_status(AppStatus *out) { if (out) *out=state; }
