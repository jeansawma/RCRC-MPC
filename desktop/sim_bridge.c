/* Desktop plant simulation uses the exact firmware control implementation. */
#include "control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

typedef struct {
    Controller controller;
    ControlConfig cfg;
    ControlResult result;
    float v1, v2, bus;
    uint32_t time_us;
    unsigned armed, fault;
} Simulator;

static void reply(char *out, size_t capacity, const char *format, ...) {
    if (!capacity) return;
    va_list args;
    va_start(args, format);
    vsnprintf(out, capacity, format, args);
    va_end(args);
}

void *sim_create(void) {
    Simulator *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    ctrl_defaults(&s->cfg);
    if (!ctrl_init(&s->controller, &s->cfg)) { free(s); return NULL; }
    s->bus = 24.0f;
    return s;
}

void sim_destroy(void *handle) { free(handle); }
uint32_t sim_sampling_us(void *handle) {
    Simulator *s = handle;
    return (uint32_t)s->cfg.sampling_us;
}

int sim_command(void *handle, const char *command, char *out, size_t capacity) {
    Simulator *s = handle;
    if (!s || !out || !capacity) return 0;
    out[0] = 0;
    if (!strcmp(command,"HELLO")) {
        reply(out,capacity,"{\"type\":\"hello\",\"protocol\":1,\"board\":\"DEMO using firmware C controllers\"}\n");
    } else if (!strcmp(command,"GET")) {
        size_t used=0;
        for (size_t i=0;i<ctrl_param_count;i++) {
            const ControlParam *p=&ctrl_params[i];
            float value=0;
            ctrl_config_get(&s->cfg,p->name,&value);
            int n=snprintf(out+used,capacity-used,
                "{\"type\":\"param\",\"name\":\"%s\",\"value\":%.9g,\"min\":%.9g,\"max\":%.9g,\"unit\":\"%s\",\"integer\":%s}\n",
                p->name,(double)value,(double)p->minimum,(double)p->maximum,p->unit,p->integer?"true":"false");
            if(n<0 || (size_t)n>=capacity-used) { out[0]=0; return 0; }
            used+=(size_t)n;
        }
        reply(out+used,capacity-used,"{\"type\":\"params_end\"}\n");
    } else if (!strncmp(command,"SET ",4)) {
        char name[40], value_text[64], extra;
        if(sscanf(command+4,"%39s %63s %c",name,value_text,&extra)!=2) {
            reply(out,capacity,"{\"type\":\"error\",\"message\":\"SET requires name and value\"}\n"); return 1;
        }
        char *end;
        float value=strtof(value_text,&end);
        if(*end || !isfinite(value)) {
            reply(out,capacity,"{\"type\":\"error\",\"message\":\"finite numeric value required\"}\n");return 1;
        }
        if(s->armed && strcmp(name,"reference")) {
            reply(out,capacity,"{\"type\":\"error\",\"message\":\"STOP before changing this parameter\"}\n");return 1;
        }
        ControlConfig next=s->cfg;
        char reason[128];
        if(!ctrl_config_set(&next,name,value) || !ctrl_validate(&next,reason,sizeof(reason))) {
            reply(out,capacity,"{\"type\":\"error\",\"message\":\"invalid or inconsistent parameter\"}\n");return 1;
        }
        if(!strcmp(name,"reference") && s->armed) {
            s->cfg=next; s->controller.cfg.reference=value;
        } else {
            if(!ctrl_init(&s->controller,&next)) {
                reply(out,capacity,"{\"type\":\"error\",\"message\":\"model initialization failed\"}\n");return 1;
            }
            s->cfg=next;
        }
        reply(out,capacity,"{\"type\":\"ack\",\"command\":\"SET\",\"name\":\"%s\",\"value\":%.9g}\n",name,(double)value);
    } else if (!strcmp(command,"ARM")) {
        if(s->fault || s->bus > s->cfg.max_bus || s->v1>s->cfg.max_voltage || s->v2>s->cfg.max_voltage) {
            reply(out,capacity,"{\"type\":\"error\",\"message\":\"fault or voltage limit prevents ARM\"}\n");return 1;
        }
        ctrl_reset(&s->controller);s->armed=1;
        reply(out,capacity,"{\"type\":\"ack\",\"command\":\"ARM\"}\n");
    } else if (!strcmp(command,"STOP")) {
        s->armed=0;s->result.duty=0;
        reply(out,capacity,"{\"type\":\"ack\",\"command\":\"STOP\"}\n");
    } else if (!strcmp(command,"CLEAR")) {
        if(!s->armed) s->fault=0;
        reply(out,capacity,"{\"type\":\"ack\",\"command\":\"CLEAR\"}\n");
    } else if (!strcmp(command,"PING")) {
        reply(out,capacity,"{\"type\":\"ack\",\"command\":\"PING\"}\n");
    } else {
        reply(out,capacity,"{\"type\":\"error\",\"message\":\"unknown command\"}\n");
    }
    return 1;
}

int sim_advance(void *handle,uint32_t steps,char *out,size_t capacity) {
    Simulator *s=handle;
    for(uint32_t i=0;i<steps;i++) {
        float input=0;
        if(s->armed) {
            if(ctrl_step(&s->controller,s->v1,s->v2,s->bus,&s->result)!=CTRL_OK) {
                s->fault=1;s->armed=0;s->result.duty=0;
            } else {
                input=s->cfg.low_drop+s->result.duty*(s->bus-s->cfg.high_drop-s->cfg.low_drop);
            }
        }
        ctrl_predict(&s->controller,s->v1,s->v2,input,&s->v1,&s->v2);
        s->time_us+=(uint32_t)s->cfg.sampling_us;
        if(s->v1>s->cfg.max_voltage || s->v2>s->cfg.max_voltage) {
            s->fault=2;s->armed=0;s->result.duty=0;
        }
    }
    reply(out,capacity,
        "{\"type\":\"sample\",\"t_us\":%lu,\"v1\":%.7g,\"v2\":%.7g,\"vbus\":%.7g,\"reference\":%.7g,\"duty\":%.7g,\"mode\":%u,\"armed\":%u,\"fault\":%u,\"qp_residual\":%.7g,\"qp_iterations\":%lu}\n",
        (unsigned long)s->time_us,(double)s->v1,(double)s->v2,(double)s->bus,
        (double)s->cfg.reference,(double)s->result.duty,(unsigned)s->cfg.mode,s->armed,s->fault,
        (double)s->result.qp_residual,(unsigned long)s->result.iterations);
    return 1;
}
