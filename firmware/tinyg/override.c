/* Manual override requests, committed feed and private-plan proof helpers. */
#include "override.h"
#include <math.h>
#include <string.h>
#ifdef __AVR
#include <avr/interrupt.h>
#define OV_LOCK uint8_t saved_sreg=SREG;cli()
#define OV_UNLOCK SREG=saved_sreg
#else
#define OV_LOCK ((void)0)
#define OV_UNLOCK ((void)0)
#endif
static volatile ovSnapshot_t ov;
static volatile uint32_t cancel_epoch;
static uint8_t utf_remaining,utf_lo,utf_hi; /* USB RX owns these while armed; arm/reset has the interrupt guard. */

void ov_init(void)
{
    OV_LOCK;
    memset((void *)&ov,0,sizeof(ov));
    ov.feed=ov.spindle=ov.committed_feed=ov.committed_spindle=OV_FACTOR_ONE;
    ov.revision=1;
    cancel_epoch=0;utf_remaining=0;utf_lo=0x80;utf_hi=0xBF;
    OV_UNLOCK;
}
void ov_snapshot(ovSnapshot_t *out)
{
    OV_LOCK;*out=ov;OV_UNLOCK;
}
static void _set(uint8_t which,uint16_t factor,uint8_t enables)
{
    uint16_t old=(which==OV_FEED)?ov.feed:ov.spindle;
    if (old!=factor || ov.enables!=enables) {
        if (which==OV_FEED) ov.feed=factor; else ov.spindle=factor;
        ov.enables=enables;ov.request++;ov.revision++;ov.state=OV_RECORD_ONLY;
    }
}
uint8_t ov_set_factor(uint8_t which,float factor,bool apply,bool permitted)
{
    if ((which!=OV_FEED && which!=OV_SPINDLE) || !isfinite(factor) || factor<0.05f || factor>2.0f) return OV_RANGE;
    if (!permitted) return OV_DENIED;
    uint16_t q=(uint16_t)(factor*1000.0f+0.5f); /* float work stays outside the interrupt guard */
    if (apply) {OV_LOCK;_set(which,q,ov.enables);OV_UNLOCK;}
    return OV_OK;
}
uint8_t ov_set_enable(uint8_t which,bool enable,bool apply,bool permitted)
{
    if (which!=OV_FEED && which!=OV_SPINDLE) return OV_RANGE;
    if (!permitted) return OV_DENIED;
    if (apply) {
        OV_LOCK;
        uint8_t enables=enable?(ov.enables|which):(ov.enables&~which);
        _set(which,which==OV_FEED?ov.feed:ov.spindle,enables);
        OV_UNLOCK;
    }
    return OV_OK;
}
uint8_t ov_arm(bool enable,bool quiet,bool permitted,bool apply)
{
    if (!permitted || !quiet) return OV_DENIED;
    if (apply) {
        OV_LOCK;
        ov.armed=enable;utf_remaining=0;utf_lo=0x80;utf_hi=0xBF;
        OV_UNLOCK;
    }
    return OV_OK;
}
/* Protect valid UTF-8 continuations BEFORE assigning any standalone Grbl byte a knob meaning. */
static bool _utf_data(uint8_t c)
{
    if (utf_remaining==255) { /* malformed UTF-8: do not reinterpret its tail as knobs */
        if(c=='\r' || c=='\n')utf_remaining=0;
        return c>=0x80;
    }
    if (utf_remaining) {
        if(c>=utf_lo && c<=utf_hi) {
            utf_remaining--;utf_lo=0x80;utf_hi=0xBF;return true;
        }
        utf_remaining=(c=='\r' || c=='\n')?0:255;
        return c>=0x80;
    }
    utf_lo=0x80;utf_hi=0xBF;
    if (c>=0xC2 && c<=0xDF) utf_remaining=1;
    else if (c>=0xE0 && c<=0xEF) {
        utf_remaining=2;if(c==0xE0)utf_lo=0xA0;if(c==0xED)utf_hi=0x9F;
    } else if (c>=0xF0 && c<=0xF4) {
        utf_remaining=3;if(c==0xF0)utf_lo=0x90;if(c==0xF4)utf_hi=0x8F;
    }
    if(c>=0xC0 && !utf_remaining)utf_remaining=255; /* invalid lead */
    return utf_remaining!=0;
}
bool ov_usb_armed(void) { return ov.armed != 0; }
bool ov_usb_byte(uint8_t c,bool permitted,bool good_frame)
{
    if (!ov.armed) return false;
    if (!good_frame) {utf_remaining=0;return false;}
    if (_utf_data(c)) return false;
    if (c>=0x95 && c<=0x97) {
        if (permitted) {
            if (ov.unsupported!=255) ov.unsupported++;
            ov.state=OV_UNSUPPORTED;
        }
        return true; /* Rapid overrides are explicitly unsupported, never aliases for feed. */
    }
    uint8_t which,command;
    if(c>=0x90 && c<=0x94){which=OV_FEED;command=c-0x90;}
    else if(c>=0x99 && c<=0x9D){which=OV_SPINDLE;command=c-0x99;}
    else return false;
    if (!permitted) return true;
    OV_LOCK;
    int16_t factor=(which==OV_FEED)?ov.feed:ov.spindle;
    if(command==0)factor=1000;
    else factor+=(command==1)?100:(command==2)?-100:(command==3)?10:-10;
    if(factor<OV_FACTOR_MIN)factor=OV_FACTOR_MIN;
    if(factor>OV_FACTOR_MAX)factor=OV_FACTOR_MAX;
    _set(which,(uint16_t)factor,ov.enables|which);
    OV_UNLOCK;
    return true;
}
/* Called inside exec's validated handoff; no floating-point work. */
void ov_commit_feed(uint16_t factor,bool capped)
{
    OV_LOCK;
    if(ov.committed_feed!=factor || ov.capped!=capped) {
        ov.committed_feed=factor;ov.capped=capped;ov.revision++;
    }
    OV_UNLOCK;
}
void ov_cancel(void)
{
    OV_LOCK;cancel_epoch++;ov.state=OV_CANCELLED;OV_UNLOCK;
}
static bool _boundary(const ovBoundary_t *b)
{
    if (!b || !b->eligible || b->slot>=32 ||
        (b->kind!=OV_BOUNDARY_BODY && b->kind!=OV_BOUNDARY_SECTION_END) ||
        !isfinite(b->velocity) || b->velocity<0 ||
        b->lead_ticks<OV_TICKET_MIN_TICKS || b->lead_ticks>OV_TICKET_MAX_TICKS) return false;
    for(uint8_t i=0;i<6;i++)if(!isfinite(b->position[i]))return false;
    return true;
}
/* Trusted runtime snapshot: capture only the request and stop generations. */
void ov_ticket_stamp(ovTicket_t *t,uint32_t now)
{
    t->created=now;
    OV_LOCK;t->request=ov.request;t->cancel_epoch=cancel_epoch;OV_UNLOCK;
}
bool ov_ticket_current(const ovTicket_t *t,uint32_t now)
{
    if (!t || (uint32_t)(now-t->created)>=t->boundary.lead_ticks) return false;
    OV_LOCK;bool current=t->request==ov.request && t->cancel_epoch==cancel_epoch;OV_UNLOCK;
    return current;
}
bool ov_ticket_make(const ovBoundary_t *b,uint32_t now,ovTicket_t *t)
{
    if (!t || !_boundary(b)) return false;
    t->boundary=*b;ov_ticket_stamp(t,now);
    return true;
}
bool ov_ticket_valid(const ovTicket_t *t,const ovBoundary_t *b,uint32_t now)
{
    if(!t || !_boundary(b) || (uint32_t)(now-t->created)>=t->boundary.lead_ticks) return false;
    return ov_ticket_current(t,now) && t->boundary.incarnation==b->incarnation && t->boundary.ordinal==b->ordinal &&
           t->boundary.stop_epoch==b->stop_epoch && t->boundary.slot==b->slot && t->boundary.kind==b->kind &&
           t->boundary.velocity==b->velocity && !memcmp(t->boundary.position,b->position,sizeof(b->position));
}
bool ov_window_valid(const ovPlan_t *p,const ovReference_t *r,uint8_t n,float entry,float exit)
{
    if(!p || !r || !n || n>OV_WINDOW_MAX || !isfinite(entry) || !isfinite(exit) || entry<0 || exit<0) return false;
    float prior=entry;
    for(uint8_t i=0;i<n;i++) {
        if(p[i].slot>=32 || p[i].slot!=r[i].slot || p[i].incarnation!=r[i].incarnation ||
           (i && p[i].slot!=(p[i-1].slot+1)%32) ||
           !isfinite(r[i].length) || r[i].length<0 || !isfinite(r[i].vmax) || r[i].vmax<0 ||
           !isfinite(p[i].entry) || !isfinite(p[i].cruise) || !isfinite(p[i].exit) ||
           p[i].entry!=prior || p[i].entry<0 || p[i].exit<0 ||
           p[i].cruise<p[i].entry || p[i].cruise<p[i].exit || p[i].cruise>r[i].vmax ||
           !isfinite(p[i].head) || !isfinite(p[i].body) || !isfinite(p[i].tail) ||
           p[i].head<0 || p[i].body<0 || p[i].tail<0) return false;
        float total=p[i].head+p[i].body+p[i].tail;
        if(!isfinite(total) || fabsf(total-r[i].length)>fmaxf(0.00001f,r[i].length*0.000002f))return false;
        prior=p[i].exit;
    }
    return prior==exit;
}

/* Called only on private snapshots in the main loop. The quintic velocity curve
 * has maximum |d2v/dt2| = (10/sqrt(3))*|delta-v|/T^2 < 6*|delta-v|/T^2.
 * This conservative duration bounds jerk for acceleration AND deceleration.
 * It does not alter the machine's jerk, axis limits, or boundary speeds. */
static float _transition_time(float from,float to,float jerk)
{
    return from==to ? 0 : sqrtf(6.0f*fabsf(to-from)/jerk);
}
bool ov_plan_transition(float length,float entry,float feed,float exit,float vmax,
                        float jerk,uint16_t factor,ovTransition_t *out)
{
    if(!out || !isfinite(length) || length<=0 || !isfinite(entry) || entry<0 ||
       !isfinite(feed) || feed<=0 || !isfinite(exit) || exit<0 ||
       !isfinite(vmax) || vmax<=0 || entry>vmax || exit>vmax ||
       !isfinite(jerk) || jerk<=0 || factor<OV_FACTOR_MIN || factor>OV_FACTOR_MAX)return false;
    ovTransition_t p;
    p.entry=entry;p.exit=exit;p.factor_q1000=factor;
    float requested=feed*(factor/1000.0f);
    if(!isfinite(requested))return false;
    p.capped=requested>vmax;p.cruise=p.capped?vmax:requested;
    p.head_time=_transition_time(entry,p.cruise,jerk);
    p.tail_time=_transition_time(p.cruise,exit,jerk);
    p.head=(entry+p.cruise)*0.5f*p.head_time;
    p.tail=(p.cruise+exit)*0.5f*p.tail_time;
    p.body=length-p.head-p.tail;
    if(!isfinite(p.body) || p.body<0)return false;
    *out=p;
    return true;
}
