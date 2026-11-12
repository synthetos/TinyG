/* Manual override requests and private future-boundary plans. */
#ifndef TINYG_OVERRIDE_H
#define TINYG_OVERRIDE_H
#include <stdbool.h>
#include <stdint.h>
#define OV_FEED 1
#define OV_SPINDLE 2
#define OV_WINDOW_MAX 6
#define OV_FACTOR_ONE 1000
#define OV_FACTOR_MIN 50
#define OV_FACTOR_MAX 2000
#define OV_TICKET_MIN_TICKS 21UL
#define OV_TICKET_MAX_TICKS 64UL
/* Wire publication remains separate from private motion-plan acceptance. */
enum { OV_RECORD_ONLY=0, OV_CANCELLED=1, OV_UNSUPPORTED=2 };
enum { OV_OK=0, OV_RANGE=1, OV_DENIED=2 };
enum { OV_BOUNDARY_HEAD=0, OV_BOUNDARY_TAIL, OV_BOUNDARY_BODY, OV_BOUNDARY_SECTION_END };
typedef struct {
    uint16_t feed,spindle,committed_feed,committed_spindle;
    uint32_t request,revision;
    uint8_t enables,armed,state,unsupported,capped,program_feed;
} ovSnapshot_t;
typedef struct {
    float position[6],velocity;
    uint32_t incarnation,ordinal,stop_epoch,lead_ticks;
    uint8_t slot,kind;
    bool eligible;
} ovBoundary_t;
typedef struct {
    ovBoundary_t boundary;
    uint32_t request,cancel_epoch,created;
} ovTicket_t;
typedef struct {
    float entry,cruise,exit,head,body,tail,naive;
    uint32_t incarnation;
    uint16_t factor_q1000;
    uint8_t slot;
} ovPlan_t;
typedef struct {float length,vmax;uint32_t incarnation;uint8_t slot;} ovReference_t;
/* Main-loop-only private plan; entry/exit are unchanged outer boundary speeds.
 * Times are minutes, speeds mm/min, jerk mm/min^3. No runtime writes. */
typedef struct {
    float entry,cruise,exit,head,body,tail,head_time,tail_time;
    uint16_t factor_q1000;
    bool capped;
} ovTransition_t;
bool ov_plan_transition(float length,float entry,float feed,float exit,float vmax,
                        float jerk,uint16_t factor,ovTransition_t *out);
void ov_init(void);
void ov_snapshot(ovSnapshot_t *out);
uint8_t ov_set_factor(uint8_t which,float factor,bool apply,bool permitted);
uint8_t ov_set_enable(uint8_t which,bool enable,bool apply,bool permitted);
uint8_t ov_arm(bool enable,bool quiet,bool permitted,bool apply);
bool ov_usb_armed(void);
bool ov_usb_byte(uint8_t c,bool permitted,bool good_frame);
void ov_cancel(void);
void ov_set_program_feed(bool enable);
void ov_reset_job(void);
void ov_commit_feed(uint16_t factor,bool capped);
/* Caller supplies a private snapshot of an analytical future boundary.
 * Stamp/current only check request, cancellation and age; the exec consumer
 * also checks the live block, segment, speed and position before accepting it. */
void ov_ticket_stamp(ovTicket_t *ticket,uint32_t now);
bool ov_ticket_current(const ovTicket_t *ticket,uint32_t now);
bool ov_ticket_make(const ovBoundary_t *boundary,uint32_t now,ovTicket_t *ticket);
bool ov_ticket_valid(const ovTicket_t *ticket,const ovBoundary_t *boundary,uint32_t now);
/* Structural composition check ONLY: not a jerk/acceleration proof or publication routine. */
bool ov_window_valid(const ovPlan_t *plans,const ovReference_t *refs,uint8_t count,float entry,float exit);
#endif
