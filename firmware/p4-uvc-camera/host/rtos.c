/*
 * A deterministic stand-in for FreeRTOS, on virtual time.
 *
 * Every firmware task is a pthread, but only one runs at a time and only when
 * the scheduler hands it the baton, so a run is exactly repeatable. Time only
 * moves when every task is blocked: it then jumps to the next timeout or
 * device timer. Code takes no time unless a stand-in says so (host_sleep_us).
 *
 * Scheduling: highest priority first, first-come among equals. A task runs
 * until it blocks; giving a semaphore, sending to a queue or notifying a task
 * hands over at once if that readied a higher-priority task, as FreeRTOS
 * would. There is no time slicing and no preemption at arbitrary points, so
 * races that need an interrupt in the middle of plain code are not explored.
 * Stack sizes are not modelled.
 *
 * The host program drives it with host_run(until): it returns when virtual
 * time reaches `until`, or earlier when a stand-in has an event for the host.
 */
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "host_internal.h"

#define MAX_TASKS 8
#define TICK_US (1000000 / CONFIG_FREERTOS_HZ)
enum { T_READY, T_BLOCKED, T_HOST, T_DONE };

struct host_task {
    char name[16];
    int prio;
    TaskFunction_t fn;
    void *arg;
    pthread_t th;
    pthread_cond_t cv;
    int state;
    int64_t wake_us;          /* timeout while blocked */
    const void *obj;          /* what it is blocked on */
    int result;               /* 1 woken, 0 timed out */
    uint64_t order;           /* first-come order among equals */
    uint32_t notify;
};

static pthread_mutex_t s_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_sched_cv = PTHREAD_COND_INITIALIZER;
static struct host_task s_tasks[MAX_TASKS];
static int s_ntasks;
static struct host_task *s_cur;
static int64_t s_now_us;
static uint64_t s_order;
static int s_event_pending;
static struct { int64_t at; host_timer_fn fn; } s_timers[2] = {{-1, NULL}, {-1, NULL}};

fw_event_t host_event;

void host_lock(void)   { pthread_mutex_lock(&s_mu); }
void host_unlock(void) { pthread_mutex_unlock(&s_mu); }
int64_t host_now_us(void) { return s_now_us; }
void host_event_post(void) { s_event_pending = 1; }

/* ---- log and violations -------------------------------------------------- */
#define LOG_LINES 1024
#define LOG_LEN 200
static char s_log[LOG_LINES][LOG_LEN];
static int s_log_n;
#define VIOL_LINES 64
static char s_viol[VIOL_LINES][LOG_LEN];
static int s_viol_n;

void host_log(char level, const char *tag, const char *fmt, ...)
{
    if (level == 'D') {
        return;
    }
    char *line = s_log[s_log_n % LOG_LINES];
    int n = snprintf(line, LOG_LEN, "%9.3f ms %c %s: ", s_now_us / 1000.0, level, tag);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line + n, LOG_LEN - n, fmt, ap);
    va_end(ap);
    s_log_n++;
}

void host_violation(const char *fmt, ...)
{
    if (s_viol_n < VIOL_LINES) {
        char *line = s_viol[s_viol_n];
        int n = snprintf(line, LOG_LEN, "%.3f ms: ", s_now_us / 1000.0);
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(line + n, LOG_LEN - n, fmt, ap);
        va_end(ap);
    }
    s_viol_n++;
}

int host_log_count(void) { return s_log_n < LOG_LINES ? s_log_n : LOG_LINES; }
const char *host_log_line(int i) { return s_log[(s_log_n < LOG_LINES ? i : s_log_n + i) % LOG_LINES]; }
int host_violation_count(void) { return s_viol_n; }
const char *host_violation_line(int i) { return i < VIOL_LINES ? s_viol[i] : "(more violations not kept)"; }

/* ---- scheduler ------------------------------------------------------------ */
static void switch_out(struct host_task *self)
{
    s_cur = NULL;
    pthread_cond_signal(&s_sched_cv);
    while (s_cur != self) {
        pthread_cond_wait(&self->cv, &s_mu);
    }
}

static void *task_thread(void *arg)
{
    struct host_task *self = arg;
    pthread_mutex_lock(&s_mu);
    while (s_cur != self) {
        pthread_cond_wait(&self->cv, &s_mu);
    }
    self->fn(self->arg);
    self->state = T_DONE;
    s_cur = NULL;
    pthread_cond_signal(&s_sched_cv);
    pthread_mutex_unlock(&s_mu);
    return NULL;
}

static struct host_task *pick(int state, const void *obj)
{
    struct host_task *best = NULL;
    for (int i = 0; i < s_ntasks; i++) {
        struct host_task *t = &s_tasks[i];
        if (t->state != state || (obj && t->obj != obj)) {
            continue;
        }
        if (!best || t->prio > best->prio || (t->prio == best->prio && t->order < best->order)) {
            best = t;
        }
    }
    return best;
}

int host_block(const void *obj, int64_t timeout_us)
{
    struct host_task *self = s_cur;
    if (!self) {
        host_violation("a blocking call was made outside any task");
        return 0;
    }
    self->state = T_BLOCKED;
    self->obj = obj;
    self->wake_us = timeout_us < 0 ? INT64_MAX : s_now_us + timeout_us;
    self->order = ++s_order;
    switch_out(self);
    return self->result;
}

void host_sleep_us(int64_t us)
{
    if (us > 0 && s_cur) {
        host_block(s_cur, us);         /* nothing wakes a task blocked on itself: it times out */
    }
}

int host_wake(const void *obj)
{
    struct host_task *t = pick(T_BLOCKED, obj);
    if (!t) {
        return 0;
    }
    t->state = T_READY;
    t->result = 1;
    t->order = ++s_order;
    return 1;
}

static void preempt_check(void)
{
    struct host_task *self = s_cur, *t = pick(T_READY, NULL);
    if (self && t && t != self && t->prio > self->prio) {
        self->state = T_READY;
        self->order = ++s_order;
        switch_out(self);
    }
}

void host_wait_for_host(void)
{
    struct host_task *self = s_cur;
    self->state = T_HOST;
    s_event_pending = 1;
    switch_out(self);
}

void host_resume_from_host(void)
{
    for (int i = 0; i < s_ntasks; i++) {
        if (s_tasks[i].state == T_HOST) {
            s_tasks[i].state = T_READY;
            s_tasks[i].order = ++s_order;
        }
    }
}

void host_timer_set(int slot, int64_t at_us, host_timer_fn fn)
{
    s_timers[slot].at = at_us;
    s_timers[slot].fn = fn;
}

/* Run until virtual time `until_us`. Returns 1 when host_event holds an event for the host. Caller holds the lock. */
int host_run(int64_t until_us)
{
    for (;;) {
        if (s_event_pending) {
            s_event_pending = 0;
            host_event.t_us = s_now_us;
            return 1;
        }
        struct host_task *t = pick(T_READY, NULL);
        if (t) {
            s_cur = t;
            pthread_cond_signal(&t->cv);
            while (s_cur) {
                pthread_cond_wait(&s_sched_cv, &s_mu);
            }
            continue;
        }
        int64_t next = INT64_MAX;
        for (int i = 0; i < s_ntasks; i++) {
            if (s_tasks[i].state == T_BLOCKED && s_tasks[i].wake_us < next) {
                next = s_tasks[i].wake_us;
            }
        }
        for (int i = 0; i < 2; i++) {
            if (s_timers[i].at >= 0 && s_timers[i].at < next) {
                next = s_timers[i].at;
            }
        }
        if (next > until_us) {
            if (until_us > s_now_us) {
                s_now_us = until_us;
            }
            return 0;
        }
        if (next > s_now_us) {
            s_now_us = next;
        }
        for (int i = 0; i < s_ntasks; i++) {
            if (s_tasks[i].state == T_BLOCKED && s_tasks[i].wake_us <= s_now_us) {
                s_tasks[i].state = T_READY;
                s_tasks[i].result = 0;
                s_tasks[i].order = ++s_order;
            }
        }
        for (int i = 0; i < 2 && !s_event_pending; i++) {
            if (s_timers[i].at >= 0 && s_timers[i].at <= s_now_us) {
                host_timer_fn fn = s_timers[i].fn;
                s_timers[i].at = -1;
                fn();
            }
        }
    }
}

int host_task_summary(char *buf, int n)
{
    static const char *names[] = {"ready", "blocked", "waiting for the host", "ended"};
    int used = 0;
    for (int i = 0; i < s_ntasks && used < n - 1; i++) {
        struct host_task *t = &s_tasks[i];
        used += snprintf(buf + used, n - used, "%s%s: %s%s", i ? "; " : "", t->name, names[t->state],
                         t->state == T_BLOCKED && t->wake_us == INT64_MAX ? " with no timeout" : "");
    }
    return used;
}

/* ---- FreeRTOS API --------------------------------------------------------- */
static int64_t ticks_to_timeout_us(TickType_t ticks)
{
    if (ticks == portMAX_DELAY) {
        return -1;
    }
    return (s_now_us / TICK_US + ticks) * (int64_t)TICK_US - s_now_us;   /* wakes on a tick boundary */
}

BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack, void *arg, UBaseType_t prio, TaskHandle_t *out)
{
    (void)stack;
    if (s_ntasks >= MAX_TASKS) {
        return pdFALSE;
    }
    struct host_task *t = &s_tasks[s_ntasks];
    memset(t, 0, sizeof(*t));
    snprintf(t->name, sizeof(t->name), "%s", name);
    t->prio = (int)prio;
    t->fn = fn;
    t->arg = arg;
    t->state = T_READY;
    t->order = ++s_order;
    pthread_cond_init(&t->cv, NULL);
    s_ntasks++;                               /* before the thread starts: it reads s_cur under the lock we hold */
    if (pthread_create(&t->th, NULL, task_thread, t) != 0) {
        s_ntasks--;
        return pdFALSE;
    }
    pthread_detach(t->th);
    if (out) {
        *out = t;
    }
    preempt_check();
    return pdPASS;
}

TickType_t xTaskGetTickCount(void) { return (TickType_t)(s_now_us / TICK_US); }

void vTaskDelay(TickType_t ticks)
{
    if (ticks) {
        host_block(s_cur, ticks_to_timeout_us(ticks));
    }
}

uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks)
{
    struct host_task *self = s_cur;
    if (!self->notify && ticks) {
        host_block(&self->notify, ticks_to_timeout_us(ticks));
    }
    uint32_t v = self->notify;
    self->notify = clear ? 0 : (v ? v - 1 : 0);
    return v;
}

BaseType_t xTaskNotifyGive(TaskHandle_t t)
{
    t->notify++;
    host_wake(&t->notify);
    preempt_check();
    return pdPASS;
}

struct host_sem { int count, max, is_mutex; struct host_task *owner; };

static SemaphoreHandle_t sem_new(int count, int is_mutex)
{
    struct host_sem *s = calloc(1, sizeof(*s));
    s->count = count;
    s->max = 1;
    s->is_mutex = is_mutex;
    return s;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void)  { return sem_new(1, 1); }
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return sem_new(0, 0); }

BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks)
{
    if (s->count > 0) {
        s->count--;
        s->owner = s_cur;
        return pdTRUE;
    }
    if (s->is_mutex && s->owner == s_cur) {
        host_violation("task '%s' takes a mutex it already holds (deadlock)", s_cur ? s_cur->name : "?");
    }
    if (!ticks || !host_block(s, ticks_to_timeout_us(ticks))) {
        return pdFALSE;
    }
    s->owner = s_cur;                           /* handed over directly by the giver */
    return pdTRUE;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t s)
{
    if (s->is_mutex && s->owner != s_cur) {
        host_violation("task '%s' gives a mutex it does not hold", s_cur ? s_cur->name : "?");
    }
    s->owner = NULL;
    if (host_wake(s)) {
        preempt_check();
        return pdTRUE;
    }
    if (s->count >= s->max) {
        return pdFALSE;
    }
    s->count++;
    return pdTRUE;
}

int host_sem_held(SemaphoreHandle_t s) { return s->count == 0; }

struct host_queue { int len, item, count, head; uint8_t *data; char writers; };

QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size)
{
    struct host_queue *q = calloc(1, sizeof(*q));
    q->len = (int)length;
    q->item = (int)item_size;
    q->data = calloc(length, item_size);
    return q;
}

BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t ticks)
{
    while (q->count == q->len) {
        if (!ticks || !host_block(&q->writers, ticks_to_timeout_us(ticks))) {
            return pdFALSE;
        }
    }
    memcpy(q->data + (size_t)((q->head + q->count) % q->len) * q->item, item, q->item);
    q->count++;
    if (host_wake(q)) {
        preempt_check();
    }
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t ticks)
{
    while (q->count == 0) {
        if (!ticks || !host_block(q, ticks_to_timeout_us(ticks))) {
            return pdFALSE;
        }
    }
    memcpy(item, q->data + (size_t)q->head * q->item, q->item);
    q->head = (q->head + 1) % q->len;
    q->count--;
    host_wake(&q->writers);
    return pdTRUE;
}
