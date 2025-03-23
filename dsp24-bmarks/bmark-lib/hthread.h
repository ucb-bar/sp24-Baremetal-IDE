#ifndef __HTHREAD_H
#define __HTHREAD_H

#include "clint.h"
#include "riscv.h"
#include "chip_config.h"

#define N_HARTS 4

void hthread_issue(uint32_t hartid, void *(* start_routine)(void *), void *arg);

void hthread_join(uint32_t hartid);

#endif /* __HTHREAD_H */
