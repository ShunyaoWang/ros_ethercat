#ifndef ANYDRIVE_TEST_H
#define ANYDRIVE_TEST_H

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "pthread.h"
//#include "ethercat.h"
//#include "ethercat.h"
#include "soem/soem/ethercat.h"

void anydriveTest(char *ifname);

OSAL_THREAD_FUNC ecatcheck( void *ptr );

#endif // ANYDRIVE_TEST_H
