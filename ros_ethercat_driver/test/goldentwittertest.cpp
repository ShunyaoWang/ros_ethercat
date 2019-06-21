#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include "soem/ethercat.h"

#define EC_TIMEOUTMON 500
#define INITIAL_POS 0

char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

/** Main slave data array.
 *  Each slave found on the network gets its own record.
 *  ec_slave[0] is reserved for the master. Structure gets filled
 *  in by the configuration function ec_config().
 */
ec_slavet               ec_slave[EC_MAXSLAVE];
/** number of slaves found on the network */
int                     ec_slavecount;
/** slave group structure */
ec_groupt               ec_group[EC_MAXGROUP];

/** cache for EEPROM read functions */
static uint8            ec_esibuf[EC_MAXEEPBUF];
/** bitmap for filled cache buffer bytes */
static uint32           ec_esimap[EC_MAXEEPBITMAP];
/** current slave for EEPROM cache buffer */
static ec_eringt        ec_elist;
static ec_idxstackT     ec_idxstack;

/** SyncManager Communication Type struct to store data of one slave */
static ec_SMcommtypet   ec_SMcommtype[EC_MAX_MAPT];
/** PDO assign struct to store data of one slave */
static ec_PDOassignt    ec_PDOassign[EC_MAX_MAPT];
/** PDO description struct to store data of one slave */
static ec_PDOdesct      ec_PDOdesc[EC_MAX_MAPT];

/** buffer for EEPROM SM data */
static ec_eepromSMt     ec_SM;
/** buffer for EEPROM FMMU data */
static ec_eepromFMMUt   ec_FMMU;
/** Global variable TRUE if error available in error stack */
boolean                 EcatError = FALSE;

int64                   ec_DCtime;

ecx_portt               ecx_port;
ecx_redportt            ecx_redport;

ecx_contextt  ecx_context = {
    &ecx_port,          // .port          =
    &ec_slave[0],       // .slavelist     =
    &ec_slavecount,     // .slavecount    =
    EC_MAXSLAVE,        // .maxslave      =
    &ec_group[0],       // .grouplist     =
    EC_MAXGROUP,        // .maxgroup      =
    &ec_esibuf[0],      // .esibuf        =
    &ec_esimap[0],      // .esimap        =
    0,                  // .esislave      =
    &ec_elist,          // .elist         =
    &ec_idxstack,       // .idxstack      =
    &EcatError,         // .ecaterror     =
    0,                  // .DCtO          =
    0,                  // .DCl           =
    &ec_DCtime,         // .DCtime        =
    &ec_SMcommtype[0],  // .SMcommtype    =
    &ec_PDOassign[0],   // .PDOassign     =
    &ec_PDOdesc[0],     // .PDOdesc       =
    &ec_SM,             // .eepSM         =
    &ec_FMMU,           // .eepFMMU       =
    NULL,               // .FOEhook()
//    NULL                // .EOEhook()
};

struct TorqueOut {
    uint16 torque;
    uint16 status;
};
struct TorqueIn {
    int32 position;
    int16 torque;
    uint16 status;
    int8 profile;
};

/**
 * helper macros
 */
#define READ(slaveId, idx, sub, buf, comment)    \
    {   \
        buf=0;  \
        int __s = sizeof(buf);    \
        int __ret = ecx_SDOread(&ecx_context, slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);   \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment);    \
     }

#define WRITE(slaveId, idx, sub, buf, value, comment) \
    {   \
        int __s = sizeof(buf);  \
        buf = value;    \
        int __ret = ecx_SDOwrite(&ecx_context, slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);  \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment);    \
    }

#define CHECKERROR(slaveId)   \
{   \
    ecx_readstate(&ecx_context);\
    printf("EC> \"%s\" %x - %x [%s] \n", (char*)ecx_elist2string(&ecx_context), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char*)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode));    \
}


void simpletest(char *ifname)
{
    int i, j, oloop, iloop, wkc_count, chk;
    needlf = FALSE;
    inOP = FALSE;

    uint32 buf32;
    uint16 buf16;
    uint8 buf8;

    struct TorqueIn *val;
    struct TorqueOut *target;

    struct TorqueIn *val2;
    struct TorqueOut *target2;

    printf("Starting simple test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ecx_init(&ecx_context,ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */

        /** network discovery */
        if ( ecx_config_init(&ecx_context, FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            for (int i=1; i<=ec_slavecount; i++) {
                printf("Slave %d has CA? %s\n", i, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true":"false" );

                /** CompleteAccess disabled for Elmo driver */
                ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            ecx_statecheck(&ecx_context, 0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

            /** set PDO mapping */
            /** opMode: 8  => Position profile */
            for (int i=1; i<=ec_slavecount; i++) {
                WRITE(i, 0x6060, 0, buf8, 10, "OpMode");
                READ(i, 0x6061, 0, buf8, "OpMode display");


                READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                READ(i, 0x1c13, 0, buf32, "txPDO:0");

                READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }

            int32 ob2;int os;
            for (int i=1; i<=ec_slavecount; i++) {
                os=sizeof(ob2); ob2 = 0x16020001;
                ecx_SDOwrite(&ecx_context, i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
                os=sizeof(ob2); ob2 = 0x1a020001;
                ecx_SDOwrite(&ecx_context, i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);

                READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                READ(i, 0x1c13, 0, buf32, "txPDO:0");

                READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }

            /** if CA disable => automapping works */
            ecx_config_map_group(&ecx_context, &IOmap, 0);

            // show slave info
            for (int i=1; i<=ec_slavecount; i++) {
                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
                ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
            }

            /** disable heartbeat alarm */
            for (int i=1; i<=ec_slavecount; i++) {
                READ(i, 0x10F1, 2, buf32, "Heartbeat?");
                WRITE(i, 0x10F1, 2, buf32, 1, "Heartbeat");

                WRITE(i, 0x60c2, 1, buf8, 2, "Time period");
                WRITE(i, 0x2f75, 0, buf16, 2, "Interpolation timeout");
            }

            printf("Slaves mapped, state to SAFE_OP.\n");

            int timestep = 700;

            /* wait for all slaves to reach SAFE_OP state */
            ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            /** old SOEM code, inactive */
            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 20) oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            if (iloop > 20) iloop = 8;

            printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ecx_send_processdata(&ecx_context);
            ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

            for (int i=1; i<=ec_slavecount; i++) {
                READ(i, 0x6083, 0, buf32, "Profile acceleration");
                READ(i, 0x6084, 0, buf32, "Profile deceleration");
                READ(i, 0x6085, 0, buf32, "Quick stop deceleration");
            }

            /* request OP state for all slaves */
            ecx_writestate(&ecx_context, 0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ecx_send_processdata(&ecx_context);
                ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
                ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("Operational state reached for all slaves.\n");
                wkc_count = 0;
                inOP = TRUE;

                /**
                 * Drive state machine transistions
                 *   0 -> 6 -> 7 -> 15
                 */
                for (int i=1; i<=ec_slavecount; i++) {
                    READ(i, 0x6041, 0, buf16, "*status word*");
                    if(buf16 == 0x218)
                    {
                        WRITE(i, 0x6040, 0, buf16, 128, "*control word*"); usleep(100000);
                        READ(i, 0x6041, 0, buf16, "*status word*");
                    }


                    WRITE(i, 0x6040, 0, buf16, 0, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 6, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 7, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 15, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    CHECKERROR(i);
                    READ(i, 0x1a0b, 0, buf8, "OpMode Display");

                    READ(i, 0x1001, 0, buf8, "Error");
                }
                int reachedInitial = 0;
                int reachedInitial2 = 0;

                /* cyclic loop for two slaves*/
                target = (struct TorqueOut *)(ec_slave[1].outputs);
                val = (struct TorqueIn *)(ec_slave[1].inputs);

//                target2 = (struct TorqueOut *)(ec_slave[2].outputs);
//                val2 = (struct TorqueIn *)(ec_slave[2].inputs);

                for(i = 1; i <= 4000; i++)
                {
                    /** PDO I/O refresh */
                    ecx_send_processdata(&ecx_context);
                    wkc = ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

                    if(wkc >= expectedWKC) {
                        printf("Processdata cycle %4d, WKC %d,", i, wkc);
                        printf("  pos: 0x%x, tor: 0x%x, stat: 0x%x, mode: 0x%x", val->position, val->torque, val->status, val->profile);

                        /** if in fault or in the way to normal status, we update the state machine */
                        // slave 1
                        switch(target->status){
                        case 0:
                            target->status = 6;
                            break;
                        case 6:
                            target->status = 7;
                            break;
                        case 7:
                            target->status = 15;
                            break;
                        case 128:
                            target->status = 0;
                            break;
                        default:
                            if(val->status >> 3 & 0x01) {
                                READ(1, 0x1001, 0, buf8, "Error");
                                target->status = 128;
                            }
                        }

                        // Slave 2
//                        switch(target2->status){
//                        case 0:
//                            target2->status = 6;
//                            break;
//                        case 6:
//                            target2->status = 7;
//                            break;
//                        case 7:
//                            target2->status = 15;
//                            break;
//                        case 128:
//                            target2->status = 0;
//                            break;
//                        default:
//                            if(val2->status >> 3 & 0x01) {
//                                READ(2, 0x1001, 0, buf8, "Error");
//                                target2->status = 128;
//                            }
//                        }


                        /** we wait to be in ready-to-run mode and with initial value reached */
                        if(reachedInitial == 0 /*&& val->position == INITIAL_POS */&& (val->status & 0x0fff) == 0x0237){
                            reachedInitial = 1;
                        }

                        if(reachedInitial2 == 0 /*&& val->position == INITIAL_POS */&& (val2->status & 0x0fff) == 0x0237){
                            reachedInitial2 = 1;
                        }

                        if((val->status & 0x0fff) == 0x0237 && reachedInitial){
                            target->torque = (int16) (sin(i/100.)*(1000));
                        }

//                        if((val2->status & 0x0fff) == 0x0237 && reachedInitial2){
//                            target2->torque = (int16) (sin(i/100.)*(1000));
//                        }

                        printf("  Target: 0x%x, control: 0x%x", target->torque, target->status);

                        printf("\r");
                        needlf = TRUE;
                    }
                    usleep(timestep);
                }
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ecx_readstate(&ecx_context);
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            printf("\nRequest init state for all slaves\n");
            for (int i=1; i<=ec_slavecount; i++) {
                WRITE(i, 0x10F1, 2, buf32, 0, "Heartbeat");
            }

            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ecx_writestate(&ecx_context, 0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ecx_close(&ecx_context);
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

void *ecatcheck( void *ptr )
{
    int slave;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ecx_readstate(&ecx_context);
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ecx_writestate(&ecx_context, slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                    printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                    ec_slave[slave].state = EC_STATE_OPERATIONAL;
                    ecx_writestate(&ecx_context, slave);
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ecx_reconfig_slave(&ecx_context, slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ecx_statecheck(&ecx_context,slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ecx_recover_slave(&ecx_context,slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf(".");
        }
        usleep(250);
    }
}

int main(int argc, char *argv[])
{
    int iret1;
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1)
    {
        /* create thread to handle slave error handling in OP */
        iret1 = pthread_create( &thread1, NULL, &ecatcheck, (void (*)) &ctime); // (void) &ctime
        /* start cyclic part */
        simpletest(argv[1]);
    }
    else
    {
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }

    printf("End program\n");
    return (0);
}
