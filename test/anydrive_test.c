/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "pthread.h"
#include "ros_ethercat_driver/anydrive_test.h"
//#include "ethercat.h"
//#include "ethercat.h"
#include "soem/ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

enum ANYDriveFSMState{
  COLD_START  = 1,
  WARM_START  = 2,
  CONFIGURE   = 3,
  CALIBRATE   = 4,
  STANDBY     = 5,
  MOTOR_OP    = 6,
  CONTROL_OP  = 7,
  ERROR       = 8,
  FATAL       = 9,
  MOTOR_PRE_OP= 10,
};

enum ANYDriveModeOfOperation{
  FREEZE                              = 1,
  DISABLE                             = 2,
  CURRENT                             = 3,
  MOTOR_POSITION                      = 4,
  MOTOR_VELOCITY                      = 5,
  GEAR_POSITION                       = 6,
  GEAR_VELOCITY                       = 7,
  JOINT_POSITION                      = 8,
  JOINT_VELOCITY                      = 9,
  JOINT_TORQUE                        = 10,
  JOINT_POSITION_VELOCITY             = 11,
  JOINT_POSITION_VELOCITY_TORQUE      = 12,
  JOINT_POSITION_VELOCITY_TORQUE_PID  = 13,

};

enum ANYDriveControlWord{
  WARM_RESET                = 1,
  CLEAR_ERRORS_TO_MOTOR_OP  = 2,
  STANDBY_TO_CONFIGURE      = 3,
  CONFIGURE_TO_STANDBY      = 4,
  CALIBRATE_TO_CONFIGURE    = 5,
  CONFIGURE_TO_CALIBRATE    = 6,
  MOTOR_OP_TO_STANDBY       = 7,
  STANDBY_TO_MOTOR_PRE_OP   = 8,
  CONTROL_OP_TO_MOTOR_OP    = 9,
  MOTOR_OP_TO_CONTROL_OP    = 10,
  CONTROL_OP_TO_STANDBY     = 11,
  CLEAR_ERRORS_TO_STANDBY   = 12,
};

//! WSHY: EtherCAT datagrams of ANYDrive of Type A, refer to the ANYDrive document Page 36
struct ANYDriveRPDO{
  uint16_t control_word;         // uint16    0x6040
  uint16_t mode_of_operation;   // float C    0x6060
  int32_t desired_motor_current;  // float A  0x2020
  uint32_t desired_velocity; // float krpm    0x60FF
  int32_t desired_joint_torque; // float NM   0x6071
  int64_t desired_position; // double rad     0x607A
  int32_t control_parameter_a;  // float      0x2024
  int32_t control_parameter_b; // float       0x2025
  int32_t control_parameter_c; //float        0x2026
  int32_t control_parameter_d;   // float     0x2027
};


struct ANYDriveTPDO{
  uint32_t state;         // uint32           0x6061 4bit FSM stste, 4bit MOP, 24bit,warnnings, errors, fatals
  uint16_t temperature;   // float C          0x2002
  uint16_t voltage;       // float V          0x2003
  int64_t motor_position; // double rad       0x6064
  int64_t gear_position;  // double rad       0x2005
  int64_t joint_position; // double rad       0x2006
  int32_t motor_current;  // float A          0x2007
  int32_t motor_velocity; // float krpm       0x606C
  int32_t gear_velocity;  // float rpm        0x2009
  int32_t joint_velocity; // float rpm        0x200A
  int32_t joint_acceleration; //float rpm/s   0x200B
  int32_t joint_torque;   // float Nm         0x6077
};

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
    NULL                // .EOEhook()
};

typedef union {
  int32_t int32data;
  uint32_t uint32data;
  float floatdata;
}Bytes4Exchage;

float convert_uint32_to_float(uint32_t *src) {
  float dest;
  memcpy(&dest, src, sizeof(float));
  return dest;
}

float convert_int32_to_float(int32_t *src) {
  float dest;
  memcpy(&dest, src, sizeof(float));
  return dest;
}
int32_t convert_float_to_int32(float *src) {
  int32_t dest;
  memcpy(&dest, src, sizeof(int32_t));
  return dest;
}
double convert_int64_to_double(int64_t *src) {
  double dest;
  memcpy(&dest, src, sizeof(double));
  return dest;
}

void anydriveTest(char *ifname)
{
    int i, j, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

   printf("Starting ANYDrive test\n");

   //! WSHY: read and write data struct
   struct ANYDriveTPDO *feedback;
   struct ANYDriveRPDO *command;

   /* initialise SOEM, bind socket to ifname */
   if (ecx_init(&ecx_context,ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */


       if ( ecx_config_init(&ecx_context, FALSE) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);

         for (int i=1; i<=ec_slavecount; i++) {
             printf("Slave %d has CA? %s\n", i, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true":"false" );
         }
         //! WSHY: the ANYDrive is not support LRW(Logical READ&WRITE) but, it can't read
         //! by the upload slave info, so we manually assigned the value to Block LRW, before
         //! ec_config_map set it to ec_context.
         ec_slave[1].blockLRW = 1;
         //! WSHY: map the all input and output of all slaves to a static array
         ecx_config_map_group(&ecx_context, &IOmap, 0);

         ecx_configdc(&ecx_context);

         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ecx_statecheck(&ecx_context,0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

         // show slave info
         for (int i=1; i<=ec_slavecount; i++) {
             printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
             i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
             ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
         }

         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         if (oloop > 8) oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
         if (iloop > 8) iloop = 8;

         printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         ec_slave[1].state = EC_STATE_OPERATIONAL;

         //! WSHY: assigned the inputs and outputs pointer to out read and write data struct
         feedback = (struct ANYDriveTPDO *)(ec_slave[1].inputs);
         command = (struct ANYDriveRPDO *)(ec_slave[1].outputs);

         command->mode_of_operation = 1;
         /* send one valid process data to make outputs in slaves happy*/
         ecx_send_processdata(&ecx_context);
         ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ecx_writestate(&ecx_context, 0);
//         ec_readstate();
         chk = 40;
         /* wait for all slaves to reach OP state */
         do
         {
             ecx_send_processdata(&ecx_context);
             ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
             ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

         // show slave info
         for (int i=1; i<=ec_slavecount; i++) {
             printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
             i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
             ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
         }

//         uint32_t *src = &feedback->motor_current;
         printf(" ANYDrive motor current is %d\n", feedback->state);//convert_uint32_to_float(feedback->motor_current));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;
                /* cyclic loop */
            for(i = 1; i <= 2000; i++)
            {
               ecx_send_processdata(&ecx_context);
               wkc = ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

                    if(wkc >= expectedWKC)
                    {
//                        printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

                        for(j = 0 ; j < oloop; j++)
                        {
//                            printf(" %2.2x", *(ec_slave[0].outputs + j));
                        }

//                        printf(" I:");
                        for(j = 0 ; j < iloop; j++)
                        {
//                            printf(" %2.2x", *(ec_slave[0].inputs + j));
                        }
//                        printf(" T:%"PRId64"\r",ec_DCtime);
                        needlf = TRUE;
                    }
                    osal_usleep(5000);
//                    printf(" %s motor voltage is %fV\n", ec_slave[1].name, 0.01*feedback->voltage);//convert_uint32_to_float(feedback->motor_current));
//                    printf(" %s motor temperature is %f°C\n", ec_slave[1].name, 0.01*feedback->temperature -55.0);//convert_uint32_to_float(feedback->motor_current));
//                    printf(" %s motor current is %f°A\n", ec_slave[1].name, convert_int32_to_float(&feedback->motor_current));//convert_uint32_to_float(feedback->motor_current));
//                    printf(" %s motor position is %f°rad\n", ec_slave[1].name, convert_int64_to_double(&feedback->motor_position));//convert_uint32_to_float(feedback->motor_current));
                      printf(" %s joint velocity is %frpm\n", ec_slave[1].name, convert_int32_to_float(&feedback->joint_velocity));//convert_uint32_to_float(feedback->motor_current));

                    if(i == 10)
                      {
                        command->control_word = CONFIGURE_TO_STANDBY;
                        printf("CONFIGURE_TO_STANDBY");
                      }
                    if((feedback->state<<28)>>28 == STANDBY){
                        command->control_word = STANDBY_TO_MOTOR_PRE_OP;
                        printf("STANDBY");
                      }
                    if((feedback->state<<28)>>28 == MOTOR_OP){
                        command->control_word = MOTOR_OP_TO_CONTROL_OP;
                        printf("MOTOR_PRE_OP");
                      }
                    if((feedback->state<<28)>>28 == CONTROL_OP){
                        command->mode_of_operation = JOINT_VELOCITY;
                        printf("CONTROL_OP");
                      }
                    if((feedback->state<<24)>>28 == JOINT_VELOCITY){
                        Bytes4Exchage vel;
                        vel.floatdata = 10.0;
                        command->desired_velocity = vel.uint32data;//convert_float_to_int32(&vel);
                      }
                    if(i == 1900){
                        command->desired_velocity = 0.0;
                      }
                    if(i == 1950){
                        command->mode_of_operation = FREEZE;
                      }
                    if(i == 1960){
                        command->control_word = CONTROL_OP_TO_STANDBY;
                      }
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

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

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
                  else if(ec_slave[slave].state > 0x00)//EC_STATE_NONE)
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
                     ecx_statecheck(&ecx_context, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == 0x00)//EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == 0x00)//EC_STATE_NONE)
                  {
                     if (ecx_recover_slave(&ecx_context, slave, EC_TIMEOUTMON))
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
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
//      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
//      osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
      /* start cyclic part */
      anydriveTest(argv[1]);
   }
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }

   printf("End program\n");
   return (0);
}
