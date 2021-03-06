/*
  AsyncGSM.h
*/
#ifndef AsyncGSM_h
#define AsyncGSM_h

#ifndef __AVR__
#include <sys/types.h> // for __time_t_defined, but avr libc lacks sys/types.h
#endif


#if !defined(__time_t_defined) // avoid conflict with newlib or other posix libc
typedef unsigned long time_t;
#endif

#define prog_char_strcmp(a, b)                                  strcmp_P((a), (b))
#define prog_char_strstr(a, b)                                  strstr_P((a), (b))

#define GSM_BUFFER_SIZE 128

#define POWER_STATE_OFF 0
#define POWER_STATE_ON 1
#define POWER_STATE_STARTING 2
#define POWER_STATE_STOPPING 3


/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24UL)
#define DAYS_PER_WEEK (7UL)
#define SECS_PER_WEEK (SECS_PER_DAY * DAYS_PER_WEEK)
#define SECS_PER_YEAR (SECS_PER_WEEK * 52UL)
#define SECS_YR_2000  (946684800UL) // the time at the start of y2k

#define GSM_DEFAULT_TIMEOUT_MS 500
#define MAX_INPUT 128

#define STATE_IDLE 0
#define STATE_WAITING_REPLY 1
#define STATE_ERROR 2
#define STATE_UCR 3

#define CONNECTION_TYPE_TCP 0
#define CONNECTION_TYPE_UDP 1

#define GPRS_STATE_UNKNOWN 0
#define GPRS_STATE_IP_INITIAL 1
#define GPRS_STATE_IP_START 2
#define GPRS_STATE_IP_CONFIG 3
#define GPRS_STATE_IP_GPRSACT 4
#define GPRS_STATE_IP_STATUS 5
#define GPRS_STATE_PDP_DEACT 6
#define GPRS_STATE_CONNECT_OK 7
#define GPRS_STATE_CONNECTION_CLOSED 8

#define COMMAND_NONE 0
#define COMMAND_ATE 1
#define COMMAND_WRITE_CIPMUX 2
#define COMMAND_TEST_CIPMUX 7
#define COMMAND_SET_CSTT 3
#define COMMAND_SET_CIICR 4
#define COMMAND_WRITE_CNMI 5
#define COMMAND_TEST_CNMI 6
#define COMMAND_CIPSTATUS 8
#define COMMAND_CIPSHUT 9
#define COMMAND_CBC 10
#define COMMAND_CSQ 11
#define COMMAND_SET_CLTS 12
#define COMMAND_TEST_CCLK 13
#define COMMAND_TEST_CREG 14
#define COMMAND_CIFSR 15
#define COMMAND_WRITE_CIPSTART 16
#define COMMAND_WRITE_CIPSEND 17
#define COMMAND_TEST_CMGF 18
#define COMMAND_WRITE_CMGF 19
#define COMMAND_UCR_CMT 20
#define COMMAND_UCR_CMT_DATA 21
#define COMMAND_TEST_CSCS 22
#define COMMAND_WRITE_CSCS 23
#define COMMAND_WRITE_CMGS 24
#define COMMAND_WRITE_CLIP 25
#define COMMAND_ATA 26
#define COMMAND_WRITE_CIPCLOSE 27
#define COMMAND_UCR_RECEIVE 28
#define COMMAND_ENABLE_POWERSAVE 29
#define COMMAND_DISABLE_POWERSAVE 30


#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

#define prog_char_strcmp(a, b) strcmp_P((a), (b))

#include "Arduino.h"

#define prog_char  char PROGMEM

typedef const __FlashStringHelper *GSMFlashStringPtr;

typedef struct  {
  uint8_t second;
  uint8_t minute;
  uint8_t hour;
  uint8_t day;
  uint8_t month;
  uint8_t year;   // offset from 1970;
} tmelements_t;


typedef struct {
  char buffer[GSM_BUFFER_SIZE];
  size_t head;
  size_t tail;
} CircularBuffer;

typedef struct {
  uint8_t connectionState;
  char address[16];
  uint16_t port;
  CircularBuffer outboundCircular;
  CircularBuffer inboundCircular;
  uint8_t connect : 1;
  uint8_t type : 1;
  uint8_t outboundBytes;
} ConnectionState;

typedef struct {
  char message[161];
  char msisdn[14];
  time_t receive_time;
  uint8_t available;
} ShortMessage;

class AsyncGSM
{
 public:
  AsyncGSM(uint8_t rst, uint8_t pstat, uint8_t key);
  uint8_t initialize(Stream &serial);
  void resetModemState();
  void setDebugStream(Stream &debugStream);
  void process();
  void queueAtCommand(GSMFlashStringPtr command, uint32_t timeout);
  void queueAtCommand(char * command, uint32_t timeout);
  uint8_t isModemIdle();
  uint8_t isModemError();
  uint8_t isModemRegistered();
  void enableGprs();
  void disableGprs();
  void enablePowerSave();
  void disablePowerSave();
  uint8_t isGprsEnabled();
  uint8_t isGprsDisabled();
  void connect(char * ipaddress, int port, int connection, int type);
  void disconnect(int connection);
  uint8_t isConnected(int connection);
  uint8_t writeData(char * data, int len, int connection);
  uint8_t messageAvailable();
  uint8_t dataAvailable(int connection);
  uint8_t outboundBufferSize(int connection);
  ShortMessage readMessage();
  void sendMessage(ShortMessage message);
  time_t getCurrentTime();
  int8_t incomingCall();
  char * getCallerIdentification();
  void answerIncomingCall();
  void hangupCall();
  void setPower(uint8_t power);
 protected:
  uint8_t handlePowerState();
  void processIncomingModemByte (const byte inByte);
  void process_modem_data (char * data);
  GSMFlashStringPtr ok_reply;
  ConnectionState connectionState[1];
 private:  
  uint8_t writeBuffer(CircularBuffer * buffer, char data);
  uint8_t readBuffer(CircularBuffer * buffer, char * data);
  uint8_t bufferSize(CircularBuffer * buffer);
  uint8_t parseConnectionNumber(char * data);
  Stream *mySerial;
  Stream *debugStream;
  time_t parseTime(char * timeString);
  char input_modem_line [MAX_INPUT];
  uint8_t input_modem_pos = 0;
  int8_t modem_state;
  int8_t command_state;
  int8_t autobauding;
  int8_t cipmux;
  int8_t cipqsend;
  int8_t gprs_state;
  int8_t gprs_active;
  uint8_t enable_gprs;
  uint8_t enable_powersave;
  uint8_t powersave;
  int8_t ip_address;
  int8_t echo;
  int8_t cnmi;
  int8_t clts;
  int8_t cmgf;
  int8_t cscs;
  int8_t creg;
  int8_t clip;
  int8_t incomingcall;
  int8_t callinprogress;
  int8_t answerincomingcall;
  int8_t currentconnection;
  uint32_t last_time_update;
  uint32_t last_csq_update;
  uint32_t last_battery_update;
  uint32_t last_creg;
  uint32_t last_udp_send;
  uint32_t last_command;
  ShortMessage messageBuffer;
  ShortMessage outboundMessage;
  time_t last_network_time;
  uint32_t last_network_time_update;
  uint32_t command_timeout;
  char callerId[14];

  // power status
  uint8_t power_state;
  uint8_t power;
  uint32_t power_state_changed;

  // fona pins
  uint8_t reset;
  uint8_t pstat;
  uint8_t key;
};

#endif
