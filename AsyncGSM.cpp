/*
  AsyncGSM.cpp
*/

#include "Arduino.h"
#include "AsyncGSM.h"

#define GSM_DEBUG_PRINT(...) debugStream->print(__VA_ARGS__)
#define GSM_DEBUG_PRINTLN(...) debugStream->println(__VA_ARGS__)

AsyncGSM::AsyncGSM(uint8_t reset, uint8_t pstat, uint8_t key)
{
  ok_reply = F("OK");
  pinMode(reset, OUTPUT);
  pinMode(pstat, INPUT);
  pinMode(key, OUTPUT);
  digitalWrite(key, HIGH);
  this->power_state = digitalRead(pstat);
  this->reset = reset;
  this->pstat = pstat;
  this->key = key;
  modem_state = STATE_IDLE;
  autobauding = 0;
  echo = 0;
  cnmi = 0;
  command_state = COMMAND_NONE;
  currentconnection = -1;
  command_timeout = 10000;
}

void AsyncGSM::setPower(uint8_t power) {
  this->power = power;
  uint8_t current_power = digitalRead(pstat);
  //GSM_DEBUG_PRINTLN(F("setPower: "));
  //GSM_DEBUG_PRINTLN(power);
  //GSM_DEBUG_PRINTLN(current_power);
}

uint8_t AsyncGSM::handlePowerState() {
  uint8_t current_power = digitalRead(pstat);
  if (!current_power && power && power_state == POWER_STATE_OFF) {
    // start turning on
    GSM_DEBUG_PRINT(F("Turning on. Current power: "));
    GSM_DEBUG_PRINTLN(current_power);
    power_state = POWER_STATE_STARTING;
    digitalWrite(key, LOW);
    power_state_changed = millis();
    return 0;
  } else if (current_power && !power && power_state == POWER_STATE_ON) {
    // start turning off
    GSM_DEBUG_PRINT(F("Turning off. Current power: "));
    GSM_DEBUG_PRINTLN(current_power);
    power_state = POWER_STATE_STOPPING;
    digitalWrite(key, LOW);
    power_state_changed = millis();
    return 0;
  } else if ((power_state == POWER_STATE_STOPPING || power_state == POWER_STATE_STARTING) && millis() > (power_state_changed + 3000)) {
    // stop "pressing" key and reset power_state
    GSM_DEBUG_PRINTLN(F("Release key button"));
    digitalWrite(key, HIGH);
    power_state = digitalRead(pstat);
    GSM_DEBUG_PRINT(F("current_power: "));
    GSM_DEBUG_PRINTLN(power_state);
    resetModemState();
    return power_state;
  }

  return current_power;
}

void AsyncGSM::resetModemState() {
  echo = 0;
  cscs = 0;
  clip = 0;
  cmgf = 0;
  cnmi = 0;
  cipmux = 0;
  creg = 0;
  modem_state = STATE_IDLE;
  autobauding = 0;
  command_state = COMMAND_NONE;
  gprs_state = GPRS_STATE_UNKNOWN;
  currentconnection = -1;
  command_timeout = 10000;
}

uint8_t AsyncGSM::initialize(Stream &serial)
{
  mySerial = &serial;
}

void AsyncGSM::setDebugStream(Stream &stream)
{
  debugStream = &stream;
}

uint8_t AsyncGSM::writeBuffer(CircularBuffer * buffer, char data) {
  int next = buffer->head + 1;
  if (next >= NELEMS(buffer->buffer))
    next = 0;

  // Cicular buffer is full
  if (next == buffer->tail)
    return -1;  // quit with an error

  buffer->buffer[buffer->head] = data;
  buffer->head = next;
  return 0;
}

uint8_t AsyncGSM::readBuffer(CircularBuffer * buffer, char * data) {
  // if the head isn't ahead of the tail, we don't have any characters
  if (buffer->head == buffer->tail)
    return -1;  // quit with an error

  *data = buffer->buffer[buffer->tail];
  buffer->buffer[buffer->tail] = 0;  // clear the data (optional)

  int next = buffer->tail + 1;
  if(next >= NELEMS(buffer->buffer))
    next = 0;

  buffer->tail = next;

  return 0;
}

uint8_t AsyncGSM::bufferSize(CircularBuffer * buffer) {
  if (buffer->head > buffer->tail) {
    return buffer->head - buffer->tail;
  }
  if (buffer->head < buffer->tail) {
    return GSM_BUFFER_SIZE - buffer->tail + buffer->head;
  }
  return 0;
}



uint8_t AsyncGSM::dataAvailable(int connection) {
  return bufferSize(&connectionState[connection].inboundCircular);
}

uint8_t AsyncGSM::outboundBufferSize(int connection) {
  return bufferSize(&connectionState[connection].outboundCircular);
}

uint8_t AsyncGSM::messageAvailable() {
  return messageBuffer.available;
}

ShortMessage AsyncGSM::readMessage() {
  ShortMessage messageCopy = messageBuffer;
  messageBuffer.available = 0;
  return messageCopy;
}

void AsyncGSM::sendMessage(ShortMessage message) {
  outboundMessage = message;
}

int8_t AsyncGSM::incomingCall() {
  return incomingcall;
}

char * AsyncGSM::getCallerIdentification() {
  return callerId;
}

void AsyncGSM::answerIncomingCall() {
  if (incomingcall && !answerincomingcall && !callinprogress) {
    answerincomingcall = 1;
  }
}

void AsyncGSM::hangupCall() {

}

// state machine
void AsyncGSM::process() {

  uint8_t current_power = handlePowerState();
  if (!current_power) {
    return;
  }

  
  if(mySerial->available() > 0) {
    processIncomingModemByte(mySerial->read());
  }

  // check for timeout
  if (modem_state == STATE_WAITING_REPLY && millis() > last_command + command_timeout) {
    GSM_DEBUG_PRINTLN(F("TIMEOUT"));
    modem_state = STATE_IDLE;
  }
  
  if (modem_state == STATE_IDLE && !autobauding) {
    queueAtCommand(F("AT"), 2000);
    return;
  }
  
  if (modem_state == STATE_IDLE && !echo && autobauding) {
    queueAtCommand(F("ATE0"), 5000);
    command_state = COMMAND_ATE;
    return;
  }

  if (modem_state == STATE_IDLE && incomingcall && answerincomingcall) {
    queueAtCommand(F("ATA"), 5000);
    command_state = COMMAND_ATA;
    return;
  }

  if (modem_state == STATE_IDLE && (millis() > last_csq_update + 90000) && autobauding) {
    queueAtCommand(F("AT+CSQ"), 5000);
    last_csq_update = millis();
    command_state = COMMAND_CSQ;
    return;
  }
  
  if (modem_state == STATE_IDLE && (millis() > last_battery_update + 60000) && autobauding) {
    queueAtCommand(F("AT+CBC"), 5000);
    last_battery_update = millis();
    command_state = COMMAND_CBC;
    return;
  }

  if (modem_state == STATE_IDLE && (millis() > last_creg + 60000) && autobauding && creg < 2) {
    queueAtCommand(F("AT+CREG?"), 5000);
    last_creg = millis();
    command_state = COMMAND_TEST_CREG;
    return;
  } else if (creg < 2) {
    return;
  }

  if (modem_state == STATE_IDLE && autobauding && !powersave && enable_powersave) {
    queueAtCommand(F("AT+CSCLK=1"), 5000);
    command_state = COMMAND_ENABLE_POWERSAVE;
    return;
  }

  if (modem_state == STATE_IDLE && autobauding && powersave && !enable_powersave) {
    queueAtCommand(F("AT+CSCLK=0"), 5000);
    command_state = COMMAND_DISABLE_POWERSAVE;
    return;
  }  
  
  if (modem_state == STATE_IDLE && !clts && autobauding) {
    queueAtCommand(F("AT+CLTS=1"), 5000);
    command_state = COMMAND_SET_CLTS;
    return;
  }

  if (modem_state == STATE_IDLE && !clip && autobauding) {
    queueAtCommand(F("AT+CLIP=1"), 5000);
    command_state = COMMAND_WRITE_CLIP;
    return;
  }
  
  if (modem_state == STATE_IDLE && cipmux == 1 && autobauding && creg == 2) {
    queueAtCommand(F("AT+CIPMUX=1"), 5000);
    command_state = COMMAND_WRITE_CIPMUX;
    return;
  }

  if (modem_state == STATE_IDLE && !cipmux && autobauding && creg == 2) {
    queueAtCommand(F("AT+CIPMUX?"), 5000);
    command_state = COMMAND_TEST_CIPMUX;
    return;
  }

  if (modem_state == STATE_IDLE && gprs_state == GPRS_STATE_UNKNOWN && autobauding && creg == 2 && enable_gprs) {
    queueAtCommand(F("AT+CIPSHUT"), 10000);
    command_state = COMMAND_CIPSHUT;
    return;
  }

  if (modem_state == STATE_IDLE && (gprs_state != GPRS_STATE_IP_INITIAL) && autobauding && creg == 2 && !enable_gprs) {
    queueAtCommand(F("AT+CIPSHUT"), 10000);
    command_state = COMMAND_CIPSHUT;
    return;
  }
  
  if (modem_state == STATE_IDLE && gprs_state == GPRS_STATE_IP_INITIAL && autobauding && creg == 2 && enable_gprs) {
    queueAtCommand(F("AT+CSTT=\"internet.saunalahti\",\"\",\"\""), 10000);
    command_state = COMMAND_SET_CSTT;
    return;
  }

  if (modem_state == STATE_IDLE && !cnmi && autobauding) {
    queueAtCommand(F("AT+CNMI?"), 10000);
    command_state = COMMAND_TEST_CNMI;
    return;
  }

  if (modem_state == STATE_IDLE && !cmgf && autobauding) {
    queueAtCommand(F("AT+CMGF?"), 10000);
    command_state = COMMAND_TEST_CMGF;
    return;
  }

  if (modem_state == STATE_IDLE && cmgf == 1 && autobauding) {
    // text mode sms
    queueAtCommand(F("AT+CMGF=1"), 5000);
    command_state = COMMAND_WRITE_CMGF;
    return;
  }

  if (modem_state == STATE_IDLE && !cscs && autobauding) {
    // text mode sms
    queueAtCommand(F("AT+CSCS=\"8859-1\""), 5000);
    command_state = COMMAND_WRITE_CSCS;
    return;
  }
  
  if (modem_state == STATE_IDLE && cnmi == 1 && autobauding) {
    queueAtCommand(F("AT+CNMI=2,2,0,0,0"), 60000);
    command_state = COMMAND_WRITE_CNMI;
    return;
  }


  if (modem_state == STATE_IDLE && (millis() > last_time_update + 120000) && autobauding) {
    queueAtCommand(F("AT+CCLK?"), 5000);
    last_time_update = millis();
    command_state = COMMAND_TEST_CCLK;
    return;
  }




  
  if (modem_state == STATE_IDLE && gprs_state == GPRS_STATE_IP_START && autobauding && creg == 2 && enable_gprs) {
    queueAtCommand(F("AT+CIICR"), 120000);
    command_state = COMMAND_SET_CIICR;
    return;
  }

  if (modem_state == STATE_IDLE && gprs_state == GPRS_STATE_IP_GPRSACT && autobauding && creg == 2 && enable_gprs) {
    queueAtCommand(F("AT+CIFSR"), 120000);
    command_state = COMMAND_CIFSR;
    return;
  }

  for (int i = 0; i < NELEMS(connectionState); i++) {
    if (modem_state == STATE_IDLE && 
	gprs_state == GPRS_STATE_IP_STATUS && 
	connectionState[i].connectionState != GPRS_STATE_CONNECT_OK && 
	creg == 2 &&
	strlen(connectionState[i].address) > 0 &&
	connectionState[i].port != 0 &&
	connectionState[i].connect &&
	enable_gprs) {
      char command[32];
      sprintf(command, "AT+CIPSTART=%u,\"%s\",\"%s\",%u",
	      i,
	      connectionState[i].type == CONNECTION_TYPE_TCP ? "TCP" : "UDP",
	      connectionState[i].address,
	      connectionState[i].port); 
      queueAtCommand(command, 60000);
      command_state = COMMAND_WRITE_CIPSTART;
      currentconnection = i;
      return;
    }
  }


  for (int j = 0; j < NELEMS(connectionState); j++) {
    if (modem_state == STATE_IDLE && 
	gprs_state == GPRS_STATE_IP_STATUS && 
	connectionState[j].connectionState == GPRS_STATE_CONNECT_OK && 
	bufferSize(&connectionState[j].outboundCircular) > 0 && creg == 2 && enable_gprs) {
      char command[16];
      connectionState[j].outboundBytes = bufferSize(&connectionState[j].outboundCircular);
      sprintf(command, "AT+CIPSEND=%u,%u", j, connectionState[j].outboundBytes);
      queueAtCommand(command, 120000);
      command_state = COMMAND_WRITE_CIPSEND;
      currentconnection = j;
      return;
    }
  }

  for (int i = 0; i < NELEMS(connectionState); i++) {
    if (modem_state == STATE_IDLE && 
	gprs_state == GPRS_STATE_IP_STATUS && 
	connectionState[i].connectionState == GPRS_STATE_CONNECT_OK && 
	creg == 2 &&
	!connectionState[i].connect && enable_gprs) {
      char command[32];
      sprintf(command, "AT+CIPCLOSE=%u,0", i);
      queueAtCommand(command, 60000);
      command_state = COMMAND_WRITE_CIPCLOSE;
      currentconnection = i;
      return;
    }
  }

  
  /*
  if (millis() > last_udp_send + 120000) {
    char data[16];
    sprintf(data, "Test%u", millis());
    writeData(data, strlen(data));
    last_udp_send = millis();  
  }
  */

  if (modem_state == STATE_IDLE && strlen(outboundMessage.message) > 0 && autobauding && creg == 2) {
    char command[64];
    sprintf(command, "AT+CMGS=\"%s\"", outboundMessage.msisdn);
    queueAtCommand(command, 10000);
    command_state = COMMAND_WRITE_CMGS;
  }

  
  
}


uint8_t AsyncGSM::isConnected(int connection) {
  return connectionState[connection].connectionState == GPRS_STATE_CONNECT_OK;
}

uint8_t AsyncGSM::isModemIdle() {
  return modem_state == STATE_IDLE;
}

uint8_t AsyncGSM::isModemError() {
  return modem_state == STATE_ERROR;
}

uint8_t AsyncGSM::isModemRegistered() {
  return creg == 2;
}

void AsyncGSM::enableGprs() {
  enable_gprs = 1;
}

void AsyncGSM::disableGprs() {
  enable_gprs = 0;
}

void AsyncGSM::enablePowerSave() {
  enable_powersave = 1;
}

void AsyncGSM::disablePowerSave() {
  enable_powersave = 0;
}

uint8_t AsyncGSM::isGprsEnabled() {
  return gprs_state == GPRS_STATE_IP_STATUS; 
}

uint8_t AsyncGSM::isGprsDisabled() {
  return gprs_state == GPRS_STATE_UNKNOWN || gprs_state == GPRS_STATE_IP_INITIAL;
}

void AsyncGSM::connect(char * data, int port, int connection, int type) {
  memcpy(connectionState[connection].address, data, strlen(data) + 1);
  connectionState[connection].port = port;
  connectionState[connection].type = type;
  connectionState[connection].connect = 1;
}

void AsyncGSM::disconnect(int connection) {
  connectionState[connection].address[0] = NULL;
  connectionState[connection].port = 0;
  connectionState[connection].connect = 0;
}

uint8_t AsyncGSM::writeData(char * data, int len, int connection) {
  for (int i = 0; i < len; i++) {
    writeBuffer(&connectionState[connection].outboundCircular, data[i]);
  }
}

void AsyncGSM::queueAtCommand(char * command, uint32_t timeout) {
  GSM_DEBUG_PRINT(F("--> ")); GSM_DEBUG_PRINTLN(command);
  mySerial->println(command);
  last_command = millis();
  command_timeout = timeout;
  modem_state = STATE_WAITING_REPLY;
  GSM_DEBUG_PRINTLN(F("STATE_WAITING_REPLY"));
}

void AsyncGSM::queueAtCommand(GSMFlashStringPtr command, uint32_t timeout) {
  GSM_DEBUG_PRINT(F("--> ")); GSM_DEBUG_PRINTLN(command);
  mySerial->println(command);
  last_command = millis();
  command_timeout = timeout;
  modem_state = STATE_WAITING_REPLY;
  GSM_DEBUG_PRINTLN(F("STATE_WAITING_REPLY"));
}

time_t AsyncGSM::getCurrentTime() {
  return last_network_time + (millis() - last_network_time_update) / 1000;
}

uint8_t AsyncGSM::parseConnectionNumber(char * data) {
  data[1] = 0;
  return atoi(data);
}

void AsyncGSM::processIncomingModemByte (const byte inByte) {

  switch (inByte) {

  case '\n':   // end of text
    input_modem_line[input_modem_pos] = 0;  // terminating null byte

    // terminator reached! process input_line here ...
    process_modem_data (input_modem_line);

    // reset buffer for next time
    input_modem_pos = 0;
    break;

  case '\r':   // discard carriage return
    break;

  case '>':
    input_modem_line[input_modem_pos++] = inByte;
    input_modem_line[input_modem_pos++] = 0;
    process_modem_data(input_modem_line);
    input_modem_pos = 0;
    break;

  default:
    // keep adding if not full ... allow for terminating null byte
    if (input_modem_pos < (MAX_INPUT - 1))
      input_modem_line [input_modem_pos++] = inByte;
    break;

  }
}

#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )

static  const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31}; // API starts months from 1, this array starts from 0

time_t AsyncGSM::parseTime(char * data) {
  char year[3];
  char month[3];
  char day[3];
  char hour[3];
  char minute[3];
  char secs[3];
  memcpy(year, data, 2);
  year[2] = 0;
  memcpy(month, data + 3, 2);
  month[2] = 0;
  memcpy(day, data + 6, 2);
  day[2] = 0;
  memcpy(hour, data + 9, 2);
  hour[2] = 0;
  memcpy(minute, data + 12, 2);
  minute[2] = 0;
  memcpy(secs, data + 15, 2);
  secs[2] = 0;
  tmelements_t tm;
  tm.second = atoi(secs);
  tm.minute = atoi(minute);
  tm.hour = atoi(hour);
  tm.day = atoi(day);
  tm.month = atoi(month);
  tm.year = atoi(year) + 2000 - 1970;

  int i;
  uint32_t seconds;

  // seconds from 1970 till 1 jan 00:00:00 of the given year
  seconds= tm.year*(SECS_PER_DAY * 365);
  for (i = 0; i < tm.year; i++) {
    if (LEAP_YEAR(i)) {
      seconds +=  SECS_PER_DAY;   // add extra days for leap years
    }
  }

  // add days for this year, months start from 1
  for (i = 1; i < tm.month; i++) {
    if ( (i == 2) && LEAP_YEAR(tm.year)) {
      seconds += SECS_PER_DAY * 29;
    } else {
      seconds += SECS_PER_DAY * monthDays[i-1];  //monthDay array starts from 0
    }
  }
  seconds+= (tm.day-1) * SECS_PER_DAY;
  seconds+= tm.hour * SECS_PER_HOUR;
  seconds+= tm.minute * SECS_PER_MIN;
  seconds+= tm.second;
  return (time_t)seconds; 
}

void AsyncGSM::process_modem_data (char * data) {
  GSM_DEBUG_PRINT(F("<-- "));
  GSM_DEBUG_PRINTLN(data);

  if (strstr(data, "+CMT:") != 0) {
    command_state = COMMAND_UCR_CMT;
    modem_state = STATE_UCR;
  }

  if (command_state == COMMAND_UCR_CMT) {
    int index = 0;
    char * pch;
    pch = strtok (data, "\"");
    while (pch != NULL) {
      if (index == 1) {
	memcpy(messageBuffer.msisdn, pch, strlen(pch) + 1);
      } else if (index == 4) {
	messageBuffer.receive_time = parseTime(pch);
      }
      
      pch = strtok (NULL, "\"");
      index++;
    }
    command_state = COMMAND_UCR_CMT_DATA;
    return;
  }

  if (command_state == COMMAND_TEST_CCLK) {
    
    if (strstr(data, "+CCLK:") != 0) {
      last_network_time = parseTime(data + 8);
      last_network_time_update = millis();
      GSM_DEBUG_PRINT("current_time: ");
      GSM_DEBUG_PRINTLN(last_network_time);
    }
  }
  
  if (command_state == COMMAND_UCR_CMT_DATA) {
    memcpy(messageBuffer.message, data, strlen(data) + 1);
    messageBuffer.available = 1;
    modem_state = STATE_IDLE;
    GSM_DEBUG_PRINTLN(messageBuffer.msisdn);
    GSM_DEBUG_PRINTLN(messageBuffer.receive_time);
    GSM_DEBUG_PRINTLN(messageBuffer.message);
    GSM_DEBUG_PRINTLN("STATE_IDLE");
  }

  if (command_state == COMMAND_WRITE_CMGS) {
    if (strstr(data, "OK") != 0) {
      modem_state = STATE_IDLE;
      GSM_DEBUG_PRINTLN("STATE_IDLE");
    }

    if (strstr(data, ">") != 0) {
      GSM_DEBUG_PRINTLN(strlen(outboundMessage.message));
      GSM_DEBUG_PRINT(F("--> ")); 
      GSM_DEBUG_PRINTLN(outboundMessage.message);
      mySerial->write(outboundMessage.message, strlen(outboundMessage.message));
      mySerial->write("\x1A");
      mySerial->flush();
      outboundMessage.message[0] = 0;
      outboundMessage.msisdn[0] = 0;
    }
    
  }

  if (command_state == COMMAND_TEST_CREG) {
    if (strstr(data, "+CREG: 0,1") != 0) {
      creg = 2;
      return;
    } else if (strstr(data, "+CREG: 0,2") != 0) {
      creg = 1;
      return;
    }
  }
  
  if (command_state == COMMAND_WRITE_CIPSEND) {
    if (strstr(data, ">") != 0) {
      // write data
      int len = connectionState[currentconnection].outboundBytes;
      char data[len];
      for (int i = 0; i < len; i++) {
	readBuffer(&connectionState[currentconnection].outboundCircular, data + i);
      }
      GSM_DEBUG_PRINT(F("Writing to gsm serial"));
      GSM_DEBUG_PRINTLN(connectionState[currentconnection].outboundBytes);
      mySerial->write(data, len);
      mySerial->flush();
      GSM_DEBUG_PRINTLN(F("Write ok."));
      return;
    }

    if (strstr(data, "SEND OK") != 0) {
      modem_state = STATE_IDLE;
      currentconnection = -1;
      GSM_DEBUG_PRINTLN(F("STATE_IDLE"));
      return;
    }
  }
  
  if (command_state == COMMAND_WRITE_CIPSTART) {
    GSM_DEBUG_PRINTLN("COMMAND_WRITE_CIPSTART2");
    
    if (strstr(data, "CONNECT OK") != 0) {
      connectionState[currentconnection].connectionState = GPRS_STATE_CONNECT_OK;
      modem_state = STATE_IDLE;
      currentconnection = -1;
      GSM_DEBUG_PRINTLN(F("STATE_IDLE"));
      return;
    } else if (strstr(data, "CONNECT FAIL") != 0) {
      // tcp or udp connection failed
      uint8_t connectionNumber = parseConnectionNumber(data);
      GSM_DEBUG_PRINTLN(connectionNumber);
      connectionState[0].connectionState = GPRS_STATE_IP_INITIAL;
      modem_state = STATE_IDLE;
      GSM_DEBUG_PRINTLN(F("STATE_IDLE"));
    }

    if (strstr(data, "OK") != 0) {
      return;
    }

  }

  if (command_state == COMMAND_WRITE_CIPCLOSE) {
    if (strstr(data, "CLOSE OK") != 0) {
      // tcp or udp connection closed
      uint8_t connectionNumber = parseConnectionNumber(data);
      connectionState[connectionNumber].connectionState = GPRS_STATE_IP_INITIAL;
      modem_state = STATE_IDLE;
      GSM_DEBUG_PRINTLN(F("STATE_IDLE"));
    }
  }
  
  if (command_state == COMMAND_TEST_CNMI && strstr(data, "+CNMI:") != 0) {
    if (strstr(data, "+CNMI:2,2,0,0,0") == 0) {
      cnmi = 1;
    }
  }

  if (command_state == COMMAND_TEST_CMGF) {
    if (strstr(data, "+CMGF: 1") != 0) {
      cmgf = 2;
    } else if (strstr(data, "+CMGF: 0") != 0) {
      cmgf = 1;
    }
  }

  if (command_state == COMMAND_CIPSHUT && strstr(data, "SHUT OK") != 0) {
    gprs_state = GPRS_STATE_IP_INITIAL;
    modem_state = STATE_IDLE;
    GSM_DEBUG_PRINTLN(F("STATE_IDLE"));
  }

  if (command_state == COMMAND_CIFSR && strlen(data) > 0) {
    gprs_state = GPRS_STATE_IP_STATUS;
    modem_state = STATE_IDLE;
    GSM_DEBUG_PRINTLN(F("STATE_IDLE"));
  }
  
  if (command_state == COMMAND_TEST_CIPMUX && strstr(data, "+CIPMUX:") != 0) {
    if (strstr(data, "+CIPMUX: 1") != 0) {
      cipmux = 2;
    } else if (strstr(data, "+CIPMUX: 0") != 0) {
      cipmux = 1;
    }
  }
  
  if (strcmp(data, "OK") == 0) {
    if (!autobauding) {
      autobauding = 1;
    }

    if (command_state == COMMAND_ENABLE_POWERSAVE) {
      powersave = 1;
    }

    if (command_state == COMMAND_DISABLE_POWERSAVE) {
      powersave = 0;
    }
    
    if (command_state == COMMAND_ATA) {
      callinprogress = 1;
      answerincomingcall = 0;
    }
    
    if (command_state == COMMAND_ATE) {
      echo = 1;
    }

    if (command_state == COMMAND_WRITE_CIPMUX) {
      cipmux = 2;
    }

    if (command_state == COMMAND_SET_CSTT) {
      gprs_state = GPRS_STATE_IP_START;
    }

    if (command_state == COMMAND_WRITE_CNMI) {
      cnmi = 2;
    }

    if (command_state == COMMAND_SET_CIICR) {
      gprs_state = GPRS_STATE_IP_GPRSACT;
    }

    if (command_state == COMMAND_SET_CLTS) {
      clts = 1;
    }

    if (command_state == COMMAND_WRITE_CMGF) {
      cmgf = 2;
    }

    if (command_state == COMMAND_WRITE_CSCS) {
      cscs = 1;
    }

    if (command_state == COMMAND_WRITE_CLIP) {
      clip = 1;
    }
    
    modem_state = STATE_IDLE;
    GSM_DEBUG_PRINTLN(F("STATE_IDLE"));
  }

  if (strstr(data, "RING") != 0) {
    incomingcall = 1;
  }

  if (strstr(data, "NO CARRIER") != 0) {
    incomingcall = 0;
    callinprogress = 0;
    answerincomingcall = 0;
  }

  if (strstr(data, "SMS Ready") != 0) {
    GSM_DEBUG_PRINTLN(F("resetModemState()"));
    resetModemState();
  }
  
  if (strstr(data, "ERROR") != 0) {
    modem_state = STATE_ERROR;
    GSM_DEBUG_PRINTLN(F("STATE_ERROR"));

    // reset modem

  }

  if (strstr(data, "CLOSED") != 0) {
    // tcp or udp connection closed
    uint8_t connectionNumber = parseConnectionNumber(data);
    connectionState[0].connectionState = GPRS_STATE_IP_INITIAL;
  }

  if (strstr(data, "+RECEIVE") != 0) {
    data[strlen(data) - 1] = NULL;
    data[10] = NULL;
    uint8_t connectionNumber = atoi(data + 9);
    uint16_t availableData = atoi(data + 11);
    GSM_DEBUG_PRINTLN(connectionNumber);
    GSM_DEBUG_PRINTLN(availableData);
    command_state = COMMAND_UCR_RECEIVE;
    GSM_DEBUG_PRINTLN(F("COMMAND_UCR_RECEIVE"));
    return;
  }

  
}
