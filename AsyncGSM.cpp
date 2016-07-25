/*
  AsyncGSM.cpp
*/

#include "Arduino.h"
#include "AsyncGSM.h"

#define GSM_DEBUG_PRINT(...) debugStream->print(__VA_ARGS__)
#define GSM_DEBUG_PRINTLN(...) debugStream->println(__VA_ARGS__)

AsyncGSM::AsyncGSM(int8_t rst)
{
  ok_reply = F("OK");
  rstpin = rst;
  modem_state = STATE_IDLE;
  autobauding = 0;
  echo = 0;
  cnmi = 0;
  command_state = COMMAND_NONE;
  currentconnection = -1;
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
    return 32 - buffer->tail + buffer->head;
  }
  return 0;
}

uint8_t AsyncGSM::dataAvailable(int connection) {
  return bufferSize(&connectionState[0].inboundCircular);
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
  while(mySerial->available()) {
    processIncomingModemByte(mySerial->read());
  }

  // check for timeout
  if (modem_state == STATE_WAITING_REPLY && millis() > last_command + 240000) {
    GSM_DEBUG_PRINTLN(F("TIMEOUT"));
    modem_state = STATE_IDLE;
    //delay(10000);
  }
  
  if (modem_state == STATE_IDLE && !autobauding) {
    queueAtCommand(F("AT"));
    return;
  }
  
  if (modem_state == STATE_IDLE && !echo && autobauding) {
    queueAtCommand(F("ATE1"));
    return;
  }

  if (modem_state == STATE_IDLE && incomingcall && answerincomingcall) {
    queueAtCommand(F("ATA"));
    return;
  }
  
  if (modem_state == STATE_IDLE && (millis() > last_battery_update + 120000) && autobauding) {
    queueAtCommand(F("AT+CBC"));
    last_battery_update = millis();
    return;
  }

  if (modem_state == STATE_IDLE && !clts && autobauding) {
    queueAtCommand(F("AT+CLTS=1"));
    return;
  }

  if (modem_state == STATE_IDLE && !clip && autobauding) {
    queueAtCommand(F("AT+CLIP=1"));
    return;
  }
  
  if (modem_state == STATE_IDLE && cipmux == 1 && autobauding && creg == 2) {
    queueAtCommand(F("AT+CIPMUX=1"));
    return;
  }

  if (modem_state == STATE_IDLE && !cipmux && autobauding && creg == 2) {
    queueAtCommand(F("AT+CIPMUX?"));
    return;
  }

  if (modem_state == STATE_IDLE && gprs_state == GPRS_STATE_UNKNOWN && autobauding && creg == 2) {
    queueAtCommand(F("AT+CIPSHUT"));
    return;
  }
  
  if (modem_state == STATE_IDLE && gprs_state == GPRS_STATE_IP_INITIAL && autobauding && creg == 2) {
    queueAtCommand(F("AT+CSTT=\"internet\",\"\",\"\""));
    return;
  }

  if (modem_state == STATE_IDLE && !cnmi && autobauding) {
    queueAtCommand(F("AT+CNMI?"));
    return;
  }

  if (modem_state == STATE_IDLE && !cmgf && autobauding) {
    queueAtCommand(F("AT+CMGF?"));
    return;
  }

  if (modem_state == STATE_IDLE && cmgf == 1 && autobauding) {
    // text mode sms
    queueAtCommand(F("AT+CMGF=1"));
    return;
  }

  if (modem_state == STATE_IDLE && !cscs && autobauding) {
    // text mode sms
    queueAtCommand(F("AT+CSCS=\"8859-1\""));
    return;
  }
  
  if (modem_state == STATE_IDLE && cnmi == 1 && autobauding) {
    queueAtCommand(F("AT+CNMI=2,2,0,0,0"));
    return;
  }

  if (modem_state == STATE_IDLE && (millis() > last_csq_update + 90000) && autobauding) {
    queueAtCommand(F("AT+CSQ"));
    last_csq_update = millis();
    return;
  }

  if (modem_state == STATE_IDLE && (millis() > last_time_update + 120000) && autobauding) {
    queueAtCommand(F("AT+CCLK?"));
    last_time_update = millis();
    return;
  }

  if (modem_state == STATE_IDLE && (millis() > last_creg + 60000) && autobauding && creg < 2) {
    queueAtCommand(F("AT+CREG?"));
    last_creg = millis();
    return;
  }
  
  if (modem_state == STATE_IDLE && gprs_state == GPRS_STATE_IP_START && autobauding && creg == 2) {
    queueAtCommand(F("AT+CIICR"));
    return;
  }

  if (modem_state == STATE_IDLE && gprs_state == GPRS_STATE_IP_GPRSACT && autobauding && creg == 2) {
    queueAtCommand(F("AT+CIFSR"));
    return;
  }

  for (int i = 0; i < NELEMS(connectionState); i++) {
    if (modem_state == STATE_IDLE && 
	gprs_state == GPRS_STATE_IP_STATUS && 
	connectionState[i].connectionState != GPRS_STATE_CONNECT_OK && 
	creg == 2 &&
	strlen(connectionState[i].address) > 0 &&
	connectionState[i].port != 0 &&
	connectionState[i].connect) {
      char command[32];
      sprintf(command, "AT+CIPSTART=%u,\"%s\",\"%s\",%u",
	      i,
	      connectionState[i].type == CONNECTION_TYPE_TCP ? "TCP" : "UDP",
	      connectionState[i].address,
	      connectionState[i].port); 
      queueAtCommand(command);
      currentconnection = i;
      return;
    }
  }

  for (int i = 0; i < NELEMS(connectionState); i++) {
    if (modem_state == STATE_IDLE && 
	gprs_state == GPRS_STATE_IP_STATUS && 
	connectionState[i].connectionState == GPRS_STATE_CONNECT_OK && 
	creg == 2 &&
	!connectionState[i].connect) {
      char command[32];
      sprintf(command, "AT+CIPCLOSE=%u", i);
      queueAtCommand(command);
      currentconnection = i;
      return;
    }
  }

  for (int j = 0; j < NELEMS(connectionState); j++) {
    if (modem_state == STATE_IDLE && 
	gprs_state == GPRS_STATE_IP_STATUS && 
	connectionState[j].connectionState == GPRS_STATE_CONNECT_OK && 
	bufferSize(&connectionState[j].outboundCircular) > 0 && creg == 2) {
      char command[16];
      sprintf(command, "AT+CIPSEND=%u,%u", j, bufferSize(&connectionState[j].outboundCircular));
      queueAtCommand(command);
      currentconnection = j;
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
    queueAtCommand(command);
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
  GSM_DEBUG_PRINTLN(connectionState[connection].outboundCircular.buffer);
  GSM_DEBUG_PRINTLN(bufferSize(&connectionState[connection].outboundCircular));
}

void AsyncGSM::queueAtCommand(char * command) {
  GSM_DEBUG_PRINT(F("--> ")); GSM_DEBUG_PRINTLN(command);
  mySerial->println(command);
  last_command = millis();
  modem_state = STATE_WAITING_REPLY;
  GSM_DEBUG_PRINTLN(F("STATE_WAITING_REPLY"));
}

void AsyncGSM::queueAtCommand(GSMFlashStringPtr command) {
  GSM_DEBUG_PRINT(F("--> ")); GSM_DEBUG_PRINTLN(command);
  mySerial->println(command);
  last_command = millis();
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
  GSM_DEBUG_PRINTLN(data);

  if (prog_char_strstr(data, (prog_char *)F("ATE")) != 0) {
    command_state = COMMAND_ATE;
  } else if (prog_char_strstr(data, (prog_char *)F("ATA")) != 0) {
    command_state = COMMAND_ATA;
    return;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CIPMUX=")) != 0) {
    command_state = COMMAND_WRITE_CIPMUX;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CSTT=")) != 0) {
    command_state = COMMAND_SET_CSTT;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CNMI?")) != 0) {
    command_state = COMMAND_TEST_CNMI;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CMGF?")) != 0) {
    command_state = COMMAND_TEST_CMGF;
    return;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CLIP=")) != 0) {
    command_state = COMMAND_WRITE_CLIP;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CMGF=")) != 0) {
    command_state = COMMAND_WRITE_CMGF;
    return;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CSCS=")) != 0) {
    command_state = COMMAND_WRITE_CSCS;
    return;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CNMI=")) != 0) {
    command_state = COMMAND_WRITE_CNMI;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CIPMUX?")) != 0) {
    command_state = COMMAND_TEST_CIPMUX;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CIPSHUT")) != 0) {
    command_state = COMMAND_CIPSHUT;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CBC")) != 0) {
    command_state = COMMAND_CBC;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CIICR")) != 0) {
    command_state = COMMAND_SET_CIICR;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CSQ")) != 0) {
    command_state = COMMAND_CSQ;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CLTS=")) != 0) {
    command_state = COMMAND_SET_CLTS;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CCLK?")) != 0) {
    command_state = COMMAND_TEST_CCLK;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CREG?")) != 0) {
    command_state = COMMAND_TEST_CREG;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CIFSR")) != 0) {
    command_state = COMMAND_CIFSR;
    return;
  } else if (prog_char_strstr(data, (prog_char *)F("AT+CIPCLOSE")) != 0) {
    command_state = COMMAND_WRITE_CIPCLOSE;
    return;    
  } else if (strstr(data, "AT+CMGS=") != 0) {
    command_state = COMMAND_WRITE_CMGS;
    return;
  } else if (strstr(data, "AT+CIPSTART=") != 0) {
    command_state = COMMAND_WRITE_CIPSTART;
    return;
  } else if (strstr(data, "AT+CIPSEND=") != 0) {
    command_state = COMMAND_WRITE_CIPSEND;
    return;
  } else if (strstr(data, "+CMT:") != 0) {
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
      char data[16];
      int len = bufferSize(&connectionState[currentconnection].outboundCircular);
      GSM_DEBUG_PRINT(F("Circular buffer size: "));
      GSM_DEBUG_PRINTLN(len);
      for (int i = 0; i < len; i++) {
	readBuffer(&connectionState[currentconnection].outboundCircular, data + i);
	GSM_DEBUG_PRINT(data[i]);
      }
      mySerial->write(data, len);
      mySerial->flush();
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
      connectionState[0].connectionState = GPRS_STATE_IP_INITIAL;
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
    return;
  }

  
}
