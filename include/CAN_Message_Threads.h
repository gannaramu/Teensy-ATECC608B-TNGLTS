#define USE_THREAD_NAMES 1
#define DEBUG true
#include <Thread.h>
#include <ThreadController.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

uint32_t shortest_period = 10;
uint8_t source_address = 0xFA;
int comp_id_index = 0;
String commandPrefix;
String commandString;
IntervalTimer CANTimer;
static CAN_message_t rxmsg;
static CAN_message_t txmsg;
static CAN_message_t temp_txmsg;
//set up a counter for each received message
uint32_t RXCount0 = 0;
uint32_t RXCount1 = 0;
uint32_t RXCount2 = 0;
uint16_t J1708RXCount = 0;

//set up a counter for each received message
uint32_t TXCount0 = 0;
uint32_t TXCount1 = 0;
uint32_t TXCount2 = 0;
uint16_t J1708TXCount = 0;
ThreadController can_thread_controller = ThreadController();

DynamicJsonDocument can_gen_doc(2048);

#define num_default_messages 21
String default_messages[num_default_messages] = {
    F("DDEC MCM 01;                   1;1;0;2;  10;   0;0;1; 8FF0001;8;01;02;03;04;05;06;07;08\n"), //DDEC 13 MCM message; CAN1
    F("DDEC TCM 01;                   2;1;0;2;  10;   0;0;1; CF00203;8; 9; a; b; c; d; e; f; 0\n"), //DDEC13 Transmission controller message; CAN1
    F("DDEC TCM 02;                   3;1;0;2;  10;   0;0;1; 8FF0303;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //DDEC 13 TCM message; CAN1
    F("DDEC TCM 03;                   4;1;0;2; 100;   0;0;1;18F00503;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Transmission on DDEC 13
    F("HRW from Brake Controller;     5;1;0;0;  20;   0;0;1; CFE6E0B;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //High Resolution wheel speed message from SA=11 (brake controller)
    F("EBC1 from Cab Controller;      6;1;0;0; 100;   0;0;1;18F00131;8; 0; 0; 0; 0; 0; 0; 0; 0"),   // Electronic Brake Controller from SA=49
    F("EBC1 from Brake Controller;    7;1;0;0; 100;   0;0;1;18F0010B;8; 0; 0; 0; 0; 0; 0; 0; 0"),   // Electronic Brake Controller from SA=11
    F("CCVS1 from Instrument Cluster; 8;1;0;0; 100;   0;0;1;18FEF117;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cruise Control/Vehicle Speed from SA=23
    F("CCVS1 from Cab Display 1;      9;1;0;0; 100;   0;0;1;18FEF128;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cruise Control/Vehicle Speed from SA=40
    F("CCVS1 from Body Controller;   10;1;0;0; 100;   0;0;1;18FEF121;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cruise Control/Vehicle Speed from SA=33
    F("CCVS1 from Cab Controller;    11;1;0;0; 100;   0;0;1;18FEF131;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cruise Control/Vehicle Speed from SA=49
    F("CM1 from Instrument Cluster;  12;1;0;0; 100;   0;0;1;18E00017;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cab Message 1 from SA=23
    F("CM1 from Climate Control 1;   13;1;0;0; 100;   0;0;1;18E00019;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cab Message 1 from SA=25
    F("CM1 from Body Controller;     14;1;0;0; 100;   0;0;1;18E00021;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cab Message 1 from SA=33
    F("CM1 from Cab Display;         15;1;0;0; 100;   0;0;1;18E00028;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cab Message 1 from SA=40
    F("CM1 from Cab Controller;      16;1;0;0; 100;   0;0;1;18E00031;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cab Message 1 from SA=49
    F("PTO from Instrument Cluster;  17;1;0;0; 100;   0;0;1;18FEF017;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cruise and PTO setup from SA=23
    F("PTO from Body Controller;     18;1;0;0; 100;   0;0;1;18FEF021;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cruise and PTO setup from SA=33
    F("PTO from Cab Display;         19;1;0;0; 100;   0;0;1;18FEF028;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cruise and PTO setup from SA=40
    F("PTO from Cab Controller;      20;1;0;0; 100;   0;0;1;18FEF031;8; 0; 0; 0; 0; 0; 0; 0; 0"),   //Cruise and PTO setup from SA=49
    F("AMB from Body Controller;     21;1;0;0;1000;   0;0;1;18FEF521;8; 0; 0; 0; 0; 0; 0; 0; 0")    //Ambient Conditions
};

void send_can_messages(CAN_message_t txmsg, uint8_t can_channel)
{
  if (true)
  {
    // Serial.println("**************************************************** Send CAN Message not implemented");
    Serial.print(" Sent CAN Message: ");
    Serial.print(txmsg.id, HEX);
    Serial.print(" ");
    Serial.print(txmsg.len);
    Serial.print(" Data: ");

    for (int i = 0; i < txmsg.len; i++)
    {
      Serial.print(txmsg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  if (can_channel == 0)
  {
    Can1.write(txmsg);
    TXCount0++;
    // memcpy(&status_buffer_2[CAN0_TX_COUNT_LOC], &TXCount0, 4);
  }
  else if (can_channel == 1)
  {
    Can2.write(txmsg);
    TXCount1++;
    // memcpy(&status_buffer_2[CAN1_TX_COUNT_LOC], &TXCount1, 4);
  }
  // else if (can_channel == 2) {
  //   // MCPCAN.sendMsgBuf(txmsg.id, txmsg.ext, txmsg.len, txmsg.buf);
  //   // TXCount2++;
  //   // memcpy(&status_buffer_2[CAN2_TX_COUNT_LOC], &TXCount2, 4);
  // }
}

class CanThread : public Thread
{
public:
  uint32_t stop_after_count;
  uint32_t transmit_number = 0;
  boolean ok_to_send = true;
  uint32_t loop_cycles = 0;
  uint32_t cycle_count = 0;
  uint32_t tx_period = 0;
  uint8_t channel = 0;

  CAN_message_t txmsg;

  uint8_t num_messages = 1;
  uint8_t message_index = 0;
  uint8_t len_list[256] = {};
  uint8_t message_list[256][8] = {};
  uint32_t id_list[256] = {};
  bool debug = false;
  bool shouldRun(unsigned long time)
  {
    if (stop_after_count > 0)
    {
      if (transmit_number >= stop_after_count)
        enabled = false;
    }
    return Thread::shouldRun(time);
  }

  void run()
  {
    //Set the CAN message data to the next one in the list.
    txmsg.id = id_list[message_index];
    txmsg.len = len_list[message_index];
    memcpy(txmsg.buf, message_list[message_index], 8);
    if (debug)
    {
      Serial.print("(" + ThreadName + ")" + "|" + String(txmsg.id, HEX) + " " + String(txmsg.len) + "|");
      for (int i = 0; i < txmsg.len; i++)
      {
        Serial.print(String(txmsg.buf[i], HEX) + " ");
      }
      Serial.print("|");
      Serial.println("");
    }
    //Write the data to the CAN bus.
    if (ok_to_send && ignitionCtlState)
    {

      send_can_messages(txmsg, channel);
      transmit_number++;
      message_index++;
    }
    cycle_count++;
    if (message_index >= num_messages)
    {
      message_index = 0;
    }
    if (cycle_count >= num_messages)
    {
      ok_to_send = false;
    }
    if (cycle_count * interval >= loop_cycles)
    {
      cycle_count = 0;
      ok_to_send = true;
    }
    return Thread::run();
  }
};

void runCANthreads()
{
  can_thread_controller.run();
}

CanThread *can_messages[MAX_THREADS] = {};

// Create a thread controller class
// ThreadController can_thread_controller = ThreadController();

int setupPeriodicCANMessage()
{

  CANTimer.end();
  CanThread *can_message;
  int16_t index;
  int16_t sub_index;
  uint8_t channel;
  uint32_t tx_period;
  uint32_t tx_delay;
  uint8_t num_messages = 1;
  uint32_t stop_after_count;
  if (DEBUG)
    Serial.println("***************Entered setupPeriodicCANMessage*******************");
  Serial.println(commandString);

  uint16_t threadSize = can_thread_controller.size(false);
  char commandBytes[128];
  commandString.toCharArray(commandBytes, sizeof(commandBytes));
  char delimiter[] = ";";
  char *commandValues;

  commandValues = strtok(commandBytes, delimiter);
  String threadName = commandValues;

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    index = constrain(atoi(commandValues), 0, threadSize + 1);
  }
  else
  {
    Serial.println(("ERROR SM command is missing arguments."));
    return -1;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    num_messages = constrain(atoi(commandValues), 1, 255);
  }
  else
  {
    Serial.println(("ERROR SM command not able to determine the number of sub messages."));
    return -2;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    //constrain the sub_index to be one larger than the
    sub_index = constrain(atoi(commandValues), 0, num_messages - 1);
  }
  else
  {
    Serial.println(("ERROR SM command missing sub_index."));
    return -3;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    channel = constrain(atoi(commandValues), 0, 2);
  }
  else
  {
    Serial.println(("ERROR SM command not able to set CAN Channel."));
    return -4;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    tx_period = constrain(strtoul(commandValues, NULL, 10), shortest_period, 0xFFFFFFFF);
  }
  else
  {
    Serial.println(("ERROR SM command not able to set period information."));
    return -5;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    tx_delay = strtoul(commandValues, NULL, 10);
  }
  else
  {
    Serial.println(("ERROR SM command not able to set delay information."));
    return -6;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    stop_after_count = strtoul(commandValues, NULL, 10);
  }
  else
  {
    Serial.println(("ERROR SM command not able to set the total number count."));
    return -7;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    temp_txmsg.flags.extended = constrain(atoi(commandValues), 0, 1);
  }
  else
  {
    Serial.println(("ERROR SM command not able to set extended ID flag."));
    temp_txmsg.flags.extended = 1;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    temp_txmsg.id = strtoul(commandValues, NULL, 16);
  }
  else
  {
    Serial.println(("WARNING SM command not able to set CAN ID information."));
    temp_txmsg.id = 0x3FFFFFFF;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    temp_txmsg.len = constrain(atoi(commandValues), 0, 8);
  }
  else
  {
    Serial.println(("WARNING SM command not able to set CAN data length code."));
    temp_txmsg.len = 8;
  }

  for (int i = 0; i < temp_txmsg.len; i++)
  {
    commandValues = strtok(NULL, delimiter);
    if (commandValues != NULL)
    {
      temp_txmsg.buf[i] = constrain(strtol(commandValues, NULL, 16), 0, 255);
    }
    else
    {
      temp_txmsg.buf[i] = 0xFF;
      Serial.printf("WARNING SM command not able to set CAN data byte in position %d.\n", i);
    }
  }
  char threadNameChars[25]; //25 (24 chars + /00) is the size of the message frame space for the thread name
  threadName.toCharArray(threadNameChars, sizeof(threadNameChars));

  if (index >= threadSize)
  { //Create a new entry
    index = threadSize;
    Serial.printf("THREAD %lu, %s (NEW)\n", index, threadNameChars);
    CanThread *can_message = new CanThread();
    can_thread_controller.add(can_message);
    can_messages[index] = can_message;
    can_messages[index]->enabled = false;
  }
  else
  {
    Serial.printf("THREAD %lu, %s (EXISTING)\n", index, threadNameChars);
  }

  if (DEBUG)
    Serial.printf("SET CAN name=%s \n index=%d\n num_messages=%d\n sub_index=%d\n channel=%d\n tx_period=%d\n tx_delay=%d\n stop_after_count=%d\n temp_txmsg.ext=%d\n ID=%08X\n DLC=%d\n DATA=[",
                  threadNameChars, index, num_messages, sub_index, channel, tx_period, tx_delay, stop_after_count, temp_txmsg.flags.extended, temp_txmsg.id, temp_txmsg.len);
  for (int i = 0; i < temp_txmsg.len - 1; i++)
  {
    Serial.printf("%02X, ", temp_txmsg.buf[i]);
  }
  if (DEBUG)
    Serial.printf("%02X]\n", temp_txmsg.buf[temp_txmsg.len - 1]);

  can_messages[index]->channel = channel;
  can_messages[index]->txmsg.flags.extended = temp_txmsg.flags.extended;
  can_messages[index]->len_list[sub_index] = temp_txmsg.len;
  can_messages[index]->id_list[sub_index] = temp_txmsg.id;
  for (int i = 0; i < temp_txmsg.len; i++)
  {
    can_messages[index]->message_list[sub_index][i] = temp_txmsg.buf[i];
  }
  can_messages[index]->stop_after_count = stop_after_count;
  can_messages[index]->transmit_number = 0;
  can_messages[index]->cycle_count = 0;
  can_messages[index]->message_index = 0;
  can_messages[index]->num_messages = num_messages;
  can_messages[index]->tx_period = tx_period;
  can_messages[index]->setInterval(tx_period);
  can_messages[index]->loop_cycles = tx_delay;
  can_messages[index]->ThreadName = threadName;

  CANTimer.begin(runCANthreads, 500);
  return index;
}

void setupComponentInfo()
{
  char byteEntry[4];
  uint8_t old_shortest_period = shortest_period;
  shortest_period = 1;
  uint16_t id_length = constrain(componentID.length(), 0, 7 * 256 - 1);
  char id[7 * 256];
  componentID.toCharArray(id, id_length + 1);
  uint8_t num_frames = id_length / 7;
  if (id_length % 7 > 0)
    num_frames++;
  char bytes_to_send[4];
  sprintf(bytes_to_send, "%02X", id_length);
  char frames_to_send[3];
  sprintf(frames_to_send, "%02X", num_frames);
  commandString = "CI from SSS2;0;1;0;0;1;0;1;1;18ECFF";
  sprintf(byteEntry, "%02X;", source_address);
  commandString += byteEntry;
  commandString += "8;20;";
  commandString += bytes_to_send;
  commandString += ";0;";
  commandString += frames_to_send;
  commandString += ";FF;EB;FE;00";

  setupPeriodicCANMessage();

  for (int i = 0; i < num_frames; i++)
  {
    commandString = "CI from SSS2;0;";
    sprintf(byteEntry, "%d;", num_frames + 1);
    commandString += byteEntry;
    sprintf(byteEntry, "%d;", i + 1);
    commandString += byteEntry;
    commandString += "0;1;0;";
    sprintf(byteEntry, "%d;", num_frames + 1);
    commandString += byteEntry;
    commandString += "1;18EBFF";
    sprintf(byteEntry, "%02X;", source_address);
    commandString += byteEntry;
    commandString += "8;";
    sprintf(byteEntry, "%02X;", i + 1);
    commandString += byteEntry;
    for (int j = 7 * i; j < 7 * i + 7; j++)
    {
      if (j < id_length)
        sprintf(byteEntry, "%02X;", id[j]);
      else
        sprintf(byteEntry, "%02X;", 0xFF);
      commandString += byteEntry;
    }
    comp_id_index = setupPeriodicCANMessage();
  }
  shortest_period = old_shortest_period;
}

void goCAN()
{
  int threadSize = can_thread_controller.size(false);
  for (int i = 0; i < threadSize; i++)
  {
    can_messages[i]->enabled = true;
    can_messages[i]->transmit_number = 0;
    can_messages[i]->message_index = 0;
    can_messages[i]->cycle_count = 0;
  }
  Serial.println(("INFO Started all CAN transmissions."));
}

void reloadCAN()
{
  setupComponentInfo();

  for (int i = 0; i < num_default_messages; i++)
  {
    commandString = default_messages[i];
    setupPeriodicCANMessage();
    delayMicroseconds(800);
  }
  commandString = "1";
  goCAN();
}

int createNewCANThread()
{
  uint16_t threadSize = can_thread_controller.size(false);
  String threadName = "New Thread;";
  String empty_messages = ";1;0;0; 100;   0;0;1;18E00021;8; 0; 0; 0; 0; 0; 0; 0; 0"; //DDEC 13 MCM message; CAN1
  int16_t index = threadSize + 1;
  String Final = threadName + index + empty_messages;
  commandString = Final;
  setupPeriodicCANMessage();
  delayMicroseconds(800);
}

void stopCAN()
{
  int threadSize = can_thread_controller.size(false);
  for (int i = 0; i < threadSize; i++)
  {
    can_messages[i]->enabled = false;
  }
  Serial.println(("INFO Stopped all CAN transmission."));
}

void clearCAN()
{
  int threadSize = can_thread_controller.size(false);
  for (int i = 1; i < threadSize; i++)
  { //Leave 0 in place because it it component ID
    delete can_messages[i];
  }
  can_thread_controller.clear();
  Serial.println(("INFO Cleared the CAN transmission thread. All messages must be reloaded."));
}

void startCAN()
{
  int threadSize = can_thread_controller.size(false);
  if (threadSize > 0)
  {
    char commandBytes[40];
    commandString.toCharArray(commandBytes, 100);
    Serial.print("Command Bytes:   ");
    Serial.println(commandBytes);
    char delimiter[] = ";";
    char *commandValues;
    // commandValues = strtok(commandBytes, delimiter);
    commandValues = strtok(NULL, delimiter);
    Serial.print("Command Values:   ");
    Serial.println(commandValues);

    commandValues = strtok(commandBytes, delimiter);
    Serial.print("Command Values:   ");
    Serial.println(commandValues);

    int index = constrain(atoi(commandValues), 0, threadSize - 1);
    Serial.print("index: ");
    Serial.println(index);
    commandValues = strtok(NULL, delimiter);
    int setting = atoi(commandValues);
    Serial.print("setting: ");
    Serial.println(setting);
    if (setting > 0)
    {
      can_messages[index]->enabled = true;
      can_messages[index]->transmit_number = 0;
      can_messages[index]->message_index = 0;
      can_messages[index]->cycle_count = 0;
      Serial.printf("SET CAN message %d with ID 0x%08X on.\n", index, can_messages[index]->id_list[0]);
    }
    else
    {
      can_messages[index]->enabled = false;
      Serial.printf("SET CAN message %d with ID 0x%08X off.\n", index, can_messages[index]->id_list[0]);
    }
  }
  else
  {
    //Serial.println("ERROR No CAN Messages Setup to turn on.");
  }
}

int UpdatePeriodicCANMessage()
{
  CANTimer.end();
  CanThread *can_message;
  int16_t index;
  int16_t sub_index;
  uint8_t channel;
  uint32_t tx_period;
  uint32_t tx_delay;
  uint8_t num_messages = 1;
  uint32_t stop_after_count;
  Serial.println("***************Entered Update Periodic CANMessage*******************");
  Serial.println(commandString);

  uint16_t threadSize = can_thread_controller.size(false);
  char commandBytes[128];
  commandString.toCharArray(commandBytes, sizeof(commandBytes));
  char delimiter[] = ";";
  char *commandValues;

  // commandValues = strtok(commandBytes, delimiter);
  // String threadName = commandValues;

  commandValues = strtok(commandBytes, delimiter);
  if (commandValues != NULL)
  {
    index = constrain(atoi(commandValues), 0, threadSize + 1);
  }
  else
  {
    Serial.println(("ERROR SM command is missing arguments."));
    return -1;
  }
  String threadName = can_messages[index]->ThreadName;
  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    num_messages = constrain(atoi(commandValues), 1, 255);
  }
  else
  {
    Serial.println(("ERROR SM command not able to determine the number of sub messages."));
    return -2;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    //constrain the sub_index to be one larger than the
    sub_index = constrain(atoi(commandValues), 0, num_messages - 1);
  }
  else
  {
    Serial.println(("ERROR SM command missing sub_index."));
    return -3;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    channel = constrain(atoi(commandValues), 0, 2);
  }
  else
  {
    Serial.println(("ERROR SM command not able to set CAN Channel."));
    return -4;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    tx_period = constrain(strtoul(commandValues, NULL, 10), shortest_period, 0xFFFFFFFF);
  }
  else
  {
    Serial.println(("ERROR SM command not able to set period information."));
    return -5;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    tx_delay = strtoul(commandValues, NULL, 10);
  }
  else
  {
    Serial.println(("ERROR SM command not able to set delay information."));
    return -6;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    stop_after_count = strtoul(commandValues, NULL, 10);
  }
  else
  {
    Serial.println(("ERROR SM command not able to set the total number count."));
    return -7;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    temp_txmsg.flags.extended = constrain(atoi(commandValues), 0, 1);
  }
  else
  {
    Serial.println(("ERROR SM command not able to set extended ID flag."));
    temp_txmsg.flags.extended = 1;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    temp_txmsg.id = strtoul(commandValues, NULL, 16);
  }
  else
  {
    Serial.println(("WARNING SM command not able to set CAN ID information."));
    temp_txmsg.id = 0x3FFFFFFF;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL)
  {
    temp_txmsg.len = constrain(atoi(commandValues), 0, 8);
  }
  else
  {
    Serial.println(("WARNING SM command not able to set CAN data length code."));
    temp_txmsg.len = 8;
  }

  for (int i = 0; i < temp_txmsg.len; i++)
  {
    commandValues = strtok(NULL, delimiter);
    if (commandValues != NULL)
    {
      temp_txmsg.buf[i] = constrain(strtol(commandValues, NULL, 16), 0, 255);
    }
    else
    {
      temp_txmsg.buf[i] = 0xFF;
      Serial.printf("WARNING SM command not able to set CAN data byte in position %d.\n", i);
    }
  }
  char threadNameChars[25]; //25 (24 chars + /00) is the size of the message frame space for the thread name
  threadName.toCharArray(threadNameChars, sizeof(threadNameChars));
  Serial.printf("THREAD %lu, %s (EXISTING)\n", index, threadNameChars);
  Serial.printf("SET CAN name=%s \n index=%d\n num_messages=%d\n sub_index=%d\n channel=%d\n tx_period=%d\n tx_delay=%d\n stop_after_count=%d\n temp_txmsg.ext=%d\n ID=%08X\n DLC=%d\n DATA=[",
                threadNameChars, index, num_messages, sub_index, channel, tx_period, tx_delay, stop_after_count, temp_txmsg.flags.extended, temp_txmsg.id, temp_txmsg.len);
  for (int i = 0; i < temp_txmsg.len - 1; i++)
  {
    Serial.printf("%02X, ", temp_txmsg.buf[i]);
  }
  Serial.printf("%02X]\n", temp_txmsg.buf[temp_txmsg.len - 1]);

  can_messages[index]->channel = channel;
  can_messages[index]->txmsg.flags.extended = temp_txmsg.flags.extended;
  can_messages[index]->len_list[sub_index] = temp_txmsg.len;
  can_messages[index]->id_list[sub_index] = temp_txmsg.id;
  for (int i = 0; i < temp_txmsg.len; i++)
  {
    can_messages[index]->message_list[sub_index][i] = temp_txmsg.buf[i];
  }
  can_messages[index]->stop_after_count = stop_after_count;
  can_messages[index]->transmit_number = 0;
  can_messages[index]->cycle_count = 0;
  can_messages[index]->message_index = 0;
  can_messages[index]->num_messages = num_messages;
  can_messages[index]->tx_period = tx_period;
  can_messages[index]->setInterval(tx_period);
  can_messages[index]->loop_cycles = tx_delay;
  can_messages[index]->ThreadName = threadName;

  CANTimer.begin(runCANthreads, 500);
  return index;
}

int StrToHex(char str[])
{
  return (int)strtol(str, 0, 16);
}

void readCANGen(Request &req, Response &res)
{
  StaticJsonDocument<20000> response;
  char json[20000];

  int threadSize = can_thread_controller.size(false);
  for (int i = 0; i < threadSize; i++)
  { //Leave 0 in place because it it component ID
    for (int j = 0; j < can_messages[i]->num_messages; j++)
    {
      uint8_t sub_index = j;
      response[String(i) + String(j)]["ThreadName"] = can_messages[i]->ThreadName;
      response[String(i) + String(j)]["ThreadID"] = i;
      response[String(i) + String(j)]["enabled"] = can_messages[i]->enabled;
      response[String(i) + String(j)]["num_messages"] = can_messages[i]->num_messages;
      response[String(i) + String(j)]["message_index"] = sub_index;
      response[String(i) + String(j)]["transmit_number"] = can_messages[i]->transmit_number;
      response[String(i) + String(j)]["cycle_count"] = can_messages[i]->cycle_count;
      response[String(i) + String(j)]["channel"] = can_messages[i]->channel;
      response[String(i) + String(j)]["tx_period"] = can_messages[i]->tx_period;
      response[String(i) + String(j)]["tx_delay"] = can_messages[i]->loop_cycles;
      response[String(i) + String(j)]["stop_after_count"] = can_messages[i]->stop_after_count;
      response[String(i) + String(j)]["extended"] = can_messages[i]->txmsg.flags.extended;
      response[String(i) + String(j)]["ID"] = String(can_messages[i]->id_list[sub_index], HEX);
      response[String(i) + String(j)]["DLC"] = can_messages[i]->txmsg.len;
      // response[String(i)]["DATA"] = [];
      for (int k = 0; k < can_messages[i]->txmsg.len; k++)
      {
        response[String(i) + String(j)]["DATA"][k] = String(can_messages[i]->message_list[sub_index][k], HEX);
      }
    }
  }
  serializeJsonPretty(response, json);
  //debug.print(DBG_DEBUG, "Size of Response: %d", response.memoryUsage());
  res.print(json);
}

bool parse_response_can_gen_doc(uint8_t *buffer)
{
  // Serial.print((char *)buffer);
  // Serial.println(" EOF");
  //Serial.println("parse_response");
  Serial.println();
  DeserializationError error = deserializeJson(can_gen_doc, buffer);
  Serial.println("Response Size: " + String(can_gen_doc.memoryUsage()) + " bytes ");

  serializeJsonPretty(can_gen_doc, Serial);

  if (error)
  {
    if (DEBUG)
      Serial.print(F("deserializeJson() failed: "));
    if (DEBUG)
      // Serial.println(error.f_str());
      return false;
  }

  return true;
}

void updateCANGen(Request &req, Response &res)
{
  uint8_t buff_can[2048];
  char buff_can_c[2048];
  if (1)
    Serial.print("Got POST Request for CAN : ");
  req.body(buff_can, sizeof(buff_can));
  if (!parse_response_can_gen_doc(buff_can))
  {
    res.print("Not a valid Json Format");
  }
  else
  {
    int index = can_gen_doc["ThreadID"];

    can_messages[index]->enabled = can_gen_doc["enabled"];
    can_messages[index]->transmit_number = 0;
    can_messages[index]->message_index = 0;
    can_messages[index]->cycle_count = 0;

    can_messages[index]->ThreadName = (char *)can_gen_doc["ThreadName"];
    can_messages[index]->num_messages = can_gen_doc["num_messages"];
    uint8_t sub_index = can_gen_doc["message_index"];
    Serial.print("Sub Index: " + String(sub_index));
    // can_messages[index]->transmit_number = can_gen_doc["transmit_number"];
    // can_messages[index]->cycle_count = can_gen_doc["cycle_count"];
    can_messages[index]->channel = can_gen_doc["channel"];
    can_messages[index]->tx_period = can_gen_doc["tx_period"];
    can_messages[index]->loop_cycles = can_gen_doc["tx_delay"];
    can_messages[index]->stop_after_count = can_gen_doc["stop_after_count"];
    can_messages[index]->txmsg.flags.extended = can_gen_doc["extended"];
    const char *CAN_ID = can_gen_doc["ID"];
    Serial.print("sub_index: " + String(sub_index));
    Serial.print("ID: ");
    Serial.print(StrToHex(can_gen_doc["ID"]), HEX);
    can_messages[index]->id_list[sub_index] = StrToHex(can_gen_doc["ID"]);
    can_messages[index]->len_list[sub_index] = can_gen_doc["DLC"];
    for (int i = 0; i < can_messages[index]->len_list[sub_index]; i++)
    {
      can_messages[index]->message_list[sub_index][i] = StrToHex(can_gen_doc["DATA"][i]);
    }

    return readCANGen(req, res);
  }
}