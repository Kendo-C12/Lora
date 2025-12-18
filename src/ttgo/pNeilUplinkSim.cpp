#include <SPI.h>
#include <RadioLib.h>
#include <EEPROM.h>


// SX1276 pin connections for TTGO LoRa32 V1
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26
#define DIO1 13

constexpr struct {
    float center_freq = 925.000000f;  // MHz
    float bandwidth   = 125.f;     // kHz
    uint8_t spreading_factor = 9;  
    uint8_t coding_rate = 8;       
    uint8_t sync_word = 0x12;      
    int8_t power = 0;             
    uint16_t preamble_length = 16;
} lora_params;

// Initialize SX1276 module
SX1276 lora = new Module(SS, DIO0, RST, DIO0);

void loraSetup(){
  
  Serial.println("Initializing SX1262...");

  int16_t lora_state = lora.begin(
    lora_params.center_freq,
    lora_params.bandwidth,
    lora_params.spreading_factor,
    lora_params.coding_rate,
    lora_params.sync_word,
    lora_params.power,
    lora_params.preamble_length
  );

  if(lora_state != RADIOLIB_ERR_NONE) {
    Serial.print("Begin failed, code: ");
    
    while(true){
        Serial.println(lora_state);
        delay(1000);
    };
  }

  Serial.print("Lora Begin: ");
  Serial.println(lora_state);

  // configure module safely
  lora_state = lora.explicitHeader();
  Serial.print("ExplicitHeader: ");
  Serial.println(lora_state);


  lora_state = lora.setCRC(true);
  Serial.print("SetCRC: ");
  Serial.println(lora_state);

  lora_state = lora.autoLDRO();
  Serial.print("AutoLDRO: ");
  Serial.println(lora_state);


  if (lora_state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(lora_state);
    while (true) { delay(10); }
  }
}

struct LoopTime{

  bool startOrNot = false;
	uint32_t start;
	uint32_t printT;
    uint32_t startTx;
	bool tx_flag;
	
  String fstage;
	
	struct arrays{
    uint8_t maxLen = 9;
		uint32_t data[9] = {0};
		uint8_t index = 0;
		uint8_t len = 0;
		uint32_t sumNum = 0;

		void push(uint32_t in){
			data[index] = in;
			index++;
			if(index > maxLen-1) index = 0;
			if(len < maxLen) len++;
		}

		uint32_t sum(){
			sumNum = 0;
			for(uint8_t indice = 0;indice < len;indice++){
				sumNum += data[indice]/len;
			}
			return sumNum;
		}
	}loraTxTimeOnAir,duration;

  void print(){
    if(millis() > printT){  
      Serial.print("TIME:");
      Serial.println(millis() - start);
      Serial.print("Duration: ");
      Serial.println(duration.sum());
      Serial.print("LoRa Time On Air: ");
      Serial.println(loraTxTimeOnAir.sum());
      Serial.println();
      printT = millis() + 100;
    }
  }

	void begin(){
		start = millis();
    printT = millis();
    startTx = millis();
		tx_flag = false;
	}

  bool tx_flag_get(){
    return tx_flag;
  }

	bool transmitOrNot(){
    if(tx_flag){
      print();
      if(millis() - start >= 1999){
        start = millis();
      }
      if((millis() - start > 50
      // if((millis() - start > loraTxTimeOnAir.sum() 
        && 2000 - (millis() - start) > 500) 
          || duration.len == 0 
          || millis() - start > 2000*2) // duration.sum()
      {
        Serial.print("\t\tWait Time: ");
        Serial.println(millis() - startTx);
        
        if(millis() - start > 50){
          fstage += ",AFTER rx : ";
          fstage += String(millis() - start);
        }
        if(2000 - (millis() - start) > 500 ){
          fstage += ",BEFORE rx : ";
          fstage += String(2000 - (millis() - start));
        }
        if(duration.len == 0 )
          fstage += ",noInput";

        if(millis() - start > 2000*2)
          fstage += ",timeout";

				tx_flag = false;
				return true;
			}
      if(fstage.length() == 0)
        fstage += "TRY\t";
		}
		return false;
	}

	void transmit(){
    startTx = millis();
	    	tx_flag = true;
        fstage = "";
	}

	void receive(uint32_t timeOnAir){
    if(tx_flag) return;
    if(!startOrNot) {
        startOrNot = true;
        start = millis();
        return;
    }
    duration.push(millis() - start);
    loraTxTimeOnAir.push(10 + timeOnAir/1000);

    start = millis();
	}

}rxLoopTime;

/* ================================================================================================== */
// LoRa State
enum class LoRaState
{
  IDLE = 0,
  TRANSMITTING,
  RECEIVING
};

String input;
String tx_data;
String arg = "";
String cmd = "";

int status_lora;
volatile bool rx_flag = false;
volatile bool tx_flag = false;
volatile bool transmit = false;
volatile bool operationDone = false;
volatile LoRaState lora_state = LoRaState::IDLE;

uint32_t printV;
uint32_t serialEndTime;
uint32_t tx_time;
uint32_t lastCount = 0;

bool tx_time_flag = false;

float lora_rssi;

uint32_t serialInTime;
int state;
uint32_t reciveTime;
bool flagAgain = false;
int count = 0;
String oldString = "";

int lossing[500] = {0};
String slossing[500];
int i = 0;
int c = 0;
int t = 0;
int r = 0;
bool sim = false;
uint32_t re = 1750;
uint32_t simT;
uint32_t simulate;
uint32_t loopSimulate;
void serialReadTask();
void rx();


void countLostInt(int value ){
  uint8_t s = 0;
  uint8_t f = 0;

  if (value - c != 1) {
    rxLoopTime.fstage = "FALSE\t" + rxLoopTime.fstage;
  }
  else{
    rxLoopTime.fstage = "SUCCESS\t" + rxLoopTime.fstage;
  }
  lossing[i] = r;
  slossing[i] = rxLoopTime.fstage;
  i++;
  
  Serial.println("value: " + String(value) + ", c: " + String(c));
  c = value;
  Serial.print("Lost: ");
  for(int j = 0;j < i;j++){
    if(slossing[j][0] == 'S'){
      Serial.print(lossing[j]);
      Serial.print(" : ");
      Serial.print(slossing[j]);
      Serial.println();
      s++;
    }
  }

  for(int j = 0;j < i;j++){
    if(slossing[j][0] == 'F'){
      Serial.print(lossing[j]);
      Serial.print(" : ");
      Serial.print(slossing[j]);
      Serial.println();
      f++;
    }
  }

  Serial.println("SUCCES : " + String(s) + "\tFAIL : " + String(f));

}

void simulatetransmitting(){
  if(sim){
    if(millis() > simulate && !rxLoopTime.tx_flag_get()){
      // countLostInt(lastCount);
      tx_data = String(r);
      rxLoopTime.transmit();
      if(r > 2000)
        while (1){ delay(100);}
    }
  }
};

String clean(String s) {
  s.trim();          // remove whitespace/newlines
  while (s.length() > 0 && (s[s.length() - 1] == '\n' || s[s.length() - 1] == '\r')) {
    s = s.substring(0, s.length() - 1);
  }
  return s;
}

void transmitting(){
    tx_flag = true;
    lora_state = LoRaState::TRANSMITTING;
    Serial.print("Transmitting: ");
    Serial.println(tx_data);
    state = lora.startTransmit(tx_data);
    if(state == RADIOLIB_ERR_NONE) {
      Serial.println("[TRANSMITTING...]");
    }
    else {
      Serial.print("Transmit failed, code: ");
      Serial.println(state);
    }
}

void setFlag(void) {
  operationDone = true;
}

void setup()
{
  delay(5000);

  Serial.begin(115200);

  Serial.println("Connected");

  SPI.begin(); // initialize SPI bus

  Serial.println("SPI begin");

  delay(1000);

  Serial.print("Lora setup");

  loraSetup();

  Serial.print("success");

  float freq;
  EEPROM.get<float>(0, freq);
  if(freq > 800){
    lora.setFrequency(freq);
    Serial.print("Set frequency to ");
    Serial.print(freq);
    Serial.println("MHz");
  } 

  tx_time = millis();

  lora.setDio0Action(setFlag,RISING);

  lora.startReceive();
  rxLoopTime.begin();

  simT = millis();
  simulate = millis();
  loopSimulate = millis();
}

void loop(){
    rx();
}

void serialReadTask() {


  if (Serial.available() && !tx_flag)
  {
    tx_data = "";
    
    serialEndTime = millis();

    while(millis() - serialEndTime < 10){
        while (Serial.available()){
            tx_data += static_cast<char>(Serial.read());
            serialEndTime = millis();
        }
    }
    if (tx_data.substring(0, 4) == "freq")
    {
      String freqStr = tx_data.substring(4);
      freqStr.trim();
      const float freq = freqStr.toFloat();
      lora.setFrequency(freq);
      EEPROM.put<float>(0, freq);
      Serial.print("Set frequency to ");
      Serial.print(freq);
      Serial.println("MHz");
    }
    else
    {
      rxLoopTime.transmit();
    }
    Serial.print("Get Serial");
  }
}

void rx()
{
  if(re >= 1){
    re = 0;
    if(!sim && !tx_flag){
      countLostInt(lastCount);
      sim = true;
      simulate = millis() + r;
      r += 20;
    }
    simT = millis();

  }

  simulatetransmitting();

  serialReadTask();

  if(rxLoopTime.transmitOrNot()){ 
    transmitting();
    sim = false;

    Serial.print("delay time: ");
    Serial.println(millis() - simT);
    Serial.print("timeNow");
    Serial.println(millis());
  }

  if(operationDone){
    operationDone = false;
    if (tx_flag)
    {
        tx_flag = false;
        lora_state = LoRaState::RECEIVING;
        lora.startReceive();
        Serial.println("[RECEIVING...]");
    }
    else
    {
      re++;
      rx_flag = false;

      String s;
      state = lora.readData(s);

      delay(1);
      if(state == RADIOLIB_ERR_NONE) {

        rxLoopTime.receive(lora.getTimeOnAir(s.length()));
        
        lastCount = s.toInt();
        // if(rxLoopTime.tx_flag_get()){
        //   countLost(s);
        // }
        s = clean(s);
        
        lora_rssi = lora.getRSSI();
        // s += ',';
        // s += lora_rssi;
        // s += ',';
        // s += lora.getSNR();
        // s += ',';
        // s += lora.getPacketLength();

        Serial.print("RSSI: ");
        Serial.println(lora_rssi);

        Serial.println("[RECEIVED]   ");
        
        Serial.println(s);

        Serial.println(t);

        Serial.println("[RECEIVING...]");
      }
      else {
        Serial.print("Receive failed, code: ");
        Serial.println(state);
      }
        // lora.finishReceive();
        lora_state = LoRaState::RECEIVING;
        lora.startReceive();
        Serial.println("[RECEIVING...]");
    }
  }
}
