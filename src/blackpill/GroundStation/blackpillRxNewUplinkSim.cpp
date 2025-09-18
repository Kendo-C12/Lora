#include <Arduino.h>

#include <RadioLib.h>
#include <SPI.h>
#include <EEPROM.h>

#define LORA_MOSI PA7
#define LORA_MISO PA6
#define LORA_SCLK PA5

SPIClass spi1(LORA_MOSI,LORA_MISO,LORA_SCLK);  // Using hardware SPI (MISO,MOSI,SCLK)

SPISettings lora_spi_settings(8000000, MSBFIRST, SPI_MODE0); // 8 MHz for Mega2560

constexpr struct {
    float center_freq = 915.000000f;  // MHz
    float bandwidth   = 125.f;     // kHz
    uint8_t spreading_factor = 9;  
    uint8_t coding_rate = 8;       
    uint8_t sync_word = 0x12;      
    int8_t power = 22;             
    uint16_t preamble_length = 16;
} lora_params;

#define LORA_NSS   PA4
#define LORA_DIO1  PB8
#define LORA_NRST  PB7
#define LORA_BUSY  PB6

SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY, spi1, lora_spi_settings);

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

	uint32_t start;
	uint32_t printT = millis();
  uint32_t startTx = millis();
	bool tx_flag;
	
	
	struct arrays{
		uint32_t data[3] = {0,0,0};
		uint8_t index = 0;
		uint8_t len = 0;
		uint32_t sumNum = 0;

		void push(uint32_t in){
			data[index] = in;
			index++;
			if(index > 2) index = 0;
			if(len < 3) len++;
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
		tx_flag = false;
	}

  bool tx_flag_get(){
    return tx_flag;
  }

	bool transmitOrNot(){
    // Serial.println(tx_flag);  
    if(tx_flag){
      // print();
      
      if((millis() - start > 400
      // if((millis() - start > loraTxTimeOnAir.sum() 
        && duration.sum() - (millis() - start) > 100 ) 
          || duration.len == 0 
          || millis() - start > 2000)
          // || millis() - start > duration.sum())
      {
        Serial.print("Wait Time: ");
        Serial.println(millis() - startTx);
				tx_flag = false;
				return true;
			}
		}
		return false;
	}

	void transmit(){
    startTx = millis();
		tx_flag = true;
	}

	void receive(uint32_t timeOnAir){
    if(tx_flag) return;
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
volatile LoRaState lora_state = LoRaState::IDLE;

uint32_t lora_tx_end_time;
uint32_t printV;
uint32_t serialEndTime;
uint32_t tx_time;
uint32_t lastCount = 0;

bool tx_time_flag = false;

float lora_rssi;

uint32_t serialInTime;
uint32_t state;
uint32_t reciveTime;
bool flagAgain = false;
int count = 0;
String oldString = "";

int lossing[100] = {0};
int i = 0;
int c = 0;
int t = 0;
int r = 1500;
bool sim = false;
uint32_t simT;
uint32_t simulate = millis();
uint32_t loopSimulate = millis();
void serialReadTask();
void rx();


void countLostInt(int value ){
  if (value - c != 1 && value - c > 0) {
      t++;
      lossing[i] = r;
      i++;
  }
  c = value;
  Serial.print("Lost: ");
  for(int j = 0;j < i;j++){
    Serial.print(lossing[j]);
    Serial.print(',');
  }
}

void simulatetransmitting(){
  if(sim){
    if(milli > simulate && !rxLoopTime.tx_flag_get()){
      countLostInt(lastCount);
      simulate = millis() + r;
      r += 10;
      tx_data = String(r);
      rxLoopTime.transmit();
      if(r > 2000)
        while (1);
      sim = false;
    }
  }
};

void countLost(String s){
  s.trim();          // remove whitespace/newlines
  int value = s.toInt();

  if (value - c != 1 && value - c > 0) {
      t++;
      lossing[i] = r;
      i++;
  }
  c = value;
  Serial.print("Lost: ");
  for(int j = 0;j < i;j++){
    Serial.print(lossing[j]);
    Serial.print(',');
  }
}

String clean(String s) {
  s.trim();          // remove whitespace/newlines
  while (s.length() > 0 && (s[s.length() - 1] == '\n' || s[s.length() - 1] == '\r')) {
    s = s.substring(0, s.length() - 1);
  }
  return s;
}

void transmitting(){
    lora_state = LoRaState::TRANSMITTING;
    lora_tx_end_time = millis() + 10 + (lora.getTimeOnAir(tx_data.length())) / 1000;
    // Serial.println(millis() - lora_tx_end_time);
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
  rx_flag = true;
}

void setup()
{
  delay(5000);

  Serial.begin(115200);

  Serial.println("Connected");

  spi1.begin(); // initialize SPI bus

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

  lora_tx_end_time = millis();
  tx_time = millis();

  lora.setPacketReceivedAction(setFlag);

  lora.startReceive();
  rxLoopTime.begin();

  simT = millis();
}

void loop(){
    rx();
}

void serialReadTask() {


  if (Serial.available() && millis() > lora_tx_end_time)
  {
    tx_data = "";
    
    serialEndTime = millis();

    while(millis() - serialEndTime < 10){
        while (Serial.available()){
            tx_data += static_cast<char>(Serial.read());
            serialEndTime = millis();
        }
    }
    if (tx_data.substring(0, 9) == "cmd freq ")
    {
      String freqStr = tx_data.substring(9);
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
  if(simT > millis() + 2000){
    if(!sim){
      sim = true;
    }
    simT = millis();
  }

  simulatetransmitting();

  serialReadTask();

  if(rxLoopTime.transmitOrNot()){ 
    transmitting();
  }

  if (millis() > lora_tx_end_time &&
      lora_state != LoRaState::RECEIVING)
  {
      lora.standby();
      lora_state = LoRaState::RECEIVING;
      lora.startReceive();
      Serial.println("[RECEIVING...]");
  }

  if (rx_flag && lora.getPacketLength() > 0 && lora_state == LoRaState::RECEIVING)
  {
    rx_flag = false;

    String s;
    state = lora.readData(s);

    rxLoopTime.receive(lora.getTimeOnAir(s.length()));

    if(state == RADIOLIB_ERR_NONE) {
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
  
    lora.standby();
    lora_state = LoRaState::IDLE;
  }
}
