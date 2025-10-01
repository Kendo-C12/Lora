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
    float center_freq = 920.400000f;  // MHz
    float bandwidth   = 125.f;     // kHz
    uint8_t spreading_factor = 9;  
    uint8_t coding_rate = 8;       
    uint8_t sync_word = 0x12;      
    int8_t power = 20;             
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
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  // we sent or received  packet, set the flag
  operationDone = true;
}

void setup() {
  Serial.begin(115200);

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  
  loraSetup();
  // set the function that will be called
  // when new packet is received
  lora.setDio0Action(setFlag, RISING);

  // #if defined(INITIATING_NODE)
    // send the first packet on this node
    Serial.print(F("[SX1278] Sending first packet ... "));
    transmissionState = lora.startTransmit("Hello World!");
    transmitFlag = true;
  // #else
  //   // start listening for LoRa packets on this node
  //   Serial.print(F("[SX1278] Starting to listen ... "));
  //   int state = lora.startReceive();
  //   if (state == RADIOLIB_ERR_NONE) {
  //     Serial.println(F("success!"));
  //   } else {
  //     Serial.print(F("failed, code "));
  //     Serial.println(state);
  //     while (true) { delay(10); }
  //   }
  // #endif
}

void loop() {
  // check if the previous operation finished
  if(operationDone) {
    // reset flag
    operationDone = false;

    if(transmitFlag) {
      // the previous operation was transmission, listen for response
      // print the result
      if (transmissionState == RADIOLIB_ERR_NONE) {
        // packet was successfully sent
        Serial.println(F("transmission finished!"));

      } else {
        Serial.print(F("failed, code "));
        Serial.println(transmissionState);

      }

      // listen for response
      lora.startReceive();
      transmitFlag = false;

    } else {
      // the previous operation was reception
      // print data and send another packet
      String str;
      int state = lora.readData(str);

      if (state == RADIOLIB_ERR_NONE) {
        // packet was successfully received
        Serial.println(F("[SX1278] Received packet!"));

        // print data of the packet
        Serial.print(F("[SX1278] Data:\t\t"));
        Serial.println(str);

        // print RSSI (Received Signal Strength Indicator)
        Serial.print(F("[SX1278] RSSI:\t\t"));
        Serial.print(lora.getRSSI());
        Serial.println(F(" dBm"));

        // print SNR (Signal-to-Noise Ratio)
        Serial.print(F("[SX1278] SNR:\t\t"));
        Serial.print(lora.getSNR());
        Serial.println(F(" dB"));

      }

      // wait a second before transmitting again
      delay(1000);

      // send another one
      Serial.print(F("[SX1278] Sending another packet ... "));
      transmissionState = lora.startTransmit("Hello World!");
      transmitFlag = true;
    }
  }
}
