#include <Arduino.h>
#include <WiFi.h>
#include <driver/adc.h>

#define AUDIO_BUFFER_MAX 2048

#define TRANSMIT_LED    2
#define STATUS_LED      4
#define RECORD_LED      14
#define TRANSMIT_BUTTON 18
#define RECORD_BUTTON   12

#define RESET   22  // VS1053 reset pin (output)
#define CS      5   // VS1053 chip select pin (output)
#define DCS     15   // VS1053 Data/command select pin (output)
#define CARDCS  21  // Card chip select pin
#define DREQ    16  // VS1053 Data request, ideally an Interrupt pin

#define RECBUFFSIZE 128  // 64 or 128 bytes.

uint8_t isRecording = false;
uint8_t recording_buffer[RECBUFFSIZE];
int STTransmit = 0;
int STRecord = 0;
uint8_t audioBuffer[AUDIO_BUFFER_MAX];
uint8_t transmitBuffer[AUDIO_BUFFER_MAX];
uint32_t bufferPointer = 0;

const char* ssid     = "ENTER YOUR SSID";
const char* password = "ENTER YOUR NETWORK PASSWORD";
const char* host     = "ENTER YOUR COMPUTER'S IP ADDRESS";

bool transmitNow = false;

WiFiClient client;

hw_timer_t * timer = NULL; // our timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(RESET, CS, DCS, DREQ, CARDCS);

File recording;  // the file we will save our recording to

uint16_t saveRecordedData(boolean isrecord) {
  uint16_t written = 0;
  
    // read how many words are waiting for us
  uint16_t wordswaiting = musicPlayer.recordedWordsWaiting();
  
  // try to process 256 words (512 bytes) at a time, for best speed
  while (wordswaiting > 256) {
    //Serial.print("Waiting: "); Serial.println(wordswaiting);
    // for example 128 bytes x 4 loops = 512 bytes
    for (int x=0; x < 512/RECBUFFSIZE; x++) {
      // fill the buffer!
      for (uint16_t addr=0; addr < RECBUFFSIZE; addr+=2) {
        uint16_t t = musicPlayer.recordedReadWord();
        //Serial.println(t, HEX);
        recording_buffer[addr] = t >> 8; 
        recording_buffer[addr+1] = t;
      }
      if (! recording.write(recording_buffer, RECBUFFSIZE)) {
            Serial.print("Couldn't write "); Serial.println(RECBUFFSIZE); 
            while (1);
      }
    }
    // flush 512 bytes at a time
    recording.flush();
    written += 256;
    wordswaiting -= 256;
  }
  
  wordswaiting = musicPlayer.recordedWordsWaiting();
  if (!isrecord) {
    Serial.print(wordswaiting); Serial.println(" remaining");
    // wrapping up the recording!
    uint16_t addr = 0;
    for (int x=0; x < wordswaiting-1; x++) {
      // fill the buffer!
      uint16_t t = musicPlayer.recordedReadWord();
      recording_buffer[addr] = t >> 8; 
      recording_buffer[addr+1] = t;
      if (addr > RECBUFFSIZE) {
          if (! recording.write(recording_buffer, RECBUFFSIZE)) {
                Serial.println("Couldn't write!");
                while (1);
          }
          recording.flush();
          addr = 0;
      }
    }
    if (addr != 0) {
      if (!recording.write(recording_buffer, addr)) {
        Serial.println("Couldn't write!"); while (1);
      }
      written += addr;
    }
    musicPlayer.sciRead(VS1053_SCI_AICTRL3);
    if (! (musicPlayer.sciRead(VS1053_SCI_AICTRL3) & (1 << 2))) {
       recording.write(musicPlayer.recordedReadWord() & 0xFF);
       written++;
    }
    recording.flush();
  }

  return written;
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux); // says that we want to run critical code and don't want to be interrupted
  uint16_t adcVal = adc1_get_raw(ADC1_CHANNEL_7); // reads the ADC
  uint8_t value = map(adcVal, 0 , 4096, 0, 255);  // converts the value to 0..255 (8bit)
  audioBuffer[bufferPointer] = value; // stores the value
  bufferPointer++;
 
  if (bufferPointer == AUDIO_BUFFER_MAX) { // when the buffer is full
    bufferPointer = 0;
    memcpy(transmitBuffer, audioBuffer, AUDIO_BUFFER_MAX); // copy buffer into a second buffer
    transmitNow = true; // sets the value true so we know that we can transmit now
  }
  portEXIT_CRITICAL_ISR(&timerMux); // says that we have run our critical code
}



void setup() {
  Serial.begin(115200);
  pinMode(TRANSMIT_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(RECORD_LED, OUTPUT);
  pinMode(RECORD_BUTTON, INPUT_PULLUP);
  pinMode(TRANSMIT_BUTTON, INPUT_PULLUP);

  digitalWrite(STATUS_LED, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("MY IP address: ");
  Serial.println(WiFi.localIP());
  
  adc1_config_width(ADC_WIDTH_12Bit); // configure the analogue to digital converter
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_0db); // connects the ADC 1 with channel 0 (GPIO 36);

  const int port = 4444;
  while (!client.connect(host, port)) {
    Serial.println("connection failed");
    delay(1000);
  }

  Serial.println("connected to server");

  // initialise the music player
  if (!musicPlayer.begin()) {
    Serial.println("VS1053 not found");
    while (1);  // don't do anything more
  }

  if (!SD.begin(CARDCS)) {
    Serial.println("SD failed, or not present");
    while (1);  // don't do anything more
  }
  Serial.println("SD OK!");
  
  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(10,10);
  
  // when the button is pressed, record!
  pinMode(RECORD_BUTTON, INPUT);
  digitalWrite(RECORD_BUTTON, HIGH);
  
  // load plugin from SD card! We'll use mono 44.1KHz, high quality
  if (! musicPlayer.prepareRecordOgg("v44k1q05.img")) {
     Serial.println("Couldn't load plugin!");
     while (1);    
  }
  timer = timerBegin(0, 80, true); // 80 Prescaler
  timerAttachInterrupt(timer, &onTimer, true); // binds the handling function to our timer 
  timerAlarmWrite(timer, 125, true);
  timerAlarmEnable(timer);
  digitalWrite(STATUS_LED, LOW);
}

void loop() {
  if(digitalRead(TRANSMIT_BUTTON) == 0) {
    delay(500);
    STTransmit = xTaskGetTickCount();
    Transmit();
  }

  if(digitalRead(RECORD_BUTTON) == 0) {
    delay(500);
    STRecord = xTaskGetTickCount();
    Record();
  }
}

void Transmit(){
  while(xTaskGetTickCount() - STTransmit < 30000) {
    if (transmitNow && digitalRead(TRANSMIT_BUTTON) == 1) { // checks if the buffer is full
      digitalWrite(TRANSMIT_LED, HIGH);
      transmitNow = false;
      client.write((const uint8_t *)audioBuffer, sizeof(audioBuffer)); // sending the buffer to our server
      digitalWrite(TRANSMIT_LED, LOW);
    } else if(digitalRead(TRANSMIT_BUTTON) == 0){
      delay(300);
      break;
    }
  }
}

void Record() {

  digitalWrite(RECORD_LED, HIGH);
  while(xTaskGetTickCount() - STRecord < 30000) {
    if (!isRecording) {
      Serial.println("Begin recording");
      isRecording = true;
      
      // Check if the file exists already
      char filename[15];
      strcpy(filename, "RECORD00.OGG");
      for (uint8_t i = 0; i < 100; i++) {
        filename[6] = '0' + i/10;
        filename[7] = '0' + i%10;
        // create if does not exist, do not open existing, write, sync after write
        if (! SD.exists(filename)) {
          break;
        }
      }
      Serial.print("Recording to "); Serial.println(filename);
      recording = SD.open(filename, FILE_WRITE);
      if (!recording) {
         Serial.println("Couldn't open file to record!");
         while (1);
      }
      musicPlayer.startRecordOgg(true); // use microphone (for linein, pass in 'false')
    }
    if (isRecording)
      saveRecordedData(isRecording);
    if (isRecording && (xTaskGetTickCount() - STRecord > 30000||digitalRead(RECORD_BUTTON) == 0)) {
      Serial.println("End recording");
      musicPlayer.stopRecordOgg();
      isRecording = false;
      // flush all the data!
      saveRecordedData(isRecording);
      // close it up
      recording.close();
    }
  }
  digitalWrite(RECORD_LED, LOW);
}