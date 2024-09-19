#include <SoftwareSerial.h>
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 8
#define BP_START_PIN (2)         // start button of the blood pressure monitor device. replaced a transistor.   
#define VALVE_PIN (3)            // checks if the measurement is done.             
#define MEASURE_BEGIN_PIN (4)    // indicates that a measurement should start. this can be connected to switch or another MCU or raspberry pi.

SoftwareSerial mySerial(11, 10);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature Sensors(&oneWire);

volatile byte i2c_data_rx;       // indicates there are available data from the i2c bus. 
volatile uint16_t count;         // indicates the total number of data collected.
volatile uint8_t sys, dia ;   // stored the measure values: systolic, diastolic and heart rate.
float temperature = 0;

//VARIABLE BPM
const byte ECG_PIN  = A0;
int Data_Hz = 60;
long INT_REGISTER = (16000000L / (Data_Hz*1024L)) - 1;
int AverageChecker = 1500L;

const byte BPM_LOG_MAX = 5;

int data = 0;
int data_last = 0;
unsigned long AverageChecker_last = 0;

long RR_peak_time = 0;
boolean R_watchout = false;

int BPM = 0;
int BPM_LOG[BPM_LOG_MAX];
int BPM_LOG_AVR = 0;

const byte sudut_data_max = 5;
int sudut_data[sudut_data_max];
unsigned long looptime = 0;
int RRTime = 0;
float angle = 0;
float angle_last = 0;
float sudut_baseline = 0;

void readsim(){
  delay(1000);
  while(true){
    if (mySerial.available()) {
      Serial.print("ada");
//      Serial.write(mySerial.read());
      mySerial.read();
    }
    else{
      Serial.print("GAADA");
      break;
    }
  }
}

void setup_sim(){
  Serial.println("Please wait.....");
  delay(2000);
  mySerial.println("AT");
  readsim();
  delay(1000);
  mySerial.println("AT+CFUN?");
  readsim();
  delay(1000);
  mySerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  readsim();
  delay(1000);
  mySerial.println("AT+CSTT=\"3gprs\",\"3gprs\",\"3gprs\"");
  readsim();
  delay(1000);
  mySerial.println("AT+SAPBR=3,1,\"APN\",\"internet.movistar.mx\"");
  readsim();
  delay(1000);
  mySerial.println("AT+SAPBR=3,1,\"USER\",\"movistar\"");
  readsim();
  delay(1000);
  mySerial.println("AT+SAPBR=3,1,\"PWD\",\"movistar\"");
  readsim();
  delay(1000);
  mySerial.println("AT+SAPBR=0,1");
  readsim();
  delay(1000);
  mySerial.println("AT+SAPBR=1,1");
  readsim();
  delay(1000);
  mySerial.println("AT+SAPBR=2,1");
  readsim();
  delay(1000);
  mySerial.println("AT+HTTPINIT");
  readsim();
  delay(1000);
  mySerial.println("AT+HTTPPARA=\"CID\",1");
  readsim();
  delay(1000);
}

void senddata(){
  mySerial.print("AT+HTTPPARA=\"URL\",\"http://medical-monitoring.mobilefitech.com/monitoring/add_data/1_");
  mySerial.print(temperature);
  mySerial.print("_");
  mySerial.print(sys);
  mySerial.print("_");
  mySerial.print(dia);
  mySerial.print("_");
  mySerial.print(BPM);
  mySerial.print("\"\r");
  readsim();
  delay(1000);
  mySerial.println("AT+HTTPACTION=0");
  readsim();
}



int a[30];
void Found_PR(unsigned long thistime){
    // ==========================
    // Catat interval dengan R sebelumnya
    // untuk memprediksi BPM
    // ===================================
    RRTime = thistime-RR_peak_time;  

    // estimasi BPM berdasarkan data saat ini
    int BPM_est = round((float)60000/RRTime);

    float bpm_sum = 0;
    for(byte rid=0; rid<BPM_LOG_MAX-1; rid++){
        BPM_LOG[rid] = BPM_LOG[rid+1];
        bpm_sum += BPM_LOG[rid+1];
    }
    BPM_LOG[BPM_LOG_MAX-1] = BPM_est;
    bpm_sum += BPM_est;
    BPM = bpm_sum/BPM_LOG_MAX;
                        
    RR_peak_time = thistime;
}

void ReadSensor(){    
  // catat waktu saat ini
  unsigned long thistime = millis();

  data = analogRead(ECG_PIN);
  int sudut_now = (atan(data-data_last) * 180/3.14159265);
  
  // ===================================
  // deteksi puncak R
  // sudut naik dari data sebelumnya harus >85 derajat dan sudut turun >85 derajat
  // tapi dari beberapa data juga harus naik. Waktu antara R-R harus lebih dari 300ms 
  // untuk menghindari false detection ketika puncak T lebih tingi dari R (T-R < 250ms)
  // ===================================
  angle_last = angle;
  angle = sudut_now;
  if ( angle_last > 85 && angle < -85 && sudut_baseline > 85 && (thistime-RR_peak_time) > 300 ){
      R_watchout = true;
  }
  
  if ( R_watchout == true ) {          
      Found_PR(thistime);
      R_watchout = false;
  }
  sudut_baseline = atan( (data-sudut_data[0])/sudut_data_max ) * 180/3.14159265;
  data_last = data;
}

void receiveEvent(int iData){ // Interrupt service routine.
  if ( iData > 0 ){
    while ( iData-- ){
      i2c_data_rx = Wire.read();
      count++;
      if (count == 28){
        sys = i2c_data_rx;
      }
      if (count == 29){
        dia = i2c_data_rx;
      }
//      a[count]=i2c_data_rx;
    }
//    count = 0;
  }
}

ISR(TIMER1_COMPA_vect){
    ReadSensor();
}

void setup() {
  // setting interrupt 60Hz ECG
  cli(); //stop interrupts
  
  // Mengaktifkan dan mengonfigurasi timer
  // TCCR1A = 0;  // set TCCR1A register = 0
  // TCCR1B = 0;  // set TCCR1B register = 0
  // TCNT1  = 0;  // inisialisasi counter = 0
  // OCR1A  = INT_REGISTER;  // untuk 60Hz
  // TCCR1B |= (1 << WGM12);  // aktifkan CTC mode
  // TCCR1B |= (1 << CS12) | (1 << CS10);  // Set CS12 & CS10 bits = 1024 prescaler
  // TIMSK1 |= (1 << OCIE1A);  // aktifkan timer compare interrupt
  
  sei(); //allow interrupts
  
  pinMode(BP_START_PIN, OUTPUT);
  pinMode(VALVE_PIN, INPUT);
  pinMode(MEASURE_BEGIN_PIN, INPUT_PULLUP);
  pinMode(ECG_PIN, INPUT);
  Sensors.begin();
  Serial.begin(9600);
  mySerial.begin(9600);
  setup_sim();

  Wire.begin(0x50);                           // the address of the EEPROM is 0x50. The Arduino should be the same.            
  Wire.onReceive(receiveEvent);               // this is the interrupt initialization for the i2c data.
}



void loop() {
  looptime = millis();
  
  // menghitung rata2 setiap (AverageChecker)
  if(looptime - AverageChecker_last > AverageChecker){
    
    AverageChecker_last = looptime;
    Sensors.requestTemperatures();
    temperature = Sensors.getTempCByIndex(0);
    if(temperature <= 0 ) temperature = 0;
  
    Serial.println("--------------------------");
    Serial.print("Suhu Tubuh    : ");
    Serial.print(temperature);
    Serial.print("°");
    Serial.println("C ");
    Serial.print("Tekanan Darah : ");
    Serial.print(sys);
    Serial.print("/");
    Serial.print(dia);
    Serial.println(" mmHg");
    Serial.print("BPM :");
    Serial.println(BPM);
    Serial.println("--------------------------");
    senddata();
  }
  
 delay(1000);
}