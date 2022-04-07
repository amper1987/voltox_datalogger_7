#include <Arduino.h>
#include "stdio.h"
#include "string.h"
#include "esp_adc_cal.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define ADC0 36
#define ADC1 39
#define ADC2 34
#define ADC3 35
#define ADC4 32
#define status_pin_gen 33
#define status_pin_load 25
#define RXD2 16
#define TXD2 17
#define UART_BUFFER 1024
#define RX_BUF_SIZE 1024
#define buf_size 400
#define sensor_buffer 400
#define current_buffer (4)
#define FILTER_LEN  150
#define SIZE 100


uint32_t period = 100;

float resistor = 56.0f;
int AN_Pot1_i = 0;
int transfer_buffer_index = 0;  
int  x;
float one = 1.00f;
float zero = 0.00f;
int length = 0;
int rssi_int;
float rssi_float = 0.0;
float divider = 0.075f;
float thousand = 1000.0f;
float subtraction = 1.68f;

char fc_buffer_ccid [SIZE]; // first character of the buffer buffer
char fc_buffer_rssi [SIZE]; // first character of the buffer buffer 
char fc_buffer_ccid_c [SIZE]; // first character of the buffer buffer
char fc_buffer_rssi_c [SIZE]; // first character of the buffer buffer 
char ccid_buf [21];
char rssi_buf [5];
char ccid[] = {"AT#CCID\r\n"};
char rssi[] = {"AT+CSQ\r\n"};
char status[4] = {0x00, 0x00, 0x00, 0x00};
char measurement_count[] = {'1'};
char db_index[] = {'C'};
char float_buffer[10];
size_t len;

char  *my_str ="POST /telemetry.php HTTP/1.1\r\n"
               "HOST: weile-enterprises.com\r\n"    
               "Connection: Keep-Alive\r\n"         
               "User-Agent: WLS\r\n"               
               "Content-Length: 58\r\n\r\n"; // end of frame        
               
uint32_t sensor(uint32_t time, int ADC_number);
uint32_t convert(int ADC_Raw);
void uart_init(void);
void capture_data(void);
//uint32_t average_dc_current(int dc_voltage);
uint32_t average_mid_current(int ADC_number);
void modem_init (void);
uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
void print_string();
//void parse_data_ccid (char *str);
void parse_data_rssi (char *str);
void measurement_header (void);
void hhtp_header (void);
//void modem_init (char *str)
void toHex( float fv, char * buf );
void float_toBytes (float val, char* bytes_array);
float round_to_dp( float in_value, int decimal_place );
//int precision (ac_current_0);
unsigned char transfer_buffer[sensor_buffer];
unsigned char http_buffer[500];
uint8_t data_buf [buf_size];
char buffer0[50];
char buffer1[50];
char buffer2[50];
char buffer3[50];
char buffer4[50];
char buffer5[50];
char buffer6[50];
char buffer7[50];
char buffer8[50];
char buffer9[50];
// sensor 1
int max_sample_0;
float voltage_0 = 0.0f;
float ac_voltage_0 = 0.0f;
float ac_current_0 = 0.0f;
// sensor 2
int max_sample_1;
float voltage_1 = 0.0f;
float ac_voltage_1 = 0.0f;
float ac_current_1 = 0.0f;
// sensor 3
int max_sample_2;
float voltage_2 = 0.0f;
float ac_voltage_2 = 0.0f;
float ac_current_2 = 0.0f;
// sensor 4
int max_sample_3;
float voltage_3 = 0.0f;
float ac_voltage_3 = 0.0f;
float ac_current_3 = 0.0f;
//sensor 5
int dc_voltage = 0;
int dc_voltage_average = 0;
float dc_voltage_real = 0.0f;
float dc_current_real = 0.0f;
int dc_voltage_int = 0;

void setup() {
   Serial.begin(9600);
   Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
   modem_init();
  }
void loop()
{
//main initialization is complete,
//open a socket and send data to the server
 Serial2.print("AT#SGACT=1,1\r\n"); //activate the PDP context
 delay(10000);
 Serial2.print("AT#SD=1,0,80,"); //socket dial command
 Serial2.print('"'); 
 Serial2.print("www.weile-enterprises.com"); //using test server
 Serial2.print('"'); 
 Serial2.print(",0,0,0\r\n"); //000
 delay(10000);
 hhtp_header();
 measurement_header();
 capture_data();
 //Serial.write(transfer_buffer, 180);
 Serial2.write(transfer_buffer, 180);
 memset(transfer_buffer,0,sizeof(transfer_buffer));
 transfer_buffer_index = 0;
 delay(10000);
 Serial2.print("AT#SGACT=1,0\r\n"); //factory reset to TELIT modem
} 
// capture data from 4xAC_current, 1xDC_current and 2xAC_voltage sensors 
void capture_data(void)
{
// ac_current_sensor_1 
 max_sample_0 = sensor(period, ADC0);
 voltage_0 = convert(max_sample_0);
 ac_voltage_0 = (voltage_0/thousand) - subtraction;
 ac_current_0 = ((ac_voltage_0/resistor)*100)/divider;
 // ac_current_sensor_2 
 max_sample_1 = sensor(period, ADC1);
 voltage_1 = convert(max_sample_1);
 ac_voltage_1 = (voltage_1/thousand) - subtraction;
 ac_current_1 = ((ac_voltage_1/resistor)*100)/divider;
// ac_current_sensor_3 
 max_sample_2 = sensor(period, ADC2);
 voltage_2 = convert(max_sample_2);
 ac_voltage_2 = (voltage_2/thousand) - subtraction;
 ac_current_2 = ((ac_voltage_2/resistor)*100)/divider;
// ac_current_sensor_4
 max_sample_3 = sensor(period, ADC3);
 voltage_3 = convert(max_sample_3);
 ac_voltage_3 = (voltage_3/thousand) - subtraction;
 ac_current_3 = ((ac_voltage_3/resistor)*100)/divider;
 //dc battery current readings
 dc_voltage_average = average_mid_current(ADC4);
 dc_voltage_int = dc_voltage_average - 3000;
 dc_voltage_real = ((dc_voltage_int*3.3)/4095);
 dc_current_real = ((dc_voltage_real)*150)/0.625;
// UART transmisiion of 4 ac current sensors
//sensor 1
//sprintf(buffer0,"%f", ac_current_0);
//dtostrf(ac_current_0, 5, 2, buffer0);
  //ac_current_0 = round_to_dp(ac_current_0, 2);
  
  float_toBytes (ac_current_0, &buffer0[0]);
   for( x = 0; x < 4; x++)
  {
    transfer_buffer[transfer_buffer_index++] =  buffer0[x];
  }

// sensor 2
//sprintf(buffer1,"%f", ac_current_1);
//dtostrf(ac_current_1, 5, 2, buffer1);
  //ac_current_1 = round_to_dp(ac_current_1, 2);
  float_toBytes (ac_current_1, &buffer1[0]);
  for( x = 0; x < 4; x++)
  {
    transfer_buffer[transfer_buffer_index++] =  buffer1[x];
  }
  
//sensor 3
//sprintf(buffer2,"%f", ac_current_2);
//dtostrf(ac_current_2, 5, 2, buffer2);
  float_toBytes (ac_current_2, &buffer2[0]);
  for( x = 0; x < 4; x++)
  {
    transfer_buffer[transfer_buffer_index++] =  buffer2[x];
  }
  
// sensor 4
//sprintf(buffer3,"%f", ac_current_3);
//dtostrf(ac_current_3, 5, 2, buffer3);
  float_toBytes (ac_current_3, &buffer3[0]);
  for( x = 0; x < 4; x++)
  {
    transfer_buffer[transfer_buffer_index++] =  buffer3[x];
  }
  
//sensor 5
//sprintf(buffer8,"%f",dc_current_real);
//dtostrf(dc_current_real, 5, 2, buffer8);

  float_toBytes (dc_current_real, &buffer8[0]);
  for( x = 0; x < 4; x++)
  {
    transfer_buffer[transfer_buffer_index++] =  buffer8[x];
  }
//ac_voltage_sense_gen. read the pin mode.
 int sensorValue_0 = analogRead(status_pin_gen);
 if (sensorValue_0 < 3000)
 {
    //dtostrf(zero, 5, 2, buffer4);
    //sprintf (buffer4,"%.2f", one);
    float_toBytes (one, &buffer4[0]);
    for (x=0; x < sizeof(buffer4); x++)
   {
     transfer_buffer[transfer_buffer_index++] = buffer4[x];
   }
  }
 else
 {
   //sprintf (buffer5,"%.2f", zero);
   //dtostrf(one, 5, 2, buffer5);
   float_toBytes (zero, &buffer5[0]);
   for (x=0; x < sizeof(buffer5); x++)
   {
    transfer_buffer[transfer_buffer_index++] = buffer5[x];
   }
 }
// ac_voltage_sense_load. read the pin mode. 
 int sensorValue_1 = analogRead(status_pin_load);
 if (sensorValue_1 < 3000)
 {
    //dtostrf(zero, 5, 2, buffer6);
    //sprintf (buffer6,"%.2f", one);
    float_toBytes (one, &buffer6[0]);
    for (x=0; x < sizeof(buffer6); x++)
   {
     transfer_buffer[transfer_buffer_index++] = buffer6[x];
   }
 }
 else
 {
    //sprintf (buffer7,"%.2f", zero);
    //dtostrf(one, 5, 2, buffer7);
    float_toBytes (zero, &buffer7[0]);
    for (x=0; x < sizeof(buffer7); x++)
   {
    transfer_buffer[transfer_buffer_index++] = buffer7[x];
   }
  }
}
//Modem initialization
void modem_init (void)
{
char buffer_ccid[SIZE];
char *ptr = buffer_ccid;
char buffer_rssi[SIZE];
char *str = buffer_rssi;
char *pt = ccid_buf; 
Serial2.print("AT&F\r\n"); //factory reset to TELIT modem
delay(2000);
Serial2.print("ATE0\r\n"); //switch echo off
delay(500);
Serial2.print("AT#QSS?\r\n"); //check SIM status
delay(2000);
Serial2.print("AT#CCID\r\n"); //read the CCID
delay(2000);
for (int i = 0; i<sizeof(buffer_ccid); i++)
{
    buffer_ccid[i]= Serial2.read();
  if ((ptr = strchr (buffer_ccid, '8'))) //check whether ' ' is found  
  {
      //size_t len = strlen (ptr); //advance pointer by 1, get length 
      memcpy (ccid_buf, ptr, 20);// copy remaining part of string
    for (int i = 0; i<sizeof(ccid_buf); i++)
    {  
      if ((pt = strchr (ccid_buf, 'G')))
      {
       ESP.restart();
      }
    }
  }  
}    
   if (strlen(ccid_buf) < 20)
  {
    ESP.restart();
  }
Serial2.print("AT+CSQ\r\n"); //check signal quality
delay(2000);
  for (int i = 0; i<sizeof(buffer_rssi); i++)
  {
    buffer_rssi[i]= Serial2.read();
    if ((str = strchr (buffer_rssi, 'Q'))) // check whether ' ' is found 
    { 
    size_t len = strlen (str+=3); //advance pointer by 1, get length 
    if (len > SIZE - 1) //check if length exceeds available 
    {     
    fputs ("error: string exceeds allowable length.\n", stderr);
    }
      memcpy (fc_buffer_rssi, str, len); //copy remaining part of string
    } 
  } 
memcpy (rssi_buf, fc_buffer_rssi, 2); //copies first 2 bytes to the buf
rssi_float =  atof(rssi_buf);
float_toBytes (rssi_float, &float_buffer[0]);
Serial2.print("AT+CREG?\r\n"); //check registration switch
delay(500);
Serial2.print("AT#SGACT=1,0\r\n"); //deactivate the PDP context
delay(10000);
Serial2.print("AT+CGDCONT=1,IP,APN.ZEROGRAVITYWIRELESS.COM\r\n"); //AT+CGDCONT=1
Serial2.print("APN.ZEROGRAVITYWIRELESS.COM\r\n"); //ZEROGRAVITY
delay(500);
Serial2.print("AT#GAUTH=0\r\n"); //PPP authentication not used
delay(500);
Serial2.print("AT#SCFG=1,1,500,90,600,50\r\n"); //timeout set to 5 seconds
delay(5000);
}
//hhtp_header
void hhtp_header (void)
{
  
  for (x=0; x < 122; x++)
  {
  transfer_buffer[transfer_buffer_index++] = my_str[x];
  }
}
//measurement_header
void measurement_header (void)
{
  for (x=0; x < 1; x++)
  {
  transfer_buffer[transfer_buffer_index++] = db_index[x];
  }
  for (x=0; x < 20; x++)
  {
  transfer_buffer[transfer_buffer_index++] = ccid_buf[x];
  }
  for (x=0; x < 4; x++)
  {
  transfer_buffer[transfer_buffer_index++] = float_buffer[x];
  }
  
  for (x=0; x < 4; x++)
   {
  transfer_buffer[transfer_buffer_index++] = status[x];
   }
  for (x=0; x < 1; x++)
  {
  transfer_buffer[transfer_buffer_index++] = measurement_count[x];
  }
}
//float to byte conversion using union type
void float_toBytes (float val, char* bytes_array)
{
typedef union cracked_float_t
{
  float float_variable;
  char temp_array [4];
};
cracked_float_t floatValue;

floatValue.float_variable = val;
bytes_array[0] = floatValue.temp_array[3];
bytes_array[1] = floatValue.temp_array[2];  
bytes_array[2] = floatValue.temp_array[1];;  
bytes_array[3] = floatValue.temp_array[0]; 
}
//average the dc current values of the battery 
uint32_t average_mid_current(int ADC_number)
{
int i = 0;
uint32_t Sum = 0;
int dc_voltage = 0;
for (i = 0; i< FILTER_LEN; i++)
{
dc_voltage = analogRead(ADC_number);
AN_Pot1_Buffer[i] = dc_voltage;
Sum += AN_Pot1_Buffer[i];
}
return (Sum/FILTER_LEN);
}
// AC current sensors max value identifier
uint32_t sensor(uint32_t time, int ADC_number)
{
int max_value = 0;
int AN_Raw = 0;
uint32_t currentTime = millis(); // start counting
uint32_t finishTime = currentTime + time; // based on the period 
while (currentTime < finishTime)
{
AN_Raw = analogRead(ADC_number);
if (max_value < AN_Raw){
max_value = AN_Raw;}
currentTime = millis(); 
}
return max_value;
}
//convert the RAW value into the real values
uint32_t convert(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}