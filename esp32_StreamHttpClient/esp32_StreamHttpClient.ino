
#include <Arduino.h>

#include <WiFi.h>
#include <WiFiMulti.h>

#include <HTTPClient.h>

#define USE_SERIAL Serial

#define RXD2 16
#define TXD2 17

#define ETX_OTA_SOF  0xAA    // Start of Frame
#define ETX_OTA_EOF  0xBB    // End of Frame

typedef enum
{
  ETX_OTA_PACKET_TYPE_CMD       = 0,    // Command
  ETX_OTA_PACKET_TYPE_DATA      = 1,    // Data
  ETX_OTA_PACKET_TYPE_HEADER    = 2,    // Header
  ETX_OTA_PACKET_TYPE_RESPONSE  = 3,    // Response
}ETX_OTA_PACKET_TYPE_;

typedef enum
{
  ETX_OTA_CMD_START = 0,    // OTA Start command
  ETX_OTA_CMD_END   = 1,    // OTA End command
  ETX_OTA_CMD_ABORT = 2,    // OTA Abort command
}ETX_OTA_CMD_;

typedef struct
{
  uint32_t package_size;
  uint32_t package_crc;
  uint32_t reserved1;
  uint32_t reserved2;
}__attribute__((packed)) meta_info;

typedef struct
{
  uint8_t   sof;
  uint8_t   packet_type;
  uint16_t  data_len;
  uint8_t   cmd;
  uint32_t  crc;
  uint8_t   eof;
}__attribute__((packed)) ETX_OTA_COMMAND_;

typedef struct
{
  uint8_t     sof;
  uint8_t     packet_type;
  uint16_t    data_len;
  meta_info   meta_data;
  uint32_t    crc;
  uint8_t     eof;
}__attribute__((packed)) ETX_OTA_HEADER_;

typedef struct
{
  uint8_t     sof;
  uint8_t     packet_type;
  uint16_t    data_len;
  uint8_t     *dat;
}__attribute__((packed)) ETX_OTA_DATA_;

typedef struct
{
  uint8_t   sof;
  uint8_t   packet_type;
  uint16_t  data_len;
  uint8_t   status;
  uint32_t  crc;
  uint8_t   eof;
}__attribute__((packed)) ETX_OTA_RESP_;

WiFiMulti wifiMulti;

uint32_t total_file_size=0;
uint8_t DATA_BUF[1024];

/*bool is_ack_resp_received( int comport )
{
  bool is_ack  = false;
  memset(DATA_BUF, 0, ETX_OTA_PACKET_MAX_SIZE);

  while(Serial2.available()>0){
      Serial2.read();
    }
  uint16_t len =  RS232_PollComport( comport, DATA_BUF, sizeof(ETX_OTA_RESP_));
  if( len > 0 )
  {
    ETX_OTA_RESP_ *resp = (ETX_OTA_RESP_*) DATA_BUF;
    if( resp->packet_type == ETX_OTA_PACKET_TYPE_RESPONSE )
    {
      //TODO: Add CRC check
      if( resp->status == ETX_OTA_ACK )
      {
        //ACK received
        is_ack = true;
      }
    }
  }
  return is_ack;
}*/

void send_startframe(){

    ETX_OTA_COMMAND_ *ota_start = (ETX_OTA_COMMAND_*)DATA_BUF;

    memset(DATA_BUF, 0, 1024);

    ota_start->sof = ETX_OTA_SOF;
    ota_start->packet_type = ETX_OTA_PACKET_TYPE_CMD;
    ota_start->data_len = 1;
    ota_start->cmd = ETX_OTA_CMD_START;
    ota_start->crc = 0x00;
    ota_start->eof = ETX_OTA_EOF;

    int len = sizeof(ETX_OTA_COMMAND_);

    for(int i = 0; i < len; i++)
    {
      Serial2.write(DATA_BUF[i]);    
      delay(10);
    }

       USE_SERIAL.println("OTA Start command sent.");
  }

void send_endframe(){

    ETX_OTA_COMMAND_ *ota_end = (ETX_OTA_COMMAND_*)DATA_BUF;

    memset(DATA_BUF, 0, 1024);

    ota_end->sof = ETX_OTA_SOF;
    ota_end->packet_type = ETX_OTA_PACKET_TYPE_CMD;
    ota_end->data_len = 1;
    ota_end->cmd = ETX_OTA_CMD_END;
    ota_end->crc = 0x00;
    ota_end->eof = ETX_OTA_EOF;

    int len = sizeof(ETX_OTA_COMMAND_);

    for(int i = 0; i < len; i++)
    {
      Serial2.write(DATA_BUF[i]);    
      delay(10);
    }

        USE_SERIAL.println("OTA END command sent.");
  }

void send_headerframe(meta_info *ota_info){

    ETX_OTA_HEADER_ *ota_header = (ETX_OTA_HEADER_*)DATA_BUF;

    memset(DATA_BUF, 0, 1024);

    ota_header->sof = ETX_OTA_SOF;
    ota_header->packet_type = ETX_OTA_PACKET_TYPE_HEADER;
    ota_header->data_len = 16;
    ota_header->crc = 0x00;
    ota_header->eof = ETX_OTA_EOF;
    
    memcpy(&ota_header->meta_data, ota_info, sizeof(meta_info) );

    int len = sizeof(ETX_OTA_HEADER_);

    for(int i = 0; i < len; i++)
    {
      Serial2.write(DATA_BUF[i]);    
      delay(10);
    }

        USE_SERIAL.println("OTA Header sent.");
  }

void send_dataframe(uint8_t *dat, uint16_t data_len){
    uint16_t leng=0;
    ETX_OTA_DATA_ *ota_data = (ETX_OTA_DATA_*)DATA_BUF;

    memset(DATA_BUF, 0, 1024);

    ota_data->sof = ETX_OTA_SOF;
    ota_data->packet_type = ETX_OTA_PACKET_TYPE_DATA;
    ota_data->data_len = data_len;
    
    leng = 4;

    memcpy(&DATA_BUF[leng], dat, data_len );
    leng += data_len;
    uint32_t crc = 0u;      

    memcpy(&DATA_BUF[leng], (uint8_t*)&crc, sizeof(crc) );
    leng += sizeof(crc);

    DATA_BUF[leng] = ETX_OTA_EOF;
    leng++;

    for(int i = 0; i < leng; i++)
    {
      Serial2.write(DATA_BUF[i]);    
      delay(10);
    }

        USE_SERIAL.println("OTA DATA sent.");
  }
  
void setup() {

    USE_SERIAL.begin(115200);
    Serial2.begin(115200,SERIAL_8N1 ,RXD2, TXD2);

    USE_SERIAL.println();
    USE_SERIAL.println();
    USE_SERIAL.println();

    for(uint8_t t = 4; t > 0; t--) {
        USE_SERIAL.printf("[SETUP] WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000);
    }

    wifiMulti.addAP("ultra_error_404", "file_not_found");

    delay(1000);

    if((wifiMulti.run() == WL_CONNECTED)) {

        HTTPClient http;

        USE_SERIAL.print("[HTTP] begin...\n");

        // configure server and url
        //http.begin("http://192.168.1.12/test.html");
        //http.begin("192.168.1.51", 8000, "/ota_server.py");
        http.begin("https://stm-ota.s3.amazonaws.com/Application.bin");

        USE_SERIAL.print("[HTTP] GET...\n");
        // start connection and send HTTP header
        int httpCode = http.GET();
        if(httpCode > 0) {
            // HTTP header has been send and Server response header has been handled
            USE_SERIAL.printf("[HTTP] GET... code: %d\n", httpCode);

            // file found at server
            if(httpCode == HTTP_CODE_OK) {
                 send_startframe();
                 delay(10);
                 
                // get length of document (is -1 when Server sends no Content-Length header)
                int len = http.getSize();
                if(len>0){
                  total_file_size=len;
                  }

                  meta_info ota_info;
                  ota_info.package_size = total_file_size;
                  ota_info.package_crc  = 0;
              
                  send_headerframe(&ota_info);
                  delay(100);
                // create buffer for read
                uint8_t buff[512] = { 0 };

                // get tcp stream
                WiFiClient * stream = http.getStreamPtr();

                // read all data from server
                while(http.connected() && (len > 0 || len == -1)) {
                    // get available data size
                    size_t size = stream->available();

                    if(size) {
                        // read up to 128 byte
                        uint16_t c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));

                        // write it to Serial
                        //USE_SERIAL.write(buff, c);    
                        //uint8_t dt[4] = {1,3,4,5};
                          
                          send_dataframe(buff,c);
                          delay(1000);
                          
                        USE_SERIAL.printf("Downloaded %d bytes of %d bytes\n",total_file_size-len+c,total_file_size);

                        if(len > 0) {
                            len -= c;
                        }
                    }
                    delay(10);
                }

                USE_SERIAL.println();
                USE_SERIAL.print("[HTTP] connection closed or file end.\n");
                send_endframe();
                delay(10);

            }
        } else {
            USE_SERIAL.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }

        http.end();
    }
}


void loop() {

}
