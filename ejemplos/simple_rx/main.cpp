#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>
#include <netinet/in.h> 

#include <sys/ioctl.h>
#include <net/if.h>

//#include <gps.h>


using namespace std;

#include "base64.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "json.hpp"


bool gwEnabled = true;

typedef bool boolean;
typedef unsigned char byte;

static const int CHANNEL = 0;

byte currentMode = 0x81;

char message[256];
char b64[256];

bool sx1272 = true;

byte receivedbytes;

struct sockaddr_in si_other, si_out;
int s, so, slen=sizeof(si_other);
struct ifreq ifr;

uint32_t cp_nb_rx_rcv;
uint32_t cp_nb_rx_ok;
uint32_t cp_nb_rx_bad;
uint32_t cp_nb_rx_nocrc;
uint32_t cp_up_pkt_fwd;
uint32_t pkt_tx;
uint32_t pkt_rx;

enum sf_t { SF7=7, SF8, SF9, SF10, SF11, SF12 };

/*******************************************************************************
 *
 * Configure these values!
 *
 *******************************************************************************/

// SX1272 - Raspberry connections

/* Agrotools GW */
int ssPin = 21;
int dio0  = 22;
int RST   = 23;

/* Dragino */
/*int ssPin = 6;
int dio0  = 7;
int RST   = 0;*/

// Set spreading factor (SF7 - SF12)
sf_t sf = SF12;

// Set center frequency
uint32_t  freq = 903900000; // in Mhz! (868.1)
uint8_t modem_config = 0x78; // 125kHz 4/8 Explicit mode
// Set location
float lat=-35.166068;
float lon=-59.09168;
int   alt=0;

/* Informal status fields */
static char platform[24]    = "qts_proto_gw";  /* platform definition */
static char email[40]       = "";                        /* used for contact email */
static char description[64] = "";                        /* used for free form description */

// define servers
// TODO: use host names and dns
#define SERVER1 "127.0.0.1"    
//#define SERVER2 "192.168.1.10"      // local
#define PORT 1700                   // The port on which to send data

// #############################################
// #############################################

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB        0x1F
#define REG_PKT_SNR_VALUE           0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH      0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD               0x39
#define REG_VERSION                 0x42

#define SX72_MODE_RX_CONTINUOS      0x85
#define SX72_MODE_TX                0x83
#define SX72_MODE_SLEEP             0x80
#define SX72_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN                0x20

// CONF REG
#define REG1                        0x0A
#define REG2                        0x84

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12

// FRF
#define        REG_FRF_MSB              0x06
#define        REG_FRF_MID              0x07
#define        REG_FRF_LSB              0x08

#define        FRF_MSB                  0xD9 // 868.1 Mhz
#define        FRF_MID                  0x06
#define        FRF_LSB                  0x66

#define BUFLEN 2048  //Max length of buffer

#define PROTOCOL_VERSION  1
#define PKT_PUSH_DATA 0
#define PKT_PUSH_ACK  1
#define PKT_PULL_DATA 2
#define PKT_PULL_RESP 3
#define PKT_PULL_ACK  4

#define TX_BUFF_SIZE  2048
#define STATUS_SIZE   1024

// Constants for radio registers
#define OPMODE_LORA      0x80
#define OPMODE_MASK      0x07
#define OPMODE_SLEEP     0x00
#define OPMODE_STANDBY   0x01
#define OPMODE_FSTX      0x02
#define OPMODE_TX        0x03
#define OPMODE_FSRX      0x04
#define OPMODE_RX        0x05
#define OPMODE_RX_SINGLE 0x06
#define OPMODE_CAD       0x07

#define RegDioMapping1                             0x40 // common
#define RegDioMapping2                             0x41 // common

#define RegPaConfig                                0x09 // common
#define RegPaRamp                                  0x0A // common
#define RegPaDac                                   0x5A // common

// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP_DIO1_LORA_NOP      0x30  // --11----
#define MAP_DIO2_LORA_NOP      0xC0  // ----11--

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80
#define IRQ_LORA_RXDONE_MASK 0x40
#define IRQ_LORA_CRCERR_MASK 0x20
#define IRQ_LORA_HEADER_MASK 0x10
#define IRQ_LORA_TXDONE_MASK 0x08
#define IRQ_LORA_CDDONE_MASK 0x04
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01


void die(const char *s)
{
    perror(s);
    exit(1);
}

void selectreceiver()
{
    digitalWrite(ssPin, LOW);
}

void unselectreceiver()
{
    digitalWrite(ssPin, HIGH);
}

byte readRegister(byte addr)
{
    unsigned char spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];
}

void writeRegister(byte addr, byte value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    unselectreceiver();
}


boolean receivePkt(char *payload)
{

    // clear rxDone
    writeRegister(REG_IRQ_FLAGS, 0x40);

    int irqflags = readRegister(REG_IRQ_FLAGS);

    cp_nb_rx_rcv++;

    //  payload crc: 0x20
    if((irqflags & 0x20) == 0x20)
    {
        printf("CRC error\n");
        writeRegister(REG_IRQ_FLAGS, 0x20);
        cp_nb_rx_bad++;
        return false;
    } else {

        cp_nb_rx_ok++;

        byte currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
        byte receivedCount = readRegister(REG_RX_NB_BYTES);
        receivedbytes = receivedCount;

        writeRegister(REG_FIFO_ADDR_PTR, currentAddr);

        for(int i = 0; i < receivedCount; i++)
        {
            payload[i] = (char)readRegister(REG_FIFO);
        }
    }
    return true;
}

void SetupLoRa()
{
    
    digitalWrite(RST, HIGH);
    delay(100);
    digitalWrite(RST, LOW);
    delay(100);

    byte version = readRegister(REG_VERSION);

    if (version == 0x22) {
        // sx1272
        printf("SX1272 detected, starting.\n");
        sx1272 = true;
    } else {
        // sx1276?
        digitalWrite(RST, LOW);
        delay(100);
        digitalWrite(RST, HIGH);
        delay(100);
        version = readRegister(REG_VERSION);
        if (version == 0x12) {
            // sx1276
            printf("SX1276 detected, starting.\n");
            sx1272 = false;
        } else {
            printf("Unrecognized transceiver.\n");
            //printf("Version: 0x%x\n",version);
            exit(1);
        }
    }

    writeRegister(REG_OPMODE, SX72_MODE_SLEEP);

    // set frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf>>16) );
    writeRegister(REG_FRF_MID, (uint8_t)(frf>> 8) );
    writeRegister(REG_FRF_LSB, (uint8_t)(frf>> 0) );

    writeRegister(REG_SYNC_WORD, 0x34); // LoRaWAN public sync word

    if (sx1272) {
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG,0x0B);
        } else {
            writeRegister(REG_MODEM_CONFIG,0x0A);
        }
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    } else {
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG3,0x0C);
        } else {
            writeRegister(REG_MODEM_CONFIG3,0x04);
        }
        writeRegister(REG_MODEM_CONFIG,modem_config);
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x04); //CRC enabled
    }

    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x08); 
    }
    writeRegister(REG_MAX_PAYLOAD_LENGTH,0x80);
    writeRegister(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
    writeRegister(REG_HOP_PERIOD,0xFF);
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));

    // Set Continous Receive Mode
    writeRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain, boost on
    writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);

}

static void opmode (uint8_t mode) {
    writeRegister(REG_OPMODE, (readRegister(REG_OPMODE) & ~OPMODE_MASK) | mode);
}

void sendudp(char *msg, int length) {

//send the update
#ifdef SERVER1
    inet_aton(SERVER1 , &si_other.sin_addr);
    if (sendto(s, (char *)msg, length, 0 , (struct sockaddr *) &si_other, slen)==-1)
    {
        die("sendto()");
    }
#endif

#ifdef SERVER2
    inet_aton(SERVER2 , &si_other.sin_addr);
    if (sendto(s, (char *)msg, length , 0 , (struct sockaddr *) &si_other, slen)==-1)
    {
        die("sendto()");
    }
#endif
}

void sendstat() {

    static char status_report[STATUS_SIZE]; /* status report as a JSON object */
    char stat_timestamp[24];
    time_t t;

    int stat_index=0;

    /* pre-fill the data buffer with fixed fields */
    status_report[0] = PROTOCOL_VERSION;
    status_report[3] = PKT_PUSH_DATA;

    status_report[4] = (unsigned char)ifr.ifr_hwaddr.sa_data[0];
    status_report[5] = (unsigned char)ifr.ifr_hwaddr.sa_data[1];
    status_report[6] = (unsigned char)ifr.ifr_hwaddr.sa_data[2];
    status_report[7] = 0xFF;
    status_report[8] = 0xFF;
    status_report[9] = (unsigned char)ifr.ifr_hwaddr.sa_data[3];
    status_report[10] = (unsigned char)ifr.ifr_hwaddr.sa_data[4];
    status_report[11] = (unsigned char)ifr.ifr_hwaddr.sa_data[5];

    /* start composing datagram with the header */
    uint8_t token_h = (uint8_t)rand(); /* random token */
    uint8_t token_l = (uint8_t)rand(); /* random token */
    status_report[1] = token_h;
    status_report[2] = token_l;
    stat_index = 12; /* 12-byte header */

    /* get timestamp for statistics */
    t = time(NULL);
    strftime(stat_timestamp, sizeof stat_timestamp, "%F %T %Z", gmtime(&t));

    int j = snprintf((char *)(status_report + stat_index), STATUS_SIZE-stat_index, "{\"stat\":{\"time\":\"%s\",\"lati\":%.5f,\"long\":%.5f,\"alti\":%i,\"rxnb\":%u,\"rxok\":%u,\"pkt_tx\":%u,\"pkt_rx\":%u,\"rxnb\":%u,\"txnb\":%u,\"pfrm\":\"%s\",\"desc\":\"%s\"}}", stat_timestamp, lat, lon, (int)alt, cp_nb_rx_rcv, cp_nb_rx_ok, pkt_tx, pkt_rx, 0, 0,platform,description);
    stat_index += j;
    status_report[stat_index] = 0; /* add string terminator, for safety */

    printf("stat update: %s\n", (char *)(status_report+12)); /* DEBUG: display JSON stat */

    //send the update
    sendudp(&status_report[12], stat_index-12);

}

void writeBuf(byte addr, byte *value, byte len) {                                                       
    unsigned char spibuf[256];                                                                          
    spibuf[0] = addr | 0x80;                                                                            
    for (int i = 0; i < len; i++) {                                                                         
        spibuf[i + 1] = value[i];                                                                       
    }                                                                                                   
    selectreceiver();                                                                                   
    wiringPiSPIDataRW(CHANNEL, spibuf, len + 1);                                                        
    unselectreceiver();                                                                                 
}

void txlora(byte *frame, byte datalen) {

    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeRegister(RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    // mask all IRQs but TxDone
    writeRegister(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_TXDONE_MASK);

    // initialize the payload size and address pointers
    writeRegister(REG_FIFO_TX_BASE_AD, 0x00);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);
    writeRegister(REG_PAYLOAD_LENGTH, datalen);

    // download buffer to the radio FIFO
    writeBuf(REG_FIFO, frame, datalen);
    // now we actually start the transmission
    opmode(OPMODE_TX);


    while(digitalRead(dio0) == 0){} // Wait for it
    
    printf("send: %s\n", frame);

    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeRegister(RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    // mask all IRQs but TxDone
    writeRegister(REG_IRQ_FLAGS_MASK, 0x00);        
    
}

static void opmodeLora() {
    uint8_t u = OPMODE_LORA;
    if (sx1272 == false)
        u |= 0x8;   // TBD: sx1276 high freq
    writeRegister(REG_OPMODE, u);
}

static void configPower (int8_t pw) {
    if (sx1272 == false) {
        // no boost used for now
        if(pw >= 17) {
            pw = 15;
        } else if(pw < 2) {
            pw = 2;
        }
        // check board type for BOOST pin
        writeRegister(RegPaConfig, (uint8_t)(0x80|(pw&0xf)));
        writeRegister(RegPaDac, readRegister(RegPaDac)|0x4);

    } else {
        // set PA config (2-17 dBm using PA_BOOST)
        if(pw > 17) {
            pw = 17;
        } else if(pw < 2) {
            pw = 2;
        }
        writeRegister(RegPaConfig, (uint8_t)(0x80|(pw-2)));
    }
}

using json::JSON;

void send_if_any(){
    char buffer[549];
    struct sockaddr_storage src_addr;
    socklen_t src_addr_len=sizeof(src_addr);
    ssize_t count=recvfrom(so,buffer,sizeof(buffer),0,(struct sockaddr*)&src_addr,&src_addr_len);
    if (count==-1) {
        //die("recv err");
    } else if (count==sizeof(buffer)) {
        //warning("datagram too large for buffer: truncated");
    } else if(count != 0) {
        buffer[count] = 0;
        


        JSON Obj = JSON::Load(buffer);
        cout << Obj["type"].ToString().compare("beacon") << endl;
        if(!(Obj["type"].ToString().compare("beacon"))){
            if(gwEnabled == false){
            return;
            }
            printf("Send packets at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
            printf("%s\n",buffer);
            printf("------------------\n");
            // es un beacon para polling
            cout << Obj["type"].ToString().compare("beacon") << endl;
            
            uint8_t lora_payload[4];
            uint16_t addr = Obj["addr"].ToInt();
            lora_payload[0] = (uint8_t)(addr & 0x00FF);
            lora_payload[1] = (uint8_t)((addr & 0xFF00)>>8);
            // Beacon -> type (4bits) = 0x2;
            lora_payload[2] = 0x20;
            lora_payload[3] = 0x00; //padding

            opmodeLora();
            // enter standby mode (required for FIFO loading))
            opmode(OPMODE_STANDBY);
            writeRegister(RegPaRamp, (readRegister(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
            configPower(23);
            txlora((byte*)lora_payload, 4);

            pkt_tx++;
            
            opmode(OPMODE_STANDBY);
            // Set Continous Receive Mode
            writeRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain
            writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);
        }else if(!(Obj["type"].ToString().compare("control"))){
            printf("\nControl packet\n");
            printf("%s\n",buffer);
            gwEnabled = Obj["enabled"].ToBool();


            freq = Obj["frecuencia"].ToInt(); // Frec
            sf = (sf_t) Obj["sf"].ToInt(); // SF
            uint8_t cr = Obj["cr"].ToInt(); // CR

            modem_config &= ~(0x0E); // Clean CR
            switch(cr){
                case 5:
                    modem_config |= 0x02;
                    break;
                case 6:
                    modem_config |= 0x04; 
                    break;
                case 7:
                    modem_config |= 0x06;
                    break;
                case 8:
                    modem_config |= 0x08;
                    break;
                default:
                    modem_config |= 0x02;
                    break;
            }

            SetupLoRa();

        }

       }
}

void receivepacket() {

    long int SNR;
    int rssicorr;

    if(digitalRead(dio0) == 1)
    {
        if(receivePkt(message)) {
            byte value = readRegister(REG_PKT_SNR_VALUE);
            if( value & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                SNR = -value;
            }
            else
            {
                // Divide by 4
                SNR = ( value & 0xFF ) >> 2;
            }
            
            if (sx1272) {
                rssicorr = 139;
            } else {
                rssicorr = 157;
            }

            printf("Packet RSSI: %d, ",readRegister(0x1A)-rssicorr);
            printf("RSSI: %d, ",readRegister(0x1B)-rssicorr);
            printf("SNR: %li, ",SNR);
            printf("Length: %i",(int)receivedbytes);
            printf("\n");

            int j;
            j = bin_to_b64((uint8_t *)message, receivedbytes, (char *)(b64), 341);
            //fwrite(b64, sizeof(char), j, stdout);

            char buff_up[TX_BUFF_SIZE]; /* buffer to compose the upstream packet */
            int buff_index=0;

            /* gateway <-> MAC protocol variables */
            //static uint32_t net_mac_h; /* Most Significant Nibble, network order */
            //static uint32_t net_mac_l; /* Least Significant Nibble, network order */

            /* pre-fill the data buffer with fixed fields */
            buff_up[0] = PROTOCOL_VERSION;
            buff_up[3] = PKT_PUSH_DATA;

            /* process some of the configuration variables */
            //net_mac_h = htonl((uint32_t)(0xFFFFFFFF & (lgwm>>32)));
            //net_mac_l = htonl((uint32_t)(0xFFFFFFFF &  lgwm  ));
            //*(uint32_t *)(buff_up + 4) = net_mac_h;
            //*(uint32_t *)(buff_up + 8) = net_mac_l;

            buff_up[4] = (unsigned char)ifr.ifr_hwaddr.sa_data[0];
            buff_up[5] = (unsigned char)ifr.ifr_hwaddr.sa_data[1];
            buff_up[6] = (unsigned char)ifr.ifr_hwaddr.sa_data[2];
            buff_up[7] = 0xFF;
            buff_up[8] = 0xFF;
            buff_up[9] = (unsigned char)ifr.ifr_hwaddr.sa_data[3];
            buff_up[10] = (unsigned char)ifr.ifr_hwaddr.sa_data[4];
            buff_up[11] = (unsigned char)ifr.ifr_hwaddr.sa_data[5];

            /* start composing datagram with the header */
            uint8_t token_h = (uint8_t)rand(); /* random token */
            uint8_t token_l = (uint8_t)rand(); /* random token */
            buff_up[1] = token_h;
            buff_up[2] = token_l;
            buff_index = 12; /* 12-byte header */

            // TODO: tmst can jump is time is (re)set, not good.
            struct timeval now;
            gettimeofday(&now, NULL);
            uint32_t tmst = (uint32_t)(now.tv_sec*1000000 + now.tv_usec);
        uint32_t tmms = (uint32_t)(now.tv_sec*1000 + now.tv_usec/1000);
        time_t now_t;
        time(&now_t); 
           
        /* start of JSON structure */
            memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[", 9);
            buff_index += 9;
            buff_up[buff_index] = '{';
            ++buff_index;
            
       j=strftime((char *)(buff_up + buff_index), 50, "\"time\":\"%FT%T", gmtime(&now_t));    
        
       // j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, "\"time\":%s", tim);
       buff_index += j;
       
        j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ".%ldZ\",\"tmms\":%u",now.tv_usec, tmms);
            buff_index += j;
           
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"tmst\":%u", tmst);
            buff_index += j;
        
        memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
            buff_index += 9;
            memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
            buff_index += 14;
            /* Lora datarate & bandwidth, 16-19 useful chars */
            switch (sf) {
            case SF7:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF7", 12);
                buff_index += 12;
                break;
            case SF8:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF8", 12);
                buff_index += 12;
                break;
            case SF9:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF9", 12);
                buff_index += 12;
                break;
            case SF10:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF10", 13);
                buff_index += 13;
                break;
            case SF11:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF11", 13);
                buff_index += 13;
                break;
            case SF12:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF12", 13);
                buff_index += 13;
                break;
            default:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF?", 12);
                buff_index += 12;
            }
            memcpy((void *)(buff_up + buff_index), (void *)"BW125\"", 6);
            buff_index += 6;
            memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/5\"", 13);
            buff_index += 13;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"lsnr\":%li", SNR);
            buff_index += j;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"rssi\":%d,\"size\":%u", readRegister(0x1A)-rssicorr, receivedbytes);
            buff_index += j;
            memcpy((void *)(buff_up + buff_index), (void *)",\"data\":\"", 9);
            buff_index += 9;
            j = bin_to_b64((uint8_t *)message, receivedbytes, (char *)(buff_up + buff_index), 341);
            buff_index += j;
            buff_up[buff_index] = '"';
            ++buff_index;

            uint16_t addr = message[1];
            addr <<= 8;
            addr |= message[0]; 

            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"addr\":%d", addr);
            buff_index += j;

            /* End of packet serialization */
            buff_up[buff_index] = '}';
            ++buff_index;
            buff_up[buff_index] = ']';
            ++buff_index;
            /* end of JSON datagram payload */
            buff_up[buff_index] = '}';
            ++buff_index;
            buff_up[buff_index] = 0; /* add string terminator, for safety */

            printf("rxpk update: %s\n", (char *)(buff_up + 12)); /* DEBUG: display JSON payload */

            //send the messages
            sendudp(&buff_up[12], buff_index-12);

            pkt_rx++;

            fflush(stdout);


            // uint8_t lora_payload[4];
            // uint16_t addr = 1000;
            // lora_payload[0] = (uint8_t)(addr & 0x00FF);
            // lora_payload[1] = (uint8_t)((addr & 0xFF00)>>8);
            // // Beacon -> type (4bits) = 0x2;
            // lora_payload[2] = 0x20;
            // lora_payload[3] = 0x00; //padding

            // opmodeLora();
            // // enter standby mode (required for FIFO loading))
            // opmode(OPMODE_STANDBY);
            // writeRegister(RegPaRamp, (readRegister(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
            // configPower(23);
            // txlora((byte*)lora_payload, 4);


            // // Set Continous Receive Mode
            // writeRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain
            // writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);

        } // received a message

    } // dio0=1
}

int main () {

    struct timeval nowtime;
    uint32_t lasttime;

    wiringPiSetup () ;
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);

    //int fd = 
    wiringPiSPISetup(CHANNEL, 500000);
    //cout << "Init result: " << fd << endl;

    SetupLoRa();

    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
    if ( (so=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("so socket");
    }

    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    si_other.sin_addr.s_addr = INADDR_ANY;

    memset((char *) &si_out, 0, sizeof(si_out));
    si_out.sin_family = AF_INET;
    si_out.sin_port = htons(PORT+1);
    si_out.sin_addr.s_addr = INADDR_ANY;

    ifr.ifr_addr.sa_family = AF_INET;
    
    strncpy(ifr.ifr_name, "eth0", IFNAMSIZ-1);  // can we rely on eth0?
    ioctl(s, SIOCGIFHWADDR, &ifr);

    int flags = 1;
    ioctl(so, FIONBIO, &flags);

    if (bind(so,(const struct sockaddr *)&si_out,sizeof(si_out))==-1) {
      die("Bind err");
    }
   // si_other.sin_port = htons(PORT);
    

    /* display result */
    printf("Gateway ID: %.2x:%.2x:%.2x:ff:ff:%.2x:%.2x:%.2x\n",
           (unsigned char)ifr.ifr_hwaddr.sa_data[0],
           (unsigned char)ifr.ifr_hwaddr.sa_data[1],
           (unsigned char)ifr.ifr_hwaddr.sa_data[2],
           (unsigned char)ifr.ifr_hwaddr.sa_data[3],
           (unsigned char)ifr.ifr_hwaddr.sa_data[4],
           (unsigned char)ifr.ifr_hwaddr.sa_data[5]);

    printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
    printf("------------------\n");

    int rc;
    struct timeval tv;

  /*  struct gps_data_t gps_data;
    if ((rc = gps_open("localhost", "2947", &gps_data)) == -1) {
        printf("code: %d, reason: %s\n", rc, gps_errstr(rc));
    }else{
        gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);
    }*/



    while(1) {

        send_if_any();

        if(gwEnabled == false){
            continue;
        }

        receivepacket();
        
        gettimeofday(&nowtime, NULL);
        uint32_t nowseconds = (uint32_t)(nowtime.tv_sec);
        if (nowseconds - lasttime >= 30) {
            lasttime = nowseconds;
//                if ((rc = gps_read(&gps_data)) == -1) {
//                    printf("error occured reading gps data. code: %d, reason: %s\n", rc, gps_errstr(rc));
//                } else {
                    /* Display data from the GPS receiver. */
                 /*   if ((gps_data.status == STATUS_FIX) && 
                        (gps_data.fix.mode == MODE_2D || gps_data.fix.mode == MODE_3D) &&
                        !isnan(gps_data.fix.latitude) && 
                        !isnan(gps_data.fix.longitude)) {
                            //gettimeofday(&tv, NULL); EDIT: tv.tv_sec isn't actually the timestamp!
                            printf("latitude: %f, longitude: %f, speed: %f, timestamp: %lf\n", gps_data.fix.latitude, gps_data.fix.longitude, gps_data.fix.speed, gps_data.fix.time);
                        lat = gps_data.fix.latitude;
                        lon = gps_data.fix.longitude;
                    } else {
                        printf("no GPS data available\n");
                    }*/
//                }
           
            sendstat();
            cp_nb_rx_rcv = 0;
            cp_nb_rx_ok = 0;
            cp_up_pkt_fwd = 0;
        }
        delay(1);
    }

    return (0);

}

