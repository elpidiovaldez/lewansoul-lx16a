/* Author: Paul Bates, heavily edited from original version by Zhihao Zheng (Arthur) */

#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include "lx16a.h"

#define FIELD(T,PKT,N) (*((T *) &PKT[N]))

int socket_fd_ = -1;
int baudrate_  = 115000;

int abs(int x) {
    return x<0 ? -x : x;
}

short clamp(short min, short max, short val) {
    if (val > max) {
        return max;
    } else if (val < min) {
        return min;
    } else {
        return val;
    }
}

void closePort() {
    if(socket_fd_ != -1)
        close(socket_fd_);
    socket_fd_ = -1;
}

void clearPort() {
    tcflush(socket_fd_, TCIFLUSH);
}

int writePort(uint8_t *packet, int length) {
    return write(socket_fd_, packet, length);
}

int readPort(uint8_t *packet, int length) {
    return read(socket_fd_, packet, length);
}

int sum(int paramc, unsigned char* params) {
    int sum = 0;
    
    for(int i=0; i<paramc; ++i) {
        sum += params[i];
    }
    
    return sum;
}

void printPacket(unsigned len, unsigned char* pkt) {
    printf("Packet = ");
    for(unsigned j=0; j<len; j++) {
        printf("%x\t",pkt[j]);
    }
    printf("\n");
}

//Return 0 if packet sent, or error code if send failed.
int txPacket(unsigned servo_id, unsigned cmd_id, unsigned paramc, uint8_t *params) {
    int i = 0;
    unsigned length = paramc + 3;
    unsigned total_packet_length = paramc+6;
    unsigned checksum = ~(servo_id + length + cmd_id + sum(paramc, params));

    // make packet header
    uint8_t txpacket[total_packet_length];
    txpacket[0] = 0x55;
    txpacket[1] = 0x55;
    txpacket[2] = servo_id;
    txpacket[3] = length;
    txpacket[4] = cmd_id;
    while (i < paramc) {
        txpacket[5+i] = params[i];
        i++;
    }
    txpacket[paramc+5] = checksum;

    // tx packet
    clearPort();
    uint16_t written_packet_length = writePort(txpacket, total_packet_length);
    if (total_packet_length != written_packet_length) {
        printf("writing length error!\n");
        return 1;
    }

    return 0;
}

//Returns 0 if packet read successfully, or error code if read failed.
int rxPacket(int buffLength, uint8_t *rxpacket) {
    
    unsigned rx_length   = 0;
    unsigned wait_length = 7;  //Minimum RECEIVABLE packet length is 4+3 (i.e. 55 55 servoId length command param1 checksum)
    unsigned maxLoops    = 50; //Ensure function terminates in a reasonable time.
    unsigned numLoops    = 0;
    
    while(numLoops++ < maxLoops) {
        
        if (buffLength<wait_length) {
            return 1;
        }
        
        rx_length += readPort(&rxpacket[rx_length], wait_length - rx_length);
                
        if (rx_length >= wait_length) {
            uint16_t idx = 0;
            
            // find packet header
            for (idx = 0; idx < (rx_length - 1); idx++) {
                if ((rxpacket[idx]==0x55) && (rxpacket[idx+1]==0x55)) break;
            }

            if (idx != 0) {
                // remove unexpected bytes packets (move to start of packet).
                for (uint16_t s=0; s<rx_length - idx; s++) {
                    rxpacket[s] = rxpacket[idx + s];
                }
                rx_length -= idx;
                
            } else {
                // re-calculate the exact length of the rx packet
                wait_length = rxpacket[3] + 3;

                if (rx_length < wait_length) {
                    continue;  // packet not complete - breakout and receive more
                }
                
                if (rx_length > wait_length) {
                    return 1;  // packet length field must be incorrect!
                }

                // packet has been fully received, so now verify checksum
                uint8_t rx_checksum = rxpacket[wait_length-1];
                uint8_t checksum    = ~sum(wait_length-3, &rxpacket[2]);
                
                if (checksum == rx_checksum) {
                    return 0;
                } else {
                    printf("checksum error!\n");
                    printPacket(rx_length, rxpacket);
                    printf("calculated checksum = %x\n", checksum);
                    return 2;
                }
                
            }
        }
        
        usleep(1000);
    }

    return 3;
}

//Returns 0 if packet sent and response received, otherwise returns error code.
int txrxPacket(uint8_t servo_id, uint8_t cmd_id, uint16_t paramc, uint8_t *params, uint8_t *rxpacket) {

    // tx packet
    if (txPacket(servo_id, cmd_id, paramc, params) != 0) {
        printf("Couldn't send command\n");
        return 1;
    }

    // rx packet
    int loopCount = 10;
    do {
        if (rxPacket(10, rxpacket)!=0) return 2;
        
        if (rxpacket[2]==servo_id) return 0;
        
    } while (--loopCount!=0);
    
    printf("rx packet timeout\n");
    return 3;
}

void setServoID(char id, char new_id) {
    uint8_t params[1];
    params[0] = new_id;
    if (txPacket(id, SERVO_ID_WRITE, 1, params)!=0) {
        printf("SERVO_ID_WRITE error\n");
    }
}

void move(char id, short position, short time) {
    uint8_t params[4];
    FIELD(short,params,0) = clamp(0, 1000, position);
    FIELD(short,params,2) = clamp(0, 30000, time);

    if (txPacket(id, SERVO_MOVE_TIME_WRITE, 4, params)!=0) {
        printf("SERVO_MOVE_TIME_WRITE error\n");
    }
}

int getMove(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_MOVE_TIME_READ, 0, NULL, rxpacket)==0) {
        short pos  = FIELD(short, rxpacket, 5);
        short time = FIELD(short, rxpacket, 7);
        int   data = (pos << 16 | time);
        return data;
    } 
    
    printf("SERVO_MOVE_TIME_READ error!\n");
    return (2000<< 16) ;
}

void movePrepare(char id, short position, short time) {
    uint8_t params[4];
    FIELD(short, params, 0) = clamp(0, 1000, position);
    FIELD(short, params, 2) = clamp(0, 30000, time);

    if (txPacket(id, SERVO_MOVE_TIME_WAIT_WRITE, 4, params)!=0) {
        printf("SERVO_MOVE_TIME_WAIT_WRITE error\n");
    }
}

int getPreparedMove(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_MOVE_TIME_WAIT_READ, 0, NULL, rxpacket)==0) {
        short pos  = FIELD(short, rxpacket, 5);
        short time = FIELD(short, rxpacket, 7);
        int   data = (pos << 16 | time);
        return data;
    } 
    
    printf("SERVO_MOVE_TIME_WAIT_READ error!\n");
    return (2000<< 16) ;
}

void moveStart(char id) {

    if (txPacket(id, SERVO_MOVE_START, 0, NULL)!=0) {
        printf("SERVO_MOVE_START error\n");
    }
}

void moveStop(char id) {

    if (txPacket(id, SERVO_MOVE_STOP, 0, NULL)!=0) {
        printf("SERVO_MOVE_STOP error\n");
    }
}

void setPositionOffset(char id, char deviation) {
    uint8_t params[1];
    params[0] = clamp(-125, 125, deviation);
    
    if (txPacket(id, SERVO_ANGLE_OFFSET_ADJUST, 1, params)!=0) {
        printf("SERVO_ANGLE_OFFSET_ADJUST error\n");
    }
}

void savePositionOffset(char id) {
    if (txPacket(id, SERVO_ANGLE_OFFSET_WRITE, 0, NULL)!=0) {
        printf("SERVO_ANGLE_OFFSET_WRITE error\n");
    }
}

int getPositionOffset(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_ANGLE_OFFSET_READ, 0, NULL, rxpacket)==0) {
        int offset = rxpacket[5];
        return offset;
    }
    
    printf("read pos error!\n");
    return 127;
}

void setPositionLimits(char id, short minPosition, short maxPosition) {
    uint8_t params[4];
    FIELD(short, params, 0) = clamp(0, 1000, minPosition);
    FIELD(short, params, 2) = clamp(0, 1000, maxPosition);
    
    if (txPacket(id, SERVO_ANGLE_LIMIT_WRITE, 4, params)!=0) {
        printf("SERVO_ANGLE_LIMIT_WRITE error\n");
    }
}

int getPositionLimits(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_ANGLE_LIMIT_READ, 0, NULL, rxpacket)==0) {
        short minpos = FIELD(short, rxpacket, 5);
        short maxpos = FIELD(short, rxpacket, 7);
        int data = (minpos << 16 | maxpos);
        return data;
    }
    
    printf("SERVO_ANGLE_LIMIT_READ error!\n");
    return (2000 << 16 | 2000);
}

void setVoltageLimits(char id, short min_volt, short max_volt) {
    uint8_t params[4];
    FIELD(short, params, 0) = clamp(4500, 12000, min_volt);
    FIELD(short, params, 2) = clamp(4500, 12000, max_volt);
    
    if (txPacket(id, SERVO_VIN_LIMIT_WRITE, 4, params)!=0) {
        printf("SERVO_VIN_LIMIT_WRITE error\n");
    }
}

int getVoltageLimits(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_VIN_LIMIT_READ, 0, NULL, rxpacket)==0) {
        short min_volt = FIELD(short, rxpacket, 5);
        short max_volt = FIELD(short, rxpacket, 7);
        int   data     = (min_volt << 16 | max_volt);
        return data;
    }
    
    printf("SERVO_VIN_LIMIT_READ error!\n");
    return (13000<<16);
}

void setMaxTemp(char id, char temp) {
    uint8_t params[1];
    params[0] = temp;
    if (txPacket(id, SERVO_TEMP_MAX_LIMIT_WRITE, 1, params)!=0) {
        printf("SERVO_TEMP_MAX_LIMIT_WRITE error\n");
    }
}

int getMaxTemp(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_TEMP_MAX_LIMIT_READ, 0, NULL, rxpacket)==0) {
        int maxtemp = rxpacket[5];
        return maxtemp;
    }
    printf("SERVO_TEMP_MAX_LIMIT_READ error!\n");
    return 127;
}

int getTemp(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_TEMP_READ, 0, NULL, rxpacket)==0) {
        int temp = rxpacket[5];
        return temp;
    }
    
    printf("SERVO_TEMP_READ error!\n");
    return 127;
}

int getVoltage(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_VIN_READ, 0, NULL, rxpacket)==0) {
        int vol = *((short *) &rxpacket[5]);
        return vol;
    }
    
    printf("SERVO_VIN_READ error!\n");
    return 20000;
}

void motorOn(char id) {
    uint8_t params[1] = {1};
    if (txPacket(id, SERVO_LOAD_OR_UNLOAD_WRITE, 1, params)!=0) {
        printf("SERVO_LOAD_OR_UNLOAD_WRITE error\n");
    }
}

void motorOff(char id) {
    uint8_t params[1] = {0};
    if (txPacket(id, SERVO_LOAD_OR_UNLOAD_WRITE, 1, params)!=0) {
        printf("SERVO_LOAD_OR_UNLOAD_WRITE error\n");
    }
}

int isMotorOn(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_LOAD_OR_UNLOAD_READ, 0, NULL, rxpacket)==0) {
        int motoron = rxpacket[5];
        return motoron;
    }
    
    printf("SERVO_LOAD_OR_UNLOAD_READ error!\n");
    return 127;
}

void setLED(char id, char status) {
    uint8_t params[1];
    params[0] = status;

    if (txPacket(id, SERVO_LED_CTRL_WRITE, 1, params)!=0) {
        printf("SERVO_LED_CTRL_WRITE error\n");
    }
}

int isLEDOn(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_LED_CTRL_READ, 0, NULL, rxpacket)==0) {
        int ledOn = rxpacket[5];
        return ledOn;
    }
    
    printf("SERVO_LED_CTRL_READ error!\n");
    return 127;
}

void setLEDErrors(char id, char error) {
    uint8_t params[1] = {clamp(0, 7, error)};
    if (txPacket(id, SERVO_LED_ERROR_WRITE, 1, params)!=0) {
        printf("SERVO_LED_ERROR_WRITE error\n");
    }
}

int getLEDErrors(char id) {
    uint8_t rxpacket[10];
    
    if (txrxPacket(id, SERVO_LED_ERROR_READ, 0, NULL, rxpacket)==0) {
        int ledErr = rxpacket[5];
        return ledErr;
    }
    
    printf("SERVO_LED_ERROR_READ error!\n");
    return 127;
}

void setPositionMode(char id){
    uint8_t params[4];

    params[0] = 0;
    params[1] = 0;
    params[2] = 0;
    params[3] = 0;

    if (txPacket(id, SERVO_OR_MOTOR_MODE_WRITE, 4, params)!=0) {
        printf("setServoMode error\n");
    }
}

int getMode(char id){
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_OR_MOTOR_MODE_READ, 0, NULL, rxpacket)==0) {
        int mode = rxpacket[5];
        return mode;
    }
    
    printf("SERVO_OR_MOTOR_MODE_READ error!\n");
    return 127;
}

//Beware: Setting speed to 0 turns the motor off (no torque). The motor
//will not respond to subsequent move commands until you have executed:
//  motorOn(id);
//  setPositionMode(id);
//Do not do these steps in the opposite order:
//  setMode(id);
//  setPositionMode(id);
//... or if the motor has received a move command while inert, it will be 
//executed as soon as the motor is turned on.
void setSpeed(char id, short speed) {
    uint8_t params[4];
    
    params[0] = 1;
    params[1] = 0;
    FIELD(short, params, 2) = clamp(-1000, 1000, speed);

    if (txPacket(id, SERVO_OR_MOTOR_MODE_WRITE, 4, params)) {
        printf("setspeed error\n");
    }
}

int getSpeed(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_OR_MOTOR_MODE_READ, 0, NULL, rxpacket)==0) {
        short spd = FIELD(short, rxpacket, 7);
        return spd;
    }
    
    printf("SERVO_OR_MOTOR_MODE_READ error!\n");
    return 10000;
}

int getPos(char id) {
    uint8_t rxpacket[10];

    if (txrxPacket(id, SERVO_POS_READ, 0, NULL, rxpacket)==0) {
        short pos = FIELD(short, rxpacket, 5);
        return pos;
    }
    
    printf("read pos error!\n");
    return 10000;
}

//Checks for a significant change in position over the sampleTime.
//WARNING: This function is quite unreliable.
int isMoving(char id, unsigned sensitivity, unsigned sampleTime) {
    int pos = getPos(id);
    usleep(sampleTime);
    int movement = abs(getPos(id) - pos);
    return movement>sensitivity;
}

//Returns when a movement command is close to it destination.
void waitForMove(char id, unsigned sensitivity) {
    int pos;
    unsigned target = getMove(id)>>16;
    do {
        pos = getPos(id);
    } while (abs(pos-target)>sensitivity);
}

int IO_init(char *filename) {
    struct termios newtio;
    
    closePort();
    socket_fd_ = open(filename, O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(socket_fd_ < 0) {
        printf(" Error opening serial port!\n");
    } else {
        bzero(&newtio, sizeof(newtio)); // clear struct for new port settings
        newtio.c_cflag      = B115200 | CS8 | CLOCAL | CREAD;
        newtio.c_iflag      = IGNPAR;
        newtio.c_oflag      = 0;
        newtio.c_lflag      = 0;
        newtio.c_cc[VTIME]  = 0;
        newtio.c_cc[VMIN]   = 0;

        // clean the buffer and activate the settings for the port
        tcflush(socket_fd_, TCIFLUSH);
        tcsetattr(socket_fd_, TCSANOW, &newtio);
}

    return socket_fd_;
}
