#ifndef rfid_control_h
#define rfid_control_h

#define RFID_LIMIT_DDR DDRF
#define RFID_LIMIT_PORTD PORTF
#define RFID_LIMIT_PIN PINF
#define RFID_LIMIT_BIT 5

extern uint8_t toolLed[];
void rfid_control_init();
void set_rfid(uint8_t flag);
void rfid_read(uint8_t* return_data);
void rfid_write(uint8_t toolNumber, uint16_t time);
void read_all_rfid();

#endif
