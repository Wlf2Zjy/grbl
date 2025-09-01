#include "grbl.h"

void switch_init()
{
    AIR_FAN_DDR |= (1 << AIR_FAN_BIT);
    SPINDLE_L_FAN_DDR |= (1 << SPINDLE_L_FAN_BIT);
    SPINDLE_R_FAN_DDR |= (1 << SPINDLE_R_FAN_BIT);
    BLOW_FAN_DDR |= (1 << BLOW_FAN_BIT);
    SUCTION_CUP_DDR |= (1 << SUCTION_CUP_BIT);
    LIGHT_DDR |= (1 << LIGHT_BIT);
    SPRAY_DDR |= (1 << SPRAY_BIT);
    L_WATER_DDR |= (1 << L_WATER_BIT);
    R_WATER_DDR |= (1 << R_WATER_BIT);
    OUTLINE_DDR |= (1 << OUTLINE_BIT);
    CAMERA_DDR |= (1 << CAMERA_BIT);
    RFID_ELE_DDR |= (1 << RFID_ELE_BIT);
    all_switch_stop();
}

void all_switch_stop()
{
    air_fan_control(0);
    spindle_fan_control(0);
    blow_fan_control(0);
    suction_cup_control(0);
    light_control(0);
    l_water_control(0);
    r_water_control(0);
    outline_control(0);
    camera_control(0);
    rfid_ele_control(0);
    spray_control(0);
}

// 1开0关
void air_fan_control(uint8_t flag)
{
    if (flag)
    {
        AIR_FAN_PORT |= (1 << AIR_FAN_BIT); // 高电平
    }
    else
    {
        AIR_FAN_PORT &= ~(1 << AIR_FAN_BIT); // 低电平
    }
    sys.airFanStatus = flag;
}

// 1左开， 2右开， 0关
void spindle_fan_control(uint8_t flag)
{
    if (flag == 1)
    {
        SPINDLE_L_FAN_PORT |= (1 << SPINDLE_L_FAN_BIT);
        SPINDLE_R_FAN_PORT &= ~(1 << SPINDLE_R_FAN_BIT);
    }
    else if(flag == 2)
    {
        SPINDLE_L_FAN_PORT &= ~(1 << SPINDLE_L_FAN_BIT);
        SPINDLE_R_FAN_PORT |= (1 << SPINDLE_R_FAN_BIT);
    }else{
        SPINDLE_L_FAN_PORT &= ~(1 << SPINDLE_L_FAN_BIT);
        SPINDLE_R_FAN_PORT &= ~(1 << SPINDLE_R_FAN_BIT);
    }
    sys.spindleFanStatus = flag;
}


// 1左开， 2右开， 0关
void coolant_control(uint8_t flag)
{
    if (flag == 1)
    {
        l_water_control(1);
        r_water_control(0);
        spray_control(1);
    }
    else if(flag == 2)
    {
        r_water_control(1);
        l_water_control(0);
        spray_control(1);
    }else{
        r_water_control(0);
        l_water_control(0);
        spray_control(0);
    }
    sys.coolingStatus = flag;
}

void coolant_close()
{
    l_water_control(0);
    r_water_control(0);
}

// 1开0关
void blow_fan_control(uint8_t flag)
{
    if (flag)
    {
        BLOW_FAN_PORT |= (1 << BLOW_FAN_BIT);
    }
    else
    {
        BLOW_FAN_PORT &= ~(1 << BLOW_FAN_BIT);
    }
    sys.blowFanStatus = flag;
}

// 1开0关
void suction_cup_control(uint8_t flag)
{
    if (flag)
    {
        SUCTION_CUP_PORT |= (1 << SUCTION_CUP_BIT);
    }
    else
    {
        SUCTION_CUP_PORT &= ~(1 << SUCTION_CUP_BIT);
    }
}

// 1开0关
void light_control(uint8_t flag)
{
    if (flag)
    {
        LIGHT_PORT |= (1 << LIGHT_BIT);
    }
    else
    {
        LIGHT_PORT &= ~(1 << LIGHT_BIT);
    }
    sys.ledStatus = flag;
}

// 1开0关
void spray_control(uint8_t flag)
{
    if (flag)
    {
        SPRAY_PORT |= (1 << SPRAY_BIT);
    }
    else
    {
        SPRAY_PORT &= ~(1 << SPRAY_BIT);
    }
}

// 1开0关
void l_water_control(uint8_t flag)
{
    if (flag)
    {
        L_WATER_PORT |= (1 << L_WATER_BIT);
    }
    else
    {
        L_WATER_PORT &= ~(1 << L_WATER_BIT);
    }
}

// 1开0关
void r_water_control(uint8_t flag)
{
    if (flag)
    {
        R_WATER_PORT |= (1 << R_WATER_BIT);
    }
    else
    {
        R_WATER_PORT &= ~(1 << R_WATER_BIT);
    }
}

// 1开0关
void outline_control(uint8_t flag)
{
    if (flag)
    {
        OUTLINE_PORT |= (1 << OUTLINE_BIT);
    }
    else
    {
        OUTLINE_PORT &= ~(1 << OUTLINE_BIT);
    }
}

// 1开0关
void camera_control(uint8_t flag)
{
    if (flag)
    {
        CAMERA_PORT |= (1 << CAMERA_BIT);
    }
    else
    {
        CAMERA_PORT &= ~(1 << CAMERA_BIT);
    }
}

// 1开0关
void rfid_ele_control(uint8_t flag)
{
    if (flag)
    {
        RFID_ELE_PORT |= (1 << RFID_ELE_BIT);
    }
    else
    {
        RFID_ELE_PORT &= ~(1 << RFID_ELE_BIT);
    }
}