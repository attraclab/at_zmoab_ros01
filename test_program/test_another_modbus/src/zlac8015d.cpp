#include "Arduino.h"
#include "zlacd8015.h"

zlac8015d::zlac8015d(){

}

// void zlac8015d::set_modbus(const int RX_PIN, const int TX_PIN, const int DE_PIN){
void zlac8015d::set_modbus(ModbusRTU *mb){
	/**
	 * Copy ModbusMaster object (node) into class
	 *
	 * */
    
    modbus = mb;
}

uint8_t zlac8015d::set_mode(uint8_t mode){
    uint8_t res;
    res = modbus->writeSingleHoldingRegister(1, OPR_MODE, mode);
    return res;
}

uint8_t zlac8015d::enable_motor(){
    uint8_t res;
    res = modbus->writeSingleHoldingRegister(1, CONTROL_REG, ENABLE);
    return res;
}

uint8_t zlac8015d::disable_motor(){
    uint8_t res;
    res = modbus->writeSingleHoldingRegister(1, CONTROL_REG, DOWN_TIME);
    return res;
}

uint8_t zlac8015d::set_accel_time(uint16_t L_ms, uint16_t R_ms){
	/**
	 * Set acceleration time of each wheel
	 * unit is in milliseconds
	 * */
    uint8_t res;
	L_ms = constrain(L_ms, 0, 32767);
	R_ms = constrain(R_ms, 0, 32767);

    uint16_t buffer[2] = {L_ms, R_ms};
	res = modbus->writeMultipleHoldingRegisters(1, L_ACL_TIME, buffer, 2);
	return res;
}

uint8_t zlac8015d::set_decel_time(uint16_t L_ms, uint16_t R_ms){
	/**
	 * Set acceleration time of each wheel
	 * unit is in milliseconds
	 * */
    uint8_t res;
	L_ms = constrain(L_ms, 0, 32767);
	R_ms = constrain(R_ms, 0, 32767);

    uint16_t buffer[2] = {L_ms, R_ms};
	res = modbus->writeMultipleHoldingRegisters(1, L_DCL_TIME, buffer, 2);
	return res;
}

uint8_t zlac8015d::set_rpm(int16_t L_rpm, int16_t R_rpm){
    uint8_t res;
    L_rpm = constrain(L_rpm, -3000, 3000);
	R_rpm = constrain(-R_rpm, -3000, 3000);
    uint16_t buffer[2] = {(uint16_t)L_rpm, (uint16_t)R_rpm};
	res = modbus->writeMultipleHoldingRegisters(1, L_CMD_RPM, buffer, 2);
	return res;

}
