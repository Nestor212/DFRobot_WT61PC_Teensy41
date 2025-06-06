/*!
 * @file  DFRobot_WT61PC.cpp
 * @brief  Realize the basic structure of DFRobot_WT61PC sensor class 
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence  The MIT License (MIT)
 * @author  huyujie(yujie.hu@dfrobot.com)
 * @version  V1.0
 * @date  2023-07-10
 * @url  https://github.com/DFRobot/DFRobot_WT61PC
 */
#include "DFRobot_WT61PC.h"

DFRobot_WT61PC::DFRobot_WT61PC(Stream *s)
{
  _s = s;
}

size_t DFRobot_WT61PC::readN(uint8_t *buf, size_t len)
{
  size_t offset = 0, left = len;
  long curr = millis();
  while (left) {
    if (_s -> available()) {
      buf[offset++] = _s->read();
      left--;
    }
    if (millis() - curr > TIMEOUT) {
      break;
    }
  }
  return offset;
}

bool DFRobot_WT61PC::recvData(uint8_t *buf, uint8_t header)
{
  long startTime = millis();
  uint8_t ch;

  while ((millis() - startTime) < TIMEOUT) {
    // Shift in one byte until we get HEADER55
    if (readN(&ch, 1) != 1) continue;
    if (ch != HEADER55) continue;

    // Read the next byte, check for the expected header
    if (readN(&ch, 1) != 1) continue;
    if (ch != header) continue;

    // Got HEADER55 + correct header byte, read the next 9 + 1 checksum
    buf[0] = HEADER55;
    buf[1] = header;

    if (readN(&buf[2], 9) != 9) continue;

    uint8_t checksum = buf[10];
    if (getCS(buf) == checksum) {
      return true; // Packet is valid
    } else {
      // Skip this packet and continue searching
      continue;
    }
  }
  return false; // Timeout or failure
}


uint8_t DFRobot_WT61PC::getCS(uint8_t *buf)
{
  uint8_t cs = 0;
  for (int i = 0; i < 10; i++) {
    cs = cs + buf[i];
  }
  return cs;
}

bool DFRobot_WT61PC::available(void)
{
  bool ret = false;
  if (recvData(receivedAccData , HEADERACC) && recvData(receivedGyroData , HEADERGYRO) && recvData(receivedAngleData , HEADERANGLE)) {
    ret = true;
    getAcc(receivedAccData);
    getGyro(receivedGyroData);
    getAngle(receivedAngleData);
  }
  return ret;
}

void DFRobot_WT61PC::getAcc(uint8_t *buf)
{
  int16_t x = (int16_t)((buf[WT61PC_XH] << 8) | buf[WT61PC_XL]);
  int16_t y = (int16_t)((buf[WT61PC_YH] << 8) | buf[WT61PC_YL]);
  int16_t z = (int16_t)((buf[WT61PC_ZH] << 8) | buf[WT61PC_ZL]);

  Acc.X = x / 32768.0 * 16.0 * 9.8;
  Acc.Y = y / 32768.0 * 16.0 * 9.8;
  Acc.Z = z / 32768.0 * 16.0 * 9.8;
}


void DFRobot_WT61PC::getGyro(uint8_t *buf)
{
  int16_t x = (int16_t)((buf[WT61PC_XH] << 8) | buf[WT61PC_XL]);
  int16_t y = (int16_t)((buf[WT61PC_YH] << 8) | buf[WT61PC_YL]);
  int16_t z = (int16_t)((buf[WT61PC_ZH] << 8) | buf[WT61PC_ZL]);

  Gyro.X = x / 32768.0 * 2000.0;
  Gyro.Y = y / 32768.0 * 2000.0;
  Gyro.Z = z / 32768.0 * 2000.0;
}

void DFRobot_WT61PC::getAngle(uint8_t *buf)
{
  int16_t x = (int16_t)((buf[WT61PC_XH] << 8) | buf[WT61PC_XL]);
  int16_t y = (int16_t)((buf[WT61PC_YH] << 8) | buf[WT61PC_YL]);
  int16_t z = (int16_t)((buf[WT61PC_ZH] << 8) | buf[WT61PC_ZL]);

  Angle.X = x / 32768.0 * 180.0;
  Angle.Y = y / 32768.0 * 180.0;
  Angle.Z = z / 32768.0 * 180.0;
}

void DFRobot_WT61PC::modifyFrequency(uint8_t frequency)
{
  Cmd[3] = frequency;
  _s->write(Cmd, 5);

}
