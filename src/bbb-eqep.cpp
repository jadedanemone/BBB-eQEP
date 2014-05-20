#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

#include "../include/bbb-eqep.h"

eQEP::eQEP(int eQEP_address):
eQEP_address_(eQEP_address)
{
  active = false;
  eQEPFd = open("/dev/mem", O_RDWR | O_SYNC);
  if (eQEPFd < 0)
  {
    debug(0, "eQEP class: Can't open /dev/mem\n");
    return;
  }
  
  // Map the PWM memory range
  map_pwm_register();
  int offset = eQEP_address_ & (getpagesize()-1);
  position_p = (uint32_t *) ((uint8_t *)pwm_addr + QPOSCNT  + offset);
  pos_init_p = (uint32_t *) ((uint8_t *)pwm_addr + QPOSINIT + offset);
  max_pos_p  = (uint32_t *) ((uint8_t *)pwm_addr + QPOSMAX  + offset);
  
  debug(2, "eQEP successfully activated\n");
  active = true;
}

eQEP::~eQEP()
{
  active = false;
  munmap(pwm_addr, PWM_BLOCK_LENGTH);
  close(eQEPFd);
}

void eQEP::map_pwm_register()
{
  int masked_address = eQEP_address_ & ~(getpagesize()-1);
  pwm_addr = (void *) mmap(NULL, PWM_BLOCK_LENGTH,
    PROT_READ | PROT_WRITE, MAP_SHARED, eQEPFd, masked_address);
  if (pwm_addr == MAP_FAILED )
  {
    debug(0, "Memory Mapping failed for 0x%04x register\n", masked_address);
    debug(0, "ERROR: (errno %d %s)\n", errno, strerror(errno));
    return;
  }
  debug(1,
    "eQEP::eQEP() eQEP at address 0x%08x mapped\n",
    masked_address);
}

uint32_t eQEP::getPosition()
{
  return *position_p;
}

void eQEP::setPosition(uint32_t position)
{
  *position_p = position;
}

uint32_t eQEP::getPosInit()
{
  return *pos_init_p;
}

void eQEP::setPosInit(uint32_t pos_init)
{
  *pos_init_p = pos_init;
}

uint32_t eQEP::getMaxPos()
{
  return *max_pos_p;
}

void eQEP::setMaxPos(uint32_t max_pos)
{
  *max_pos_p = max_pos;
}
