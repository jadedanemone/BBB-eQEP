#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

#include "../include/bbb-eqep.h"
#include "../include/debug.h"

namespace BBB
{
eQEP::eQEP(int eQEP_address):
eQEP_address_(eQEP_address)
{
  if (eQEP_address_ < 3) {
    switch (eQEP_address_) {
    case 0:
      eQEP_address_ = eQEP0;
      break;
    case 1:
      eQEP_address_ = eQEP1;
      break;
    case 2:
      eQEP_address_ = eQEP2;
      break;
    }
  }
  active = false;
  eQEPFd = open("/dev/mem", O_RDWR | O_SYNC);
  if (eQEPFd < 0)
  {
    debug(0, "eQEP class: Can't open /dev/mem\n");
    return;
  }
  
  // Map the PWM memory range
  map_pwm_register();
  eqep_addr = pwm_addr + (eQEP_address_ & (getpagesize()-1));
  position_p = (uint32_t *) (eqep_addr + EQEP_QPOSCNT);
  
  debug(2, "eQEP successfully activated\n");
  defaultSettings();
  active = true;
}

eQEP::~eQEP()
{
  active = false;
  munmap(pwm_addr, PWM_BLOCK_LENGTH);
  close(eQEPFd);
}

void eQEP::initPWM() {
  *(uint32_t*)(pwm_addr + PWM_CLKCONFIG) |= PWM_CLKCONFIG_EQEPCLK_EN;
}

uint8_t* eQEP::getPWMSSPointer() {
  return pwm_addr;
}

uint8_t* eQEP::getEQEPPointer() {
  return eqep_addr;
}

void eQEP::defaultSettings() {
  // Set all options off
  setDecoderControl(0);
  // Enable the eQEP unit
  setControl(EQEP_QEPCTL_PHEN);
  // Set all options off
  setCaptureControl(0);
  // Set all options off
  setPositionCompareControl(0);
  // Default to no interrupts enabled.
  setInterruptEnable(0);
  // Clear all of the interrupts.
  setInterruptClear(EQEP_INT_ENABLE_ALL);
  // Clear all of the sticky bits.
  setStatus(EQEP_QEPSTS_COEF | EQEP_QEPSTS_CDEF | EQEP_QEPSTS_FIMF);
  // Set the Max Position to the maximum possible.
  setMaxPos(-1);
}

void eQEP::map_pwm_register()
{
  int masked_address = eQEP_address_ & ~(getpagesize()-1);
  pwm_addr = (uint8_t*)mmap(NULL, PWM_BLOCK_LENGTH,
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

void eQEP::setHelper(int offset, uint32_t value) {
  *(uint32_t*)(eqep_addr + offset) = value;
}
void eQEP::setHelper(int offset, uint16_t value) {
  *(uint16_t*)(eqep_addr + offset) = value;
}
uint32_t eQEP::getHelper32(int offset) {
  return *(uint32_t*)(eqep_addr + offset);
}
uint16_t eQEP::getHelper16(int offset) {
  return *(uint16_t*)(eqep_addr + offset);
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
  return getHelper32(EQEP_QPOSINIT);
}

void eQEP::setPosInit(uint32_t pos_init)
{
  setHelper(EQEP_QPOSINIT,pos_init);
}

uint32_t eQEP::getMaxPos()
{
  return getHelper32(EQEP_QPOSMAX);
}

void eQEP::setMaxPos(uint32_t max_pos)
{
  setHelper(EQEP_QPOSMAX,max_pos);
}

// eQEP Position-Compare Register
/**
 * The position-compare value in this register is compared with the position
 * counter (QPOSCNT) to generate sync output and/or interrupt on compare
 * match.
**/
uint32_t eQEP::getPositionCompare() {
  return getHelper32(EQEP_QPOSCMP);
}
/**
 * The position-compare value in this register is compared with the position
 * counter (QPOSCNT) to generate sync output and/or interrupt on compare
 * match.
 * 
 * @see getPosition()
 * @see setPosition()
 * @see setPositionCompareControl()
**/
void eQEP::setPositionCompare(uint32_t value) {
  setHelper(EQEP_QPOSCMP,value);
}

// eQEP Index Position Latch Register
/**
 * The position-counter value is latched into this register on an index event
 * as defined by the QEPCTL[IEL] bits. 
 * 
 * TODO: add @see's here for the QEPCTL[IEL] bits
**/
uint32_t eQEP::getIndexPositionLatch() {
  return getHelper32(EQEP_QPOSILAT);
}

// eQEP Strobe Position Latch Register
/**
 * The position-counter value is latched into this register on strobe event
 * as defined by the QEPCTL[SEL] bits.
**/
uint32_t eQEP::getStrobePositionLatch() {
  return getHelper32(EQEP_QPOSSLAT);
}

// eQEP Position Counter Latch Register
/**
 * The position-counter value is latched into this register on unit time out
 * event.
**/
uint32_t eQEP::getPositionCounterLatch() {
  return getHelper32(EQEP_QPOSLAT);
}

// eQEP Unit Timer Register
/**
 * This register acts as time base for unit time event generation. When this
 * timer value matches with unit time period value, unit time event is
 * generated.
**/
uint32_t eQEP::getUnitTimer() {
  return getHelper32(EQEP_QUTMR);
}
/**
 * This register acts as time base for unit time event generation. When this
 * timer value matches with unit time period value, unit time event is
 * generated.
**/
void eQEP::setUnitTimer(uint32_t value) {
  setHelper(EQEP_QUTMR,value);
}

// eQEP Unit Period Register
/**
 * This register contains the period count for unit timer to generate
 * periodic unit time events to latch the eQEP position information at
 * periodic interval and optionally to generate interrupt.
**/
uint32_t eQEP::getUnitPeriod() {
  return getHelper32(EQEP_QUPRD);
}
/**
 * This register contains the period count for unit timer to generate
 * periodic unit time events to latch the eQEP position information at
 * periodic interval and optionally to generate interrupt.
**/
void eQEP::setUnitPeriod(uint32_t value) {
  setHelper(EQEP_QUPRD, value);
}

// eQEP Watchdog Timer Register
/**
 * This register acts as time base for watch dog to detect motor stalls. When
 * this timer value matches with watch dog period value, watch dog timeout
 * interrupt is generated. This register is reset upon edge transition in
 * quadrature-clock indicating the motion.
**/
uint16_t eQEP::getWatchdogTimer() {
  return getHelper16(EQEP_QWDTMR);
}
/**
 * This register acts as time base for watch dog to detect motor stalls. When
 * this timer value matches with watch dog period value, watch dog timeout
 * interrupt is generated. This register is reset upon edge transition in
 * quadrature-clock indicating the motion.
**/
void eQEP::setWatchdogTimer(uint16_t value) {
  setHelper(EQEP_QWDTMR, value);
}

// eQEP Watchdog Period Register
/** 
 * This register contains the time-out count for the eQEP peripheral watch
 * dog timer. When the watchdog timer value matches the watchdog period
 * value, a watchdog timeout interrupt is generated.
**/
uint16_t eQEP::getWatchdogPeriod() {
  return getHelper16(EQEP_QWDPRD);
}
/** 
 * This register contains the time-out count for the eQEP peripheral watch
 * dog timer. When the watchdog timer value matches the watchdog period
 * value, a watchdog timeout interrupt is generated.
**/
void eQEP::setWatchdogPeriod(uint16_t value) {
  setHelper(EQEP_QWDPRD, value);
}

// eQEP Decoder Control Register
/**
 * Get the current eQEP Decoder control register setting
**/
uint16_t eQEP::getDecoderControl() {
  return getHelper16(EQEP_QDECCTL);
}
/**
 * Set the current eQEP Decoder control register setting
**/
void eQEP::setDecoderControl(uint16_t value) {
  setHelper(EQEP_QDECCTL, value);
}

/**
 * Position-Counter source selection.
 *
 * /param source Select from:
 * 0: Quadrature count mode
 * 1: Direction count mode
 * 2: UP count mode for frequency measurement
 * 3: DOWN count mode for frequency measurement
**/
void eQEP::positionCounterSourceSelection(int source) {
  setDecoderControl(
    // Ensure both bits are off
    (getDecoderControl() & ~(EQEP_QDECCTL_QSRC(3)))
      // Then set whichever need to be on again
      & (source << 14));
}
/**
 * Disable position-compare sync output.
**/
void eQEP::disableSyncOutput() {
  setDecoderControl(getDecoderControl() & ~EQEP_QDECCTL_SOEN);
}

/**
 * Sync output enable pin selection enum.
 * 
 * @see enableSyncOutput();
**/
//enum SOPin {
//  Index = 0, ///< Use the index pin for sync output.
//  Strobe = 1 ///< Use the strobe pin for sync output.
//};
/**
 * Enable position-compare sync output.
 * 
 * /param pin Defines the pins to sync output. Index or Strobe.
**/
void eQEP::enableSyncOutput(SOPin pin) {
  setDecoderControl(
    // Ensure the pin is off
    (getDecoderControl() & ~EQEP_QDECCTL_SOEN)
      // Then turn it back on
      & ((uint16_t)pin << 12));
}
/**
 * External clock rate. Default (2x).
**/
void eQEP::twoXClockRate() {
  setDecoderControl(getDecoderControl() & ~EQEP_QDECCTL_XCR);
}
/**
 * External clock rate. Half (1x).
**/
void eQEP::oneXClockRate() {
  setDecoderControl(getDecoderControl() | EQEP_QDECCTL_XCR);
}
/**
 * Swap quadrature clock inputs. This swaps the input to the quadrature
 * decoder, reversing the counting direction.
**/
void eQEP::swapInputs() {
  setDecoderControl(getDecoderControl() | EQEP_QDECCTL_SWAP);
}
/**
 * Disable index pulse gating option.
**/
void eQEP::disableGating() {
  setDecoderControl(getDecoderControl() & ~EQEP_QDECCTL_IGATE);
}
/**
 * Gate the index pin with strobe.
**/
void eQEP::enableGating() {
  setDecoderControl(getDecoderControl() | EQEP_QDECCTL_IGATE);
}
/**
 * Inverts eQEPA input. Useful for rectifying after using a transistor.
**/
void eQEP::invertInputA() {
  setDecoderControl(getDecoderControl() | EQEP_QDECCTL_QAP);
}
/**
 * Inverts eQEPB input. Useful for rectifying after using a transistor.
**/
void eQEP::invertInputB() {
  setDecoderControl(getDecoderControl() | EQEP_QDECCTL_QBP);
}
/**
 * Inverts Index input. Useful for rectifying after using a transistor.
**/
void eQEP::invertIndex() {
  setDecoderControl(getDecoderControl() | EQEP_QDECCTL_QIP);
}
/**
 * Inverts Strobe input. Useful for rectifying after using a transistor.
**/
void eQEP::invertStrobe() {
  setDecoderControl(getDecoderControl() | EQEP_QDECCTL_QSP);
}

// eQEP Control Register
/**
 * get eQEP Control Register
**/
uint16_t eQEP::getControl() {
  return getHelper16(EQEP_QEPCTL);
}
/**
 * set eQEP Control Register
**/
void eQEP::setControl(uint16_t value) {
  setHelper(EQEP_QEPCTL, value);
}

/**
 * Emulation control enum. Effects eQEP behavior through emulation suspensions
 * such as debugging.
 *
 * @see setEmulationControl()
**/
//enum ECB {
  /**
   * Counters and timers will stop immediately on emulation suspend
  **/
//  StopImmediately = 0,
  /**
   * Counters and timers will wait for the next event on emulation suspend
  **/
//  WaitForRollover = 1,
  /**
   * Counters and timers will continue through emulation suspend
  **/
//  NoStop = 2
//};
/**
 * Set the Emulation Control bits.
**/
void eQEP::setEmulationControl(ECB mode) {
  setControl(
    // Ensure both bits are off
    (getControl() & ~(EQEP_QEPCTL_ECB(3)))
      // Then set whichever need to be on again
      & ((uint16_t)mode << 14));
}
/**
 * Position Counter Reset Mode enum.
 *
 * @see setPositionCounterResetMode()
**/
//enum PCRM {
//  OnIndex = 0, /**< Position counter reset on an index event.*/
//  OnMax = 1,   /**< Position counter reset on the maximum position.*/
//  OnFirstIndex = 2, /**< Position counter reset on the first index event.*/
//  OnUnitTime = 3 /**< Position counter resent on a unit time event.*/
//};
/**
 * Sets the condition for reseting the position counter.
**/
void eQEP::setPositionCounterResetMode(PCRM mode) {
  setControl(
    // Ensure both bits are off
    (getControl() & ~(EQEP_QEPCTL_PCRM(3)))
      // Then set whichever need to be on again
      & ((uint16_t)mode << 12));
}
/**
 * Strobe event initialization enum.
 *
 * @see setStrobeEventInit();
**/
//enum SEI {
//  /**
//   * Does nothing (action disabled)
//  **/
//  SEIDisabled = 0,
//  /**
//   * Initializes the position counter on rising edge of QEPS signal.
//  **/
//  SEIRisingEdge = 2,
//  /**
//   * Clockwise Direction: Initializes the position counter on the rising
//   * edge of QEPS strobe
//   * Counter Clockwise Direction: Initializes the position counter on the
//   * falling edge of QEPS strobe
//  **/
//  SEIConditionalEdge = 3
//};
/**
 * Sets when a strobe event should initialize the position counter
**/
void eQEP::setStrobeEventInit(SEI mode) {
  setControl(
    // Ensure both bits are off
    (getControl() & ~(EQEP_QEPCTL_SEI(3)))
      // Then set whichever need to be on again
      & ((uint16_t)mode << 10));
}
/**
 * Index Event Initialization of position counter enum
**/
//enum IEI { 
//  /**
//   * Does nothing (action disabled)
//  **/
//  IEIDisabled = 0,
//  /**
//   * Initializes the position counter on the rising edge of the QEPI signal
//   * (QPOSCNT = QPOSINIT)
//  **/
//  IEIRisingEdge = 2,
//  /**
//   * Initializes the position counter on the falling edge of QEPI signal
//   * (QPOSCNT = QPOSINIT)
//  **/
//  IEIFallingEdge = 3
//};
/**
 * Sets when an index event should initialize the position counter
**/
void eQEP::setIndexEventInit(IEI mode) {
  setControl(
    // Ensure both bits are off
    (getControl() & ~(EQEP_QEPCTL_IEI(3)))
      // Then set whichever need to be on again
      & ((uint16_t)mode << 8));
}
/**
 * Software initialization of position counter. Call this function to reset
 * the postion counter. QPOSCNT = QPOSINIT
**/
void eQEP::resetPositionCounter() {
  setControl(getControl() | EQEP_QEPCTL_SWI);
}
/**
 * Strobe event latch of position counter
**/
//enum SEL { 
//  /**
//   * The position counter is latched on the rising edge of QEPS strobe
//   * (QPOSSLAT = POSCCNT). Latching on the falling edge can be done by
//   * inverting the strobe input using invertStrobe() (the QSP bit in the
//   * QDECCTL register.)
//   *
//   * @see invertStrobe()
//  **/
//  SELRisingEdge = 0,
//  /**
//   * Clockwise Direction: Position counter is latched on rising edge of
//   * QEPS strobe
//   * Counter Clockwise Direction: Position counter is latched on falling edge
//   * of QEPS strobe
//  **/
//  SELConditional = 1,
//};
/**
 * Set which strobe event should latch the position counter
**/
void eQEP::setStrobeEventPositionLatch(SEL mode) {
  setControl(
    // Ensure the bit is off
    (getControl() & ~(EQEP_QEPCTL_SEL))
      // Then set whichever need to be on again
      & ((uint16_t)mode << 6));
}
/**
 * Strobe event latch of position counter
**/
//enum IEL { 
//  /**
//   * Latches position counter on rising edge of the index signal
//  **/
//  IELRisingEdge = 1,
//  /**
//   * Latches position counter on falling edge of the index signal
//  **/
//  IELFallingEdge = 2,
//  /**
//   * Software index marker. Latches the position counter and quadrature
//   * direction flag on index event marker. The position counter is latched to
//   * the QPOSILAT register and the direction flag is latched in the
//   * QEPSTS[QDLF] bit. This mode is useful for software index marking.
//  **/
//  IELSoftware = 3,
//};
/**
 * Set which index event should latch the position counter
**/
void eQEP::setIndexEventPositionLatch(IEL mode) {
  setControl(
    // Ensure both bits are off
    (getControl() & ~(EQEP_QEPCTL_IEL(3)))
      // Then set whichever need to be on again
      & ((uint16_t)mode << 4));
}
/**
 * Quadrature position counter enable/software reset
 * Reset the eQEP peripheral internal operating flags/read-only registers.
 * Control/configuration registers are not disturbed by a software reset.
 * should be followed by a call to enableeQEP()
 *
 * @see enableeQEP()
**/
void eQEP::reseteQEP() {
  setControl(getControl() & ~EQEP_QEPCTL_PHEN);
}
/**
 * Quadrature position counter enable after software reset
 * @see reseteQEP()
**/
void eQEP::enableeQEP() {
  setControl(getControl() | EQEP_QEPCTL_PHEN);
}
/**
 * eQEP capture latch modes enum
**/
//enum CLM {
//  /**
//   * Latch on position counter read by CPU. Capture timer and capture period
//   * values are latched into QCTMRLAT and QCPRDLAT registers when CPU reads
//   * the QPOSCNT register.
//  **/
//  CLMCPU = 0,
//  /**
//   * Latch on unit time out. Position counter, capture timer and capture
//   * period values are latched into QPOSLAT, QCTMRLAT and QCPRDLAT registers
//   * on unit time out. Unit timer must be enabled.
//   *
//   * @see enableUnitTimer()
//  **/
//  CLMUnitTime = 1
//};
/**
 * eQEP capture latch mode
**/
void eQEP::setCaptureLatchMode(CLM mode) {
  setControl(
    // Ensure the bits is
    (getControl() & ~(EQEP_QEPCTL_QCLM))
      // Then set on again
      & ((uint16_t)mode << 2));
}
/**
 * Enables the unit timer
**/
void eQEP::enableUnitTimer() {
  setControl(
    getControl() | EQEP_QEPCTL_UTE);
}
/**
 * Disables the unit timer
**/
void eQEP::disableUnitTimer() {
  setControl(
    getControl() & ~EQEP_QEPCTL_UTE);
}
/**
 * Enables the watchdog timer
**/
void eQEP::enableWatchdogTimer() {
  setControl(
    getControl() | EQEP_QEPCTL_WDE);
}
/**
 * Disables the watchdog timer
**/
void eQEP::disableWatchdogTimer() {
  setControl(
    getControl() & ~EQEP_QEPCTL_WDE);
}

// eQEP Capture Control Register
/**
 * Get Capture Control register value.
**/
uint16_t eQEP::getCaptureControl() {
  return getHelper16(EQEP_QCAPCTL);
}
/**
 * Set Capture Control register value.
 *
 * /warning The QCAPCTL register should not be modified dynamically (such as
 * switching CAPCLK prescaling mode from QCLK/4 to QCLK/8). The capture unit
 * must be disabled before changing the prescaler. @see disableCaptureUnit()
**/
void eQEP::setCaptureControl(uint16_t value) {
  setHelper(EQEP_QCAPCTL, value);
}
/**
 * Disable eQEP Capture Unit
**/
void eQEP::disableCaptureUnit() {
  setCaptureControl(getCaptureControl() & ~EQEP_QCAPCTL_CEN);
}
/**
 * Enable eQEP Capture Unit
**/
void eQEP::enableCaptureUnit() {
  setCaptureControl(getCaptureControl() | EQEP_QCAPCTL_CEN);
}
/**
 * Capture timer clock prescaler.
 *
 * /param value The prescaler factor used in equation CAPCLK=SYSCLKOUT/2^value.
 * Should be between 0h and 7h.
 *
 * /warning The QCAPCTL register should not be modified dynamically (such as
 * switching CAPCLK prescaling mode from QCLK/4 to QCLK/8). The capture unit
 * must be disabled before changing the prescaler. @see disableCaptureUnit()
**/
void eQEP::setCaptureTimeClockPrescaler(int value) {
  setCaptureControl(
    (getCaptureControl() & ~EQEP_QCAPCTL_CCPS(7))
      | (EQEP_QCAPCTL_CCPS(7) & value));
}
/**
 * Unit position event prescaler.
 *
 * /param value The prescaler factor used in equation UPEVNT=QCLK/2^value.
 * Should be between 0h and Bh
 *
 * /warning The QCAPCTL register should not be modified dynamically (such as
 * switching CAPCLK prescaling mode from QCLK/4 to QCLK/8). The capture unit
 * must be disabled before changing the prescaler. @see disableCaptureUnit()
**/
void eQEP::setPositionEventPrescaler(int value) {
  setCaptureControl(
    (getCaptureControl() & ~EQEP_QCAPCTL_UPPS(0xF))
      | (EQEP_QCAPCTL_UPPS(0xF) & value));
}

// eQEP Position-Compare Control Register
/**
 * Get Position Compare Control register value.
 * The eQEP peripheral includes a position-compare unit that is used to
 * generate a sync output and/or interrupt on a position-compare match.
 * The position-compare (QPOSCMP) register is shadowed and shadow mode can be
 * enabled or disabled using enablePositionCompareShadow() and
 * disablePositionCompareShadow() (the QPOSCTL[PSSHDW] bit). If the shadow
 * mode is not enabled, the CPU writes directly to the active position compare
 * register.
 *
 * In shadow mode, you can configure the position-compare unit
 * (QPOSCTL[PCLOAD]) to load the shadow register value into the active
 * register on the following events and to generate the position-compare
 * ready (QFLG[PCR]) interrupt after loading.
 * * Load on compare match
 * * Load on position-counter zero event
 *
 * The position-compare match (QFLG[PCM]) is set when the position-counter
 * value (QPOSCNT) matches with the active position-compare register
 * (QPOSCMP) and the position-compare sync output of the programmable pulse
 * width is generated on compare match to trigger an external device.
**/
uint16_t eQEP::getPositionCompareControl() {
  return getHelper16(EQEP_QPOSCTL);
}
/**
 * Set Position Compare Control register value.
**/
void eQEP::setPositionCompareControl(uint16_t value) {
  setHelper(EQEP_QPOSCTL, value);
}
/**
 * Disable Position Compare Shadow. Load Immediate
**/
void eQEP::disablePositionCompareShadow() {
  setPositionCompareControl(getPositionCompareControl() & ~EQEP_QPOSCTL_PCSHDW);
}
/**
 * Enable Position Compare Shadow
**/
void eQEP::enablePositionCompareShadow() {
  setPositionCompareControl(getPositionCompareControl() | EQEP_QPOSCTL_PCSHDW);
}
/**
 * Position Compare shadow load mode selection enum
**/
//enum PCLOAD {
//  /**
//   * Load on Position == 0
//  **/
//  PCLOADCountEq0 = 0,
//  /**
//   * Load on Position == Position Compare register
//   *
//   * @see setPositionCompare()
//  **/
//  PCLOADCountEqPosCmp = 1
//};
/**
 * Set position-compare shadow load mode
**/
void eQEP::setPositionCompareShadowLoadMode(PCLOAD mode) {
  setPositionCompareControl(
    (getPositionCompareControl() & ~EQEP_QPOSCTL_PCLOAD)
      | ((uint16_t)mode & EQEP_QPOSCTL_PCLOAD));
}
/**
 * Polarity of sync output enum
 *
 * @see setPositionCompareSyncOutput()
**/
//enum PCPOL {
//  /**
//   * Active high
//  **/
//  PCPOLActiveHigh = 0,
//  /**
//   * Active low
//  **/
//  PCPOLActiveLow = 1
//};
/**
 * Polarity of sync output
**/
void eQEP::setPositionCompareSyncOutput(PCPOL mode) {
  setPositionCompareControl(
    (getPositionCompareControl() & ~EQEP_QPOSCTL_PCPOL)
      | ((uint16_t)mode & EQEP_QPOSCTL_PCPOL));
}
/**
 * Enable the Position Compare Unit
**/
void eQEP::enablePositionCompareUnit() {
  setPositionCompareControl(getPositionCompareControl() | EQEP_QPOSCTL_PCE);
}
/**
 * Disable the Position Compare Unit
**/
void eQEP::disablePositionCompareUnit() {
  setPositionCompareControl(getPositionCompareControl() & ~EQEP_QPOSCTL_PCE);
}
/**
 * Select-position-compare sync output pulse width.
 *
 * /param value The prescaler factor used in equation 
 * (value+1) * 4 * SYSCLKOUT cycles.
 * Should be between 0h and FFFh.
**/
void eQEP::setPositionCompareSyncOutputPulseWidth(uint16_t value) {
  setPositionCompareControl(
    (getPositionCompareControl() & ~EQEP_QPOSCTL_PCSPW(0xFFF))
      | (EQEP_QPOSCTL_PCSPW(0xFFF) & value));
}

// eQEP Interrupt Enable Register
/**
 * Get the current status of the interrupt enable bits
**/
uint16_t eQEP::getInterruptEnable() {
  return getHelper16(EQEP_QEINT);
}
/**
 * Turn on interrupts by passing the correct bits
**/
void eQEP::setInterruptEnable(uint16_t value) {
  setHelper(EQEP_QEINT, value);
}
/**
 * Enable Unit Timeout Interrupt
**/
void eQEP::enableUnitTimeoutInterrupt() {
  setInterruptEnable(getInterruptEnable() | EQEP_INT_UTO);
}
/**
 * Disable Unit Timeout Interrupt
**/
void eQEP::disableUnitTimeoutInterrupt() {
  setInterruptEnable(getInterruptEnable() & ~EQEP_INT_UTO);
}
/**
 * Enable Index Event Latch Interrupt
**/
void eQEP::enableIndexEventLatchInterrupt() {
  setInterruptEnable(getInterruptEnable() | EQEP_INT_IEL);
}
/**
 * Disable Index Event Latch Interrupt
**/
void eQEP::disableIndexEventLatchInterrupt() {
  setInterruptEnable(getInterruptEnable() & ~EQEP_INT_IEL);
}
/**
 * Enable Strobe Event Latch Interrupt
**/
void eQEP::enableStrobeEventLatchInterrupt() {
  setInterruptEnable(getInterruptEnable() | EQEP_INT_SEL);
}
/**
 * Disable Strobe Event Latch Interrupt
**/
void eQEP::disableStrobeEventLatchInterrupt() {
  setInterruptEnable(getInterruptEnable() & ~EQEP_INT_SEL);
}
/**
 * Enable Position Compare Match Interrupt
**/
void eQEP::enablePositionCompareMatchInterrupt() {
  setInterruptEnable(getInterruptEnable() | EQEP_INT_PCM);
}
/**
 * Disable Position Compare Match Interrupt
**/
void eQEP::disablePositionCompareMatchInterrupt() {
  setInterruptEnable(getInterruptEnable() & ~EQEP_INT_PCM);
}
/**
 * Enable Position Compare Ready Interrupt
**/
void eQEP::enablePositionCompareReadyInterrupt() {
  setInterruptEnable(getInterruptEnable() | EQEP_INT_PCR);
}
/**
 * Disable Position Compare Ready Interrupt
**/
void eQEP::disablePositionCompareReadyInterrupt() {
  setInterruptEnable(getInterruptEnable() & ~EQEP_INT_PCR);
}
/**
 * Enable Position Counter Overflow Interrupt
**/
void eQEP::enablePositionCounterOverflowInterrupt() {
  setInterruptEnable(getInterruptEnable() | EQEP_INT_PCO);
}
/**
 * Disable Position Counter Overflow Interrupt
**/
void eQEP::disablePositionCounterOverflowInterrupt() {
  setInterruptEnable(getInterruptEnable() & ~EQEP_INT_PCO);
}
/**
 * Enable Position Counter Underflow Interrupt
**/
void eQEP::enablePositionCounterUnderflowInterrupt() {
  setInterruptEnable(getInterruptEnable() | EQEP_INT_PCU);
}
/**
 * Disable Position Counter Underflow Interrupt
**/
void eQEP::disablePositionCounterUnderflowInterrupt() {
  setInterruptEnable(getInterruptEnable() & ~EQEP_INT_PCU);
}
/**
 * Enable Watchdog Time Out Interrupt
**/
void eQEP::enableWatchdogTimeOutInterrupt() {
  setInterruptEnable(getInterruptEnable() | EQEP_INT_WTO);
}
/**
 * Disable Watchdog Time Out Interrupt
**/
void eQEP::disableWatchdogTimeOutInterrupt() {
  setInterruptEnable(getInterruptEnable() & ~EQEP_INT_WTO);
}
/**
 * Enable Quadrature Direction Change Interrupt
**/
void eQEP::enableQuadratureDirectionChangeInterrupt() {
  setInterruptEnable(getInterruptEnable() | EQEP_INT_QDC);
}
/**
 * Disable Quadrature Direction Change Interrupt
**/
void eQEP::disableQuadratureDirectionChangeInterrupt() {
  setInterruptEnable(getInterruptEnable() & ~EQEP_INT_QDC);
}
/**
 * Enable Quadrature Phase Error Interrupt
**/
void eQEP::enableQuadraturePhaseErrorInterrupt() {
  setInterruptEnable(getInterruptEnable() | EQEP_INT_PHE);
}
/**
 * Disable Unit Timeout Interrupt
**/
void eQEP::disableQuadraturePhaseErrorInterrupt() {
  setInterruptEnable(getInterruptEnable() & ~EQEP_INT_PHE);
}
/**
 * Enable Position Counter Error Interrupt
**/
void eQEP::enablePositionCounterErrorInterrupt() {
  setInterruptEnable(getInterruptEnable() | EQEP_INT_PCE);
}
/**
 * Disable Position Counter Error Interrupt
**/
void eQEP::disablePositionCounterErrorInterrupt() {
  setInterruptEnable(getInterruptEnable() & ~EQEP_INT_PCE);
}

// eQEP Interrupt Flag Register
/**
 * Get the status of all of the interrupt flags.
**/
uint16_t eQEP::getInterruptFlag() {
  return getHelper16(EQEP_QFLG);
}
/**
 * Unit Time Out Interrupt flag.
 *
 * \return Set by eQEP unit timer period match.
**/
bool eQEP::getUnitTimeoutInterruptFlag() {
  return getInterruptFlag() & EQEP_INT_UTO;
}
/**
 * Index Event Latch Interrupt flag.
 *
 * \return True after latching the QPOSCNT to QPOSILAT.
**/
bool eQEP::getIndexEventLatchInterruptFlag() {
  return getInterruptFlag() & EQEP_INT_IEL;
}
/**
 * Strobe Event Latch Interrupt flag.
 *
 * \return True after latching the QPOSCNT to QPASSLAT.
**/
bool eQEP::getStrobeEventLatchInterruptFlag() {
  return getInterruptFlag() & EQEP_INT_SEL;
}
/**
 * Position Compare Match Interrupt flag.
 *
 * \return True after position compare match.
**/
bool eQEP::getPositionCompareMatchInterruptFlag() {
  return getInterruptFlag() & EQEP_INT_PCM;
}
/**
 * Position Compare Ready Interrupt flag.
 *
 * \return True after transferring the shadow register value to the active
 *         position compare register.
**/
bool eQEP::getPositionCompareReadyInterruptFlag() {
  return getInterruptFlag() & EQEP_INT_PCR;
}
/**
 * Position Counter Overflow Interrupt flag.
 *
 * \return True after position counter overflow.
**/
bool eQEP::getPositionCounterOverflowInterruptFlag() {
  return getInterruptFlag() & EQEP_INT_PCO;
}
/**
 * Position Counter Underflow Interrupt flag.
 *
 * \return True after position counter underflow.
**/
bool eQEP::getPositionCounterUnderflowInterruptFlag() {
  return getInterruptFlag() & EQEP_INT_PCU;
}
/**
 * Watchdog Time Out Interrupt flag.
 *
 * \return True after watch dog timeout.
**/
bool eQEP::getWatchdogTimeOutInterruptFlag() {
  return getInterruptFlag() & EQEP_INT_WTO;
}
/**
 * Quadrature Direction Change Interrupt flag.
 *
 * \return True after change in direction.
**/
bool eQEP::getQuadratureDirectionChangeInterruptFlag() {
  return getInterruptFlag() & EQEP_INT_QDC;
}
/**
 * Quadrature Phase Error Interrupt flag.
 *
 * \return True on simultaneous transition of QEPA and QEPB.
**/
bool eQEP::getQuadraturePhaseErrorInterruptFlag() {
  return getInterruptFlag() & EQEP_INT_PHE;
}
/**
 * Position Counter Error Interrupt Flag.
 *
 * \return True on position counter error.
**/
bool eQEP::getPositionCounterErrorInterruptFlag() {
  return getInterruptFlag() & EQEP_INT_PCE;
}
/**
 * Global Interrupt Status Flag
 *
 * \return True when an interrupt has been generated.
**/
bool eQEP::getGlobalInterruptStatusFlag() {
  return getInterruptFlag() & EQEP_INT_INT;
}

// eQEP Interrupt Clear Register
/**
 * Get the interrupts to be cleared. Not very useful.
**/
uint16_t eQEP::getInterruptClear() {
  return getHelper16(EQEP_QCLR);
}
/**
 * Clear triggered interrupts
 *
 * \param Set bits indicating the interrupts to clear
**/
void eQEP::setInterruptClear(uint16_t value) {
  setHelper(EQEP_QCLR, value);
}
/**
 * Clear all of the interrupts
**/
void eQEP::clearInterrupts() {
  setInterruptClear(EQEP_INT_ENABLE_ALL);
}
/**
 * Clear the Unit Time Out Interrupt flag.
**/
void eQEP::clearUnitTimeoutInterruptFlag() {
  setInterruptClear(EQEP_INT_UTO);
}
/**
 * Clear the Index Event Latch Interrupt
**/
void eQEP::clearIndexEventLatchInterruptFlag() {
  setInterruptClear(EQEP_INT_IEL);
}
/**
 * Clear the Strobe Event Latch Interrupt
**/
void eQEP::clearStrobeEventLatchInterruptFlag() {
  setInterruptClear(EQEP_INT_SEL);
}
/**
 * Clear the Position Compare Match Interrupt
**/
void eQEP::clearPositionCompareMatchInterruptFlag() {
  setInterruptClear(EQEP_INT_PCM);
}
/**
 * Clear the Position Compare Ready Interrupt
**/
void eQEP::clearPositionCompareReadyInterruptFlag() {
  setInterruptClear(EQEP_INT_PCR);
}
/**
 * Clear the Position Counter Overflow Interrupt
**/
void eQEP::clearPositionCounterOverflowInterruptFlag() {
  setInterruptClear(EQEP_INT_PCO);
}
/**
 * Clear the Position Counter Underflow Interrupt
**/
void eQEP::clearPositionCounterUnderflowInterruptFlag() {
  setInterruptClear(EQEP_INT_PCU);
}
/**
 * Clear the Watchdog Time Out Interrupt
**/
void eQEP::clearWatchdogTimeOutInterruptFlag() {
  setInterruptClear(EQEP_INT_WTO);
}
/**
 * Clear the Quadrature Direction Change Interrupt
**/
void eQEP::clearQuadratureDirectionChangeInterruptFlag() {
  setInterruptClear(EQEP_INT_QDC);
}
/**
 * Clear the Quadrature Phase Error Interrupt
**/
void eQEP::clearQuadraturePhaseErrorInterruptFlag() {
  setInterruptClear(EQEP_INT_PHE);
}
/**
 * Clear the Position Counter Error Interrupt Flag
**/
void eQEP::clearPositionCounterErrorInterruptFlag() {
  setInterruptClear(EQEP_INT_PCE);
}
/**
 * Clear the Global Interrupt Status Flag. Clears the interrupt flag and
 * enables further interrupts to be generated if an event flags is set to 1.
**/
void eQEP::clearGlobalInterruptStatusFlag() {
  setInterruptClear(EQEP_INT_INT);
}

// eQEP Interrupt Force Register
/**
 * Read forced interrupts. Not much value here.
**/
uint16_t eQEP::getInterruptForce() {
  return getHelper16(EQEP_QFRC);
}
/**
 * Force interrupts to be triggered.
**/
void eQEP::setInterruptForce(uint16_t value) {
  setHelper(EQEP_QFRC, value);
}
/**
 * Force the Unit Time Out Interrupt flag.
**/
void eQEP::forceUnitTimeoutInterruptFlag() {
  setInterruptForce(EQEP_INT_UTO);
}
/**
 * Force the Index Event Latch Interrupt
**/
void eQEP::forceIndexEventLatchInterruptFlag() {
  setInterruptForce(EQEP_INT_IEL);
}
/**
 * Force the Strobe Event Latch Interrupt
**/
void eQEP::forceStrobeEventLatchInterruptFlag() {
  setInterruptForce(EQEP_INT_SEL);
}
/**
 * Force the Position Compare Match Interrupt
**/
void eQEP::forcePositionCompareMatchInterruptFlag() {
  setInterruptForce(EQEP_INT_PCM);
}
/**
 * Force the Position Compare Ready Interrupt
**/
void eQEP::forcePositionCompareReadyInterruptFlag() {
  setInterruptForce(EQEP_INT_PCR);
}
/**
 * Force the Position Counter Overflow Interrupt
**/
void eQEP::forcePositionCounterOverflowInterruptFlag() {
  setInterruptForce(EQEP_INT_PCO);
}
/**
 * Force the Position Counter Underflow Interrupt
**/
void eQEP::forcePositionCounterUnderflowInterruptFlag() {
  setInterruptForce(EQEP_INT_PCU);
}
/**
 * Force the Watchdog Time Out Interrupt
**/
void eQEP::forceWatchdogTimeOutInterruptFlag() {
  setInterruptForce(EQEP_INT_WTO);
}
/**
 * Force the Quadrature Direction Change Interrupt
**/
void eQEP::forceQuadratureDirectionChangeInterruptFlag() {
  setInterruptForce(EQEP_INT_QDC);
}
/**
 * Force the Quadrature Phase Error Interrupt
**/
void eQEP::forceQuadraturePhaseErrorInterruptFlag() {
  setInterruptForce(EQEP_INT_PHE);
}
/**
 * Force the Position Counter Error Interrupt Flag
**/
void eQEP::forcePositionCounterErrorInterruptFlag() {
  setInterruptForce(EQEP_INT_PCE);
}

// eQEP Status Register
/**
 * Get the status of the eQEP functions.
**/
uint16_t eQEP::getStatus() {
  return getHelper16(EQEP_QEPSTS);
}
/**
 * Set the status of the eQEP functions. Only some bits may be set.
**/
void eQEP::setStatus(uint16_t value) {
  setHelper(EQEP_QEPSTS, value);
}
/**
 * Unit position event flag
 *
 * \return False on no unit position event detected.
 *         True on unit position event detected.
**/
bool eQEP::getUnitPositionEventFlag() {
  return getStatus() & EQEP_QEPSTS_UPEVNT;
}
/**
 * Clear the Unit Position Event Flag.
**/
void eQEP::clearUnitPositionEventFlag(){
  setStatus(getStatus() | EQEP_QEPSTS_UPEVNT);
}
/**
 * Direction on the first index marker. Status of the direction is latched on
 * the first index event marker.
 *
 * \return false for Counter-clockwise rotation (or reverse movement) on the
 *           first index event
 *         true Clockwise rotation (or forward movement) on the first index
 *           event
**/
bool eQEP::getFirstIndexDirection() {
  return getStatus() & EQEP_QEPSTS_FDF;
}
/**
 * Quadrature direction flag
 *
 * \return false for Counter-clockwise rotation (or reverse movement)
 *         true for Clockwise rotation (or forward movement)
**/
bool eQEP::getQuadratureDirection() {
  return getStatus() & EQEP_QEPSTS_QDF;
}
/**
 * eQEP direction latch flag. Status of direction is latched on every index
 * event marker.
 *
 * \return false for Counter-clockwise rotation (or reverse movement)
 *         true for Clockwise rotation (or forward movement)
**/
bool eQEP::getQuadratureDirectionLatch() {
  return getStatus() & EQEP_QEPSTS_QDLF;
}
/**
 * Capture overflow error flag. Sticky bit, cleared by calling
 * clearCaptureOverflowErrorFlag().
 *
 * \return true for Overflow occurred in eQEP Capture timer
**/
bool eQEP::getCaptureOverflowErrorFlag() {
  return getStatus() & EQEP_QEPSTS_COEF;
}
/**
 * Clear the capture overflow error flag.
**/
void eQEP::clearCaptureOverflowErrorFlag() {
  setStatus(getStatus() | EQEP_QEPSTS_COEF);
}
/**
 * Capture direction error flag. Sticky bit, cleared by calling
 * clearCaptureDirectionErrorFlag().
 *
 * \return true for Direction change occurred between the capture position
 *         event.
**/
bool eQEP::getCaptureDirectionErrorFlag() {
  return getStatus() & EQEP_QEPSTS_CDEF;
}
/**
 * Clear the Capture Direction Error Flag.
**/
void eQEP::clearCaptureDirectionErrorFlag() {
  setStatus(getStatus() | EQEP_QEPSTS_CDEF);
}
/**
 * First index marker flag. Sticky bit, cleared by calling
 * clearFirstIndexMarkerFlag(). Set by first occurrence of index pulse
**/
bool eQEP::getFirstIndexMarkerFlag() {
  return getStatus() & EQEP_QEPSTS_FIMF;
}
/**
 * Clear the first index marker flag allowing it to flip on next index event.
**/
void eQEP::clearFirstIndexMarkerFlag() {
  setStatus(getStatus() | EQEP_QEPSTS_FIMF);
}
/**
 * Position counter error flag. This bit is not sticky and it is updated for
 * every index event.
 *
 * \return false No error occurred during the last index transition.
 *         true Position counter error.
**/
bool eQEP::getPositionCounterErrorFlag() {
  return getStatus() & EQEP_QEPSTS_PCEF;
}

// eQEP Capture Timer Register
/**
 * This register provides time base for edge capture unit.
**/
uint16_t eQEP::getCaptureTimer() {
  return getHelper16(EQEP_QCTMR);
}
/**
 * This register provides time base for edge capture unit.
 *
 * @see setCaptureLatchMode()
 * @see setCaptureTimeClockPrescaler()
 * @see enableCaptureUnit()
**/
void eQEP::setCaptureTimer(uint16_t value) {
  setHelper(EQEP_QCTMR, value);
}

// eQEP Capture Period Register
/**
 * This register holds the period count value between the last successive
 * eQEP position events
 *
 * @see setCaptureLatchMode()
 * @see setCaptureTimeClockPrescaler()
 * @see enableCaptureUnit()
**/
uint16_t eQEP::getCapturePeriod() {
  return getHelper16(EQEP_QCTMR);
}
/**
 * This register holds the period count value between the last successive
 * eQEP position events
 *
 * @see setCaptureLatchMode()
 * @see setCaptureTimeClockPrescaler()
 * @see enableCaptureUnit()
**/
void eQEP::setCapturePeriod(uint16_t value) {
  setHelper(EQEP_QCTMR, value);
}
// eQEP Capture Timer Latch Register
/**
 * The eQEP capture timer value can be latched into this register on two
 * events viz., unit timeout event, reading the eQEP position counter.
 *
 * @see setCaptureLatchMode()
 * @see setCaptureTimeClockPrescaler()
 * @see enableCaptureUnit()
**/
uint16_t eQEP::getCaptureTimerLatch() {
  return getHelper16(EQEP_QCTMRLAT);
}
// eQEP Capture Period Latch Register
/**
 * eQEP capture period value can be latched into this register on two events
 * viz., unit timeout event, reading the eQEP position counter.
 *
 * @see setCaptureLatchMode()
 * @see setCaptureTimeClockPrescaler()
 * @see enableCaptureUnit()
**/
uint16_t eQEP::getCapturePeriodLatch() {
  return getHelper16(EQEP_QCPRDLAT);
}
/**
 * eQEP capture period value can be latched into this register on two events
 * viz., unit timeout event, reading the eQEP position counter.
 *
 * @see setCaptureLatchMode()
 * @see setCaptureTimeClockPrescaler()
 * @see enableCaptureUnit()
**/
void eQEP::setCapturePeriodLatch(uint16_t value) {
  setHelper(EQEP_QCPRDLAT, value);
}

// eQEP Revision ID Register
/**
 * eQEP revision ID
**/
uint32_t eQEP::getRevisionID() {
  return getHelper32(EQEP_REVID);
}

} /* BBB */
