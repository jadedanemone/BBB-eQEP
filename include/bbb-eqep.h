/*! \file bbb-eqep.h
 * \brief Beaglebone Black eQEP header file.
 * Contains all of the method declarations for class eQEP.
**/

#ifndef BBB_EQEP_H
#define BBB_EQEP_H

#include <sys/mman.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>

#define EQEP_QPOSCNT	0x00
#define EQEP_QPOSINIT	0x04
#define EQEP_QPOSMAX	0x08
#define EQEP_QPOSCMP	0x0c
#define EQEP_QPOSILAT	0x10
#define EQEP_QPOSSLAT	0x14
#define EQEP_QPOSLAT	0x18
#define EQEP_QUTMR	0x1c
#define EQEP_QUPRD	0x20
#define EQEP_QWDTMR	0x24
#define EQEP_QWDPRD	0x26
#define EQEP_QDECCTL	0x28
#define		EQEP_QDECCTL_QSRC(x)	(((x) & 0x3) << 14)
#define		EQEP_QDECCTL_SOEN	(1 << 13)
#define		EQEP_QDECCTL_SPSEL	(1 << 12)
#define		EQEP_QDECCTL_XCR	(1 << 11)
#define		EQEP_QDECCTL_SWAP	(1 << 10)
#define		EQEP_QDECCTL_IGATE	(1 << 9)
#define		EQEP_QDECCTL_QAP	(1 << 8)
#define		EQEP_QDECCTL_QBP	(1 << 7)
#define		EQEP_QDECCTL_QIP	(1 << 6)
#define		EQEP_QDECCTL_QSP	(1 << 5)
#define EQEP_QEPCTL	0x2a
#define		EQEP_QEPCTL_PCRM(x)	(((x) & 0x3) << 12)
#define		EQEP_QEPCTL_SEI(x)	(((x) & 0x3) << 10)
#define		EQEP_QEPCTL_IEI(x)	(((x) & 0x3) << 8)
#define		EQEP_QEPCTL_SWI		(1 << 7)
#define		EQEP_QEPCTL_SEL		(1 << 6)
#define		EQEP_QEPCTL_IEL(x)	(((x) & 0x3) << 4)
#define		EQEP_QEPCTL_PHEN	(1 << 3)
#define		EQEP_QEPCTL_QCLM	(1 << 2)
#define		EQEP_QEPCTL_UTE		(1 << 1)
#define		EQEP_QEPCTL_WDE		(1 << 0)
#define EQEP_QCAPCTL	0x2c
#define EQEP_QPOSCTL	0x2e
#define EQEP_QEINT	0x30
#define EQEP_QFLG	0x32
#define EQEP_QCLR	0x34
#define EQEP_QFRC	0x36
#define		EQEP_INT_UTO		(1 << 11) /* Same for all intr regs */
#define		EQEP_INT_IEL		(1 << 10)
#define		EQEP_INT_SEL		(1 << 9)
#define		EQEP_INT_PCM		(1 << 8)
#define		EQEP_INT_PCR		(1 << 7)
#define		EQEP_INT_PCO		(1 << 6)
#define		EQEP_INT_PCU		(1 << 5)
#define		EQEP_INT_WTO		(1 << 4)
#define		EQEP_INT_QDC		(1 << 3)
#define		EQEP_INT_PHE		(1 << 2)
#define		EQEP_INT_PCE		(1 << 1)
#define		EQEP_INT_INT		(1 << 0)
#define		EQEP_INT_ENABLE_ALL	(EQEP_INT_UTO | EQEP_INT_IEL \
			| EQEP_INT_SEL | EQEP_INT_PCM | EQEP_INT_PCR \
			| EQEP_INT_PCO | EQEP_INT_PCU | EQEP_INT_WTO \
			| EQEP_INT_QDC | EQEP_INT_PHE | EQEP_INT_PCE)
#define		EQEP_INT_MASK		(EQEP_INT_ENABLE_ALL | EQEP_INT_INT)
#define EQEP_QEPSTS	0x38
#define		EQEP_QEPSTS_UPEVNT	(1 << 7)
#define		EQEP_QEPSTS_FDF		(1 << 6)
#define		EQEP_QEPSTS_QDF		(1 << 5)
#define		EQEP_QEPSTS_QDLF	(1 << 4)
#define		EQEP_QEPSTS_COEF	(1 << 3)
#define		EQEP_QEPSTS_CDEF	(1 << 2)
#define		EQEP_QEPSTS_FIMF	(1 << 1)
#define		EQEP_QEPSTS_PCEF	(1 << 0)
#define EQEP_QCTMR	0x3a
#define EQEP_QCPRD	0x3c
#define EQEP_QCTMRLAT	0x3e
#define EQEP_QCPRDLAT	0x40
#define EQEP_REVID	0x5c

#define EQEP_INPUT_DEV_PHYS_SIZE	32

/** Beaglebone Black eQEP control and access class
 * This class allows for easy setup of all of the TI Sitara's eQEP registers.
 * Also, all of the registers can be easy accessed through this class.
**/

class eQEP
{
private:
  int eQEP_address_; /**< The start address of the eQEP memory section */
  int eQEPFd; /**< File descriptor to a mmap of the eQEP memory section */
  
  bool active; /**< Set to true when the eQEP memory is properly mapped and available */
  
  void *pwm_addr; /**< Pointer to the parent PWM memory section */
  volatile uint32_t *position_p; /**< Direct register access pointer */
  volatile uint32_t *pos_init_p; /**< Direct register access pointer */
  volatile uint32_t *max_pos_p;  /**< Direct register access pointer */
  
  void map_pwm_register();
  void setHelper(int offset, int32_t value);
  void setHelper(int offset, int16_t value);
  int32_t getHelper(int offset);
  int16_t getHelper(int offset);
	
public:
  eQEP(int eQEP_address);
  ~eQEP();
  
  // eQEP Position Counter Register
  /** Returns the current eQEP counter value
   * 32-bit position counter register counts up/down on every eQEP pulse
   * based on direction input. This counter acts as a position integrator whose
   * count value is proportional to position from a give reference point.
  **/
  uint32_t getPosition();
  operator uint32_t();
  /**
   *  This 32-bit position counter register counts up/down on every eQEP pulse
   * based on direction input. This counter acts as a position integrator whose
   * count value is proportional to position from a give reference point.
  **/
  void setPosition(uint32_t position);
  void operator=(uint32_t position);
  
  // eQEP Position Counter Initialization Register 
  /**
   * This register contains the position value that is used to initialize the
   * position counter based on external strobe or index event. The position
   * counter can be initialized through software.
  **/
  uint32_t getPosInit();
  /**
   * This register contains the position value that is used to initialize the
   * position counter based on external strobe or index event. The position
   * counter can be initialized through software.
  **/
  void setPosInit(uint32_t pos_init);
  
  // eQEP Maximum Position Count Register
  /**
   * This register contains the maximum position counter value.
  **/
  uint32_t getMaxPos();
  /**
   * This register contains the maximum position counter value.
  **/
  void setMaxPos(uint32_t max_pos);
  
  /////////////////////
  /* New Functions!! */
  /////////////////////
  // eQEP Position-Compare Register
  /**
   * The position-compare value in this register is compared with the position
   * counter (QPOSCNT) to generate sync output and/or interrupt on compare
   * match.
  **/
  uint32_t getPositionCompare();
  /**
   * The position-compare value in this register is compared with the position
   * counter (QPOSCNT) to generate sync output and/or interrupt on compare
   * match.
   * 
   * @see getPosition()
   * @see setPosition()
   * @see setPositionCompareControl()
  **/
  void setPositionCompare(uint32_t);
  
  // eQEP Index Position Latch Register
  /**
   * The position-counter value is latched into this register on an index event
   * as defined by the QEPCTL[IEL] bits. 
   * 
   * TODO: add @see's here for the QEPCTL[IEL] bits
  **/
  uint32_t getIndexPositionLatch();
  
  // eQEP Strobe Position Latch Register
  /**
   * The position-counter value is latched into this register on strobe event
   * as defined by the QEPCTL[SEL] bits.
  **/
  uint32_t getStrobePositionLatch();
  
  // eQEP Position Counter Latch Register
  /**
   * The position-counter value is latched into this register on unit time out
   * event.
  **/
  uint32_t getPositionCounterLatch();
  
  // eQEP Unit Timer Register
  /**
   * This register acts as time base for unit time event generation. When this
   * timer value matches with unit time period value, unit time event is
   * generated.
  **/
  uint32_t getUnitTimer();
  /**
   * This register acts as time base for unit time event generation. When this
   * timer value matches with unit time period value, unit time event is
   * generated.
  **/
  void setUnitTimer(uint32_t);
  
  // eQEP Unit Period Register
  /**
   * This register contains the period count for unit timer to generate
   * periodic unit time events to latch the eQEP position information at
   * periodic interval and optionally to generate interrupt.
  **/
  uint32_t getUnitPeriod();
  /**
   * This register contains the period count for unit timer to generate
   * periodic unit time events to latch the eQEP position information at
   * periodic interval and optionally to generate interrupt.
  **/
  void setUnitPeriod(uint32_t);
  
  // eQEP Watchdog Timer Register
  /**
   * This register acts as time base for watch dog to detect motor stalls. When
   * this timer value matches with watch dog period value, watch dog timeout
   * interrupt is generated. This register is reset upon edge transition in
   * quadrature-clock indicating the motion.
  **/
  uint16_t getWatchdogTimer();
  /**
   * This register acts as time base for watch dog to detect motor stalls. When
   * this timer value matches with watch dog period value, watch dog timeout
   * interrupt is generated. This register is reset upon edge transition in
   * quadrature-clock indicating the motion.
  **/
  void setWatchdogTimer(uint16_t);
  
  // eQEP Watchdog Period Register
  /** 
   * This register contains the time-out count for the eQEP peripheral watch
   * dog timer. When the watchdog timer value matches the watchdog period
   * value, a watchdog timeout interrupt is generated.
  **/
  uint16_t getWatchdogPeriod();
  /** 
   * This register contains the time-out count for the eQEP peripheral watch
   * dog timer. When the watchdog timer value matches the watchdog period
   * value, a watchdog timeout interrupt is generated.
  **/
  void setWatchdogPeriod(uint16_t);
  
  // eQEP Decoder Control Register
  /**
   * Get the current eQEP Decoder control register setting
  **/
  uint16_t getDecoderControl();
  /**
   * Set the current eQEP Decoder control register setting
  **/
  void setDecoderControl(uint16_t);
  
  /**
   * Position-Counter source selection.
  *
   * /param source Select from:
   * 0: Quadrature count mode
   * 1: Direction count mode
   * 2: UP count mode for frequency measurement
   * 3: DOWN count mode for frequency measurement
  **/
  void positionCounterSourceSelection(int source);
  /**
   * Disable position-compare sync output.
  **/
  void disableSyncOutput();
  
  /**
   * Sync output enable pin selection enum.
   * 
   * @see enableSyncOutput();
  **/
  enum SOPin {
    Index = 0, ///< Use the index pin for sync output.
    Strobe = 1 ///< Use the strobe pin for sync output.
  };
  /**
   * Enable position-compare sync output.
   * 
   * /param pin Defines the pins to sync output. Index or Strobe.
  **/
  void enableSyncOutput(SOPin pin);
  /**
   * External clock rate. Default (2x).
  **/
  void twoXClockRate();
  /**
   * External clock rate. Half (1x).
  **/
  void oneXClockRate();
  /**
   * Swap quadrature clock inputs. This swaps the input to the quadrature
   * decoder, reversing the counting direction.
  **/
  void swapInputs();
  /**
   * Disable index pulse gating option.
  **/
  void disableGating();
  /**
   * Gate the index pin with strobe.
  **/
  void enableGating();
  /**
   * Inverts eQEPA input. Useful for rectifying after using a transistor.
  **/
  void invertInputA();
  /**
   * Inverts eQEPB input. Useful for rectifying after using a transistor.
  **/
  void invertInputB();
  /**
   * Inverts Index input. Useful for rectifying after using a transistor.
  **/
  void invertIndex();
  /**
   * Inverts Strobe input. Useful for rectifying after using a transistor.
  **/
  void invertStrobe();
  
  // eQEP Control Register
  /**
   * get eQEP Control Register
  **/
  uint16_t getControl();
  /**
   * set eQEP Control Register
  **/
  void setControl(uint16_t);
  
  /**
   * Emulation control enum. Effects eQEP behavior through emulation suspensions
   * such as debugging.
   *
   * @see setEmulationControl()
  **/
  enum ECB {
    /**
     * Counters and timers will stop immediately on emulation suspend
    **/
    StopImmediately = 0,
    /**
     * Counters and timers will wait for the next event on emulation suspend
    **/
    WaitForRollover = 1,
    /**
     * Counters and timers will continue through emulation suspend
    **/
    NoStop = 2
  };
  /**
   * Set the Emulation Control bits.
  **/
  void setEmulationControl(ECB mode);
  /**
   * Position Counter Reset Mode enum.
   *
   * @see setPositionCounterResetMode()
  **/
  enum PCRM {
    OnIndex = 0, /**< Position counter reset on an index event.*/
    OnMax = 1,   /**< Position counter reset on the maximum position.*/
    OnFirstIndex = 2, /**< Position counter reset on the first index event.*/
    OnUnitTime = 3 /**< Position counter resent on a unit time event.*/
  };
  /**
   * Sets the condition for reseting the position counter.
  **/
  void setPositionCounterResetMode(PCRM mode);
  
  /**
   * Strobe event initialization enum.
   *
   * @see setStrobeEventInit();
  **/
  enum SEI {
    /**
     * Does nothing (action disabled)
    **/
    SEIDisabled = 0,
    /**
     * Initializes the position counter on rising edge of QEPS signal.
    **/
    SEIRisingEdge = 2,
    /**
     * Clockwise Direction: Initializes the position counter on the rising
     * edge of QEPS strobe
     * Counter Clockwise Direction: Initializes the position counter on the
     * falling edge of QEPS strobe
    **/
    SEIConditionalEdge = 3
  };
  /**
   * Sets when a strobe event should initialize the position counter
  **/
  void setStrobeEventInit(SEI mode);
  /**
   * Index Event Initialization of position counter enum
  **/
  enum IEI { 
    /**
     * Does nothing (action disabled)
    **/
    IEIDisabled = 0,
    /**
     * Initializes the position counter on the rising edge of the QEPI signal
     * (QPOSCNT = QPOSINIT)
    **/
    IEIRisingEdge = 2,
    /**
     * Initializes the position counter on the falling edge of QEPI signal
     * (QPOSCNT = QPOSINIT)
    **/
    IEIFallingEdge = 3
  };
  /**
   * Sets when an index event should initialize the position counter
  **/
  void setIndexEventInit(IEI mode);
  /**
   * Software initialization of position counter. Call this function to reset
   * the postion counter. QPOSCNT = QPOSINIT
  **/
  void resetPositionCounter();
  /**
   * Strobe event latch of position counter
  **/
  enum SEL { 
    /**
     * The position counter is latched on the rising edge of QEPS strobe
     * (QPOSSLAT = POSCCNT). Latching on the falling edge can be done by
     * inverting the strobe input using invertStrobe() (the QSP bit in the
     * QDECCTL register.)
     *
     * @see invertStrobe()
    **/
    SELRisingEdge = 0,
    /**
     * Clockwise Direction: Position counter is latched on rising edge of
     * QEPS strobe
     * Counter Clockwise Direction: Position counter is latched on falling edge
     * of QEPS strobe
    **/
    SELConditional = 1,
  };
  /**
   * Set which strobe event should latch the position counter
  **/
  void setStrobeEventPositionLatch(SEL mode);
  /**
   * Strobe event latch of position counter
  **/
  enum IEL { 
    /**
     * Latches position counter on rising edge of the index signal
    **/
    IELRisingEdge = 1,
    /**
     * Latches position counter on falling edge of the index signal
    **/
    IELFallingEdge = 2,
    /**
     * Software index marker. Latches the position counter and quadrature
     * direction flag on index event marker. The position counter is latched to
     * the QPOSILAT register and the direction flag is latched in the
     * QEPSTS[QDLF] bit. This mode is useful for software index marking.
    **/
    IELSoftware = 3,
  };
  /**
   * Set which index event should latch the position counter
  **/
  void setIndexEventPositionLatch(IEL mode);
  /**
   * Quadrature position counter enable/software reset
   * Reset the eQEP peripheral internal operating flags/read-only registers.
   * Control/configuration registers are not disturbed by a software reset.
   * should be followed by a call to enableeQEP()
   *
   * @see enableeQEP()
  **/
  void reseteQEP();
  /**
   * Quadrature position counter enable after software reset
   * @see reseteQEP()
  **/
  void enableeQEP();
  /**
   * eQEP capture latch modes enum
  **/
  enum CLM {
    /**
     * Latch on position counter read by CPU. Capture timer and capture period
     * values are latched into QCTMRLAT and QCPRDLAT registers when CPU reads
     * the QPOSCNT register.
    **/
    CLMCPU = 0,
    /**
     * Latch on unit time out. Position counter, capture timer and capture
     * period values are latched into QPOSLAT, QCTMRLAT and QCPRDLAT registers
     * on unit time out. Unit timer must be enabled.
     *
     * @see enableUnitTimer()
    **/
    CLMUnitTime = 1
  };
  /**
   * eQEP capture latch mode
  **/
  void setCaptureLatchMode(CLM mode);
  /**
   * Enables the unit timer
  **/
  void enableUnitTimer();
  /**
   * Disables the unit timer
  **/
  void disableUnitTimer();
  /**
   * Enables the watchdog timer
  **/
  void enableWatchdogTimer();
  /**
   * Disables the watchdog timer
  **/
  void disableWatchdogTimer();

  // eQEP Capture Control Register
  /**
   * Get Capture Control register value.
  **/
  uint16_t getCaptureControl();
  /**
   * Set Capture Control register value.
   *
   * /warning The QCAPCTL register should not be modified dynamically (such as
   * switching CAPCLK prescaling mode from QCLK/4 to QCLK/8). The capture unit
   * must be disabled before changing the prescaler. @see disableCaptureUnit()
  **/
  void setCaptureControl(uint16_t);
  /**
   * Disable eQEP Capture Unit
  **/
  void disableCaptureUnit();
  /**
   * Enable eQEP Capture Unit
  **/
  void enableCaptureUnit();
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
  void setCaptureTimeClockPrescaler(int value);
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
  void setPositionEventPrescaler(int value);
  
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
  uint16_t getPositionCompareControl();
  /**
   * Set Position Compare Control register value.
  **/
  void setPositionCompareControl(uint16_t);
  /**
   * Disable Position Compare Shadow. Load Immediate
  **/
  void disablePositionCompareShadow();
  /**
   * Enable Position Compare Shadow
  **/
  void enablePositionCompareShadow();
  /**
   * Position Compare shadow load mode selection enum
  **/
  enum PCLOAD {
    /**
     * Load on Position == 0
    **/
    PCLOADCountEq0 = 0,
    /**
     * Load on Position == Position Compare register
     *
     * @see setPositionCompare()
    **/
    PCLOADCountEqPosCmp = 1
  };
  /**
   * Set position-compare shadow load mode
  **/
  void setPositionCompareShadowLoadMode(PCLOAD mode);
  /**
   * Polarity of sync output enum
   *
   * @see setPositionCompareSyncOutput()
  **/
  enum PCPOL {
    /**
     * Active high
    **/
    PCPOLActiveHigh = 0,
    /**
     * Active low
    **/
    PCPOLActiveLow = 1
  };
  /**
   * Polarity of sync output
  **/
  void setPositionCompareSyncOutput(PCPOL mode);
  /**
   * Enable the Position Compare Unit
  **/
  void enablePositionCompareUnit();
  /**
   * Disable the Position Compare Unit
  **/
  void disablePositionCompareUnit();
  /**
   * Select-position-compare sync output pulse width.
   *
   * /param value The prescaler factor used in equation 
  (value+1) * 4 * SYSCLKOUT cycles.
   * Should be between 0h and FFFh.
  **/
  void setPositionCompareSyncOutputPulseWidth(uint16_t);
  
  // eQEP Interrupt Enable Register
  /**
   * Get the current status of the interrupt enable bits
  **/
  uint16_t getInterruptEnable();
  /**
   * Turn on interrupts by passing the correct bits
  **/
  void setInterruptEnable(uint16_t);
  /**
   * Enable Unit Timeout Interrupt
  **/
  void enableUnitTimeoutInterrupt();
  /**
   * Disable Unit Timeout Interrupt
  **/
  void disableUnitTimeoutInterrupt();
  /**
   * Enable Index Event Latch Interrupt
  **/
  void enableIndexEventLatchInterrupt();
  /**
   * Disable Index Event Latch Interrupt
  **/
  void disableIndexEventLatchInterrupt();
  /**
   * Enable Strobe Event Latch Interrupt
  **/
  void enableStrobeEventLatchInterrupt();
  /**
   * Disable Strobe Event Latch Interrupt
  **/
  void disableStrobeEventLatchInterrupt();
  /**
   * Enable Position Compare Match Interrupt
  **/
  void enablePositionCompareMatchInterrupt();
  /**
   * Disable Position Compare Match Interrupt
  **/
  void disablePositionCompareMatchInterrupt();
  /**
   * Enable Position Compare Ready Interrupt
  **/
  void enablePositionCompareReadyInterrupt();
  /**
   * Disable Position Compare Ready Interrupt
  **/
  void disablePositionCompareReadyInterrupt();
  /**
   * Enable Position Counter Overflow Interrupt
  **/
  void enablePositionCounterOverflowInterrupt();
  /**
   * Disable Position Counter Overflow Interrupt
  **/
  void disablePositionCounterOverflowInterrupt();
  /**
   * Enable Position Counter Underflow Interrupt
  **/
  void enablePositionCounterUnderflowInterrupt();
  /**
   * Disable Position Counter Underflow Interrupt
  **/
  void disablePositionCounterUnderflowInterrupt();
  /**
   * Enable Watchdog Time Out Interrupt
  **/
  void enableWatchdogTimeOutInterrupt();
  /**
   * Disable Watchdog Time Out Interrupt
  **/
  void disableWatchdogTimeOutInterrupt();
  /**
   * Enable Quadrature Direction Change Interrupt
  **/
  void enableQuadratureDirectionChangeInterrupt();
  /**
   * Disable Quadrature Direction Change Interrupt
  **/
  void disableQuadratureDirectionChangeInterrupt();
  /**
   * Enable Quadrature Phase Error Interrupt
  **/
  void enableQuadraturePhaseErrorInterrupt();
  /**
   * Disable Unit Timeout Interrupt
  **/
  void disableQuadraturePhaseErrorInterrupt();
  /**
   * Enable Position Counter Error Interrupt
  **/
  void enablePositionCounterErrorInterrupt();
  /**
   * Disable Position Counter Error Interrupt
  **/
  void disablePositionCounterErrorInterrupt();
  
  // eQEP Interrupt Flag Register
  uint16_t getInterruptFlag();
  /**
   * Unit Time Out Interrupt flag.
   *
   * \return Set by eQEP unit timer period match.
  **/
  bool getUnitTimeoutInterruptFlag();
  /**
   * Index Event Latch Interrupt flag.
   *
   * \return True after latching the QPOSCNT to QPOSILAT.
  **/
  bool getIndexEventLatchInterruptFlag();
  /**
   * Strobe Event Latch Interrupt flag.
   *
   * \return True after latching the QPOSCNT to QPASSLAT.
  **/
  bool getStrobeEventLatchInterruptFlag();
  /**
   * Position Compare Match Interrupt flag.
   *
   * \return True after position compare match.
  **/
  bool getPositionCompareMatchInterruptFlag();
  /**
   * Position Compare Ready Interrupt flag.
   *
   * \return True after transferring the shadow register value to the active
   *         position compare register.
  **/
  bool getPositionCompareReadyInterruptFlag();
  /**
   * Position Counter Overflow Interrupt flag.
   *
   * \return True after position counter overflow.
  **/
  bool getPositionCounterOverflowInterruptFlag();
  /**
   * Position Counter Underflow Interrupt flag.
   *
   * \return True after position counter underflow.
  **/
  bool getPositionCounterUnderflowInterruptFlag();
  /**
   * Watchdog Time Out Interrupt flag.
   *
   * \return True after watch dog timeout.
  **/
  bool getWatchdogTimeOutInterruptFlag();
  /**
   * Quadrature Direction Change Interrupt flag.
   *
   * \return True after change in direction.
  **/
  bool getQuadratureDirectionChangeInterruptFlag();
  /**
   * Quadrature Phase Error Interrupt flag.
   *
   * \return True on simultaneous transition of QEPA and QEPB.
  **/
  bool getQuadraturePhaseErrorInterruptFlag();
  /**
   * Position Counter Error Interrupt Flag.
   *
   * \return True on position counter error.
  **/
  bool getPositionCounterErrorInterruptFlag();
  /**
   * Global Interrupt Status Flag
   *
   * \return True when an interrupt has been generated.
  **/
  bool getGlobalInterruptStatusFlag();
  
  // eQEP Interrupt Clear Register
  uint16_t getInterruptClear();
  void setInterruptClear(uint16_t);
  void clearInterrupts();
  /**
   * Clear the Unit Time Out Interrupt flag.
  **/
  void clearUnitTimeoutInterruptFlag();
  /**
   * Clear the Index Event Latch Interrupt
  **/
  void clearIndexEventLatchInterruptFlag();
  /**
   * Clear the Strobe Event Latch Interrupt
  **/
  void clearStrobeEventLatchInterruptFlag();
  /**
   * Clear the Position Compare Match Interrupt
  **/
  void clearPositionCompareMatchInterruptFlag();
  /**
   * Clear the Position Compare Ready Interrupt
  **/
  void clearPositionCompareReadyInterruptFlag();
  /**
   * Clear the Position Counter Overflow Interrupt
  **/
  void clearPositionCounterOverflowInterruptFlag();
  /**
   * Clear the Position Counter Underflow Interrupt
  **/
  void clearPositionCounterUnderflowInterruptFlag();
  /**
   * Clear the Watchdog Time Out Interrupt
  **/
  void clearWatchdogTimeOutInterruptFlag();
  /**
   * Clear the Quadrature Direction Change Interrupt
  **/
  void clearQuadratureDirectionChangeInterruptFlag();
  /**
   * Clear the Quadrature Phase Error Interrupt
  **/
  void clearQuadraturePhaseErrorInterruptFlag();
  /**
   * Clear the Position Counter Error Interrupt Flag
  **/
  void clearPositionCounterErrorInterruptFlag();
  /**
   * Clear the Global Interrupt Status Flag. Clears the interrupt flag and
   * enables further interrupts to be generated if an event flags is set to 1.
  **/
  void clearGlobalInterruptStatusFlag();
  
  // eQEP Interrupt Force Register
  uint16_t getInterruptForce();
  void setInterruptForce(uint16_t);
  /**
   * Force the Unit Time Out Interrupt flag.
  **/
  void forceUnitTimeoutInterruptFlag();
  /**
   * Force the Index Event Latch Interrupt
  **/
  void forceIndexEventLatchInterruptFlag();
  /**
   * Force the Strobe Event Latch Interrupt
  **/
  void forceStrobeEventLatchInterruptFlag();
  /**
   * Force the Position Compare Match Interrupt
  **/
  void forcePositionCompareMatchInterruptFlag();
  /**
   * Force the Position Compare Ready Interrupt
  **/
  void forcePositionCompareReadyInterruptFlag();
  /**
   * Force the Position Counter Overflow Interrupt
  **/
  void forcePositionCounterOverflowInterruptFlag();
  /**
   * Force the Position Counter Underflow Interrupt
  **/
  void forcePositionCounterUnderflowInterruptFlag();
  /**
   * Force the Watchdog Time Out Interrupt
  **/
  void forceWatchdogTimeOutInterruptFlag();
  /**
   * Force the Quadrature Direction Change Interrupt
  **/
  void forceQuadratureDirectionChangeInterruptFlag();
  /**
   * Force the Quadrature Phase Error Interrupt
  **/
  void forceQuadraturePhaseErrorInterruptFlag();
  /**
   * Force the Position Counter Error Interrupt Flag
  **/
  void forcePositionCounterErrorInterruptFlag();
  
  // eQEP Status Register
  uint16_t getStatus();
  void setStatus(uint16_t);
  
  // eQEP Capture Timer Register
  /**
   * This register provides time base for edge capture unit.
  **/
  uint16_t getCatureTimer();
  /**
   * This register provides time base for edge capture unit.
  **/
  void setCaptureTimer(uint16_t);
  
  // eQEP Capture Period Register
  /**
   * This register holds the period count value between the last successive
   * eQEP position events
  **/
  uint16_t getCapturePeriod();
  /**
   * This register holds the period count value between the last successive
   * eQEP position events
  **/
  void setCapturePeriod(uint16_t);
  
  // eQEP Capture Timer Latch Register
  /**
   * The eQEP capture timer value can be latched into this register on two
   * events viz., unit timeout event, reading the eQEP position counter.
  **/
  uint16_t getCaptureTimerLatch();
  
  // eQEP Capture Period Latch Register
  /**
   * eQEP capture period value can be latched into this register on two events
   * viz., unit timeout event, reading the eQEP position counter.
  **/
  uint16_t getCapturePeriodLatch();
  /**
   * eQEP capture period value can be latched into this register on two events
   * viz., unit timeout event, reading the eQEP position counter.
  **/
  void setCapturePeriodLatch(uint16_t);
  
  // eQEP Revision ID Register
  /**
   * eQEP revision ID
  **/
  uint32_t getRevisionID();

//#define EQEP_QPOSCNT	0x00
//#define EQEP_QPOSINIT	0x04
//#define EQEP_QPOSMAX	0x08
//#define EQEP_QPOSCMP	0x0c
//#define EQEP_QPOSILAT	0x10
//#define EQEP_QPOSSLAT	0x14
//#define EQEP_QPOSLAT	0x18
//#define EQEP_QUTMR	0x1c
//#define EQEP_QUPRD	0x20
//#define EQEP_QWDTMR	0x24
//#define EQEP_QWDPRD	0x26
//#define EQEP_QDECCTL	0x28
//#define		EQEP_QDECCTL_QSRC(x)	(((x) & 0x3) << 14)
//#define		EQEP_QDECCTL_SOEN	(1 << 13)
//#define		EQEP_QDECCTL_SPSEL	(1 << 12)
//#define		EQEP_QDECCTL_XCR	(1 << 11)
//#define		EQEP_QDECCTL_SWAP	(1 << 10)
//#define		EQEP_QDECCTL_IGATE	(1 << 9)
//#define		EQEP_QDECCTL_QAP	(1 << 8)
//#define		EQEP_QDECCTL_QBP	(1 << 7)
//#define		EQEP_QDECCTL_QIP	(1 << 6)
//#define		EQEP_QDECCTL_QSP	(1 << 5)
//#define EQEP_QEPCTL	0x2a
//#define		EQEP_QEPCTL_PCRM(x)	(((x) & 0x3) << 12)
//#define		EQEP_QEPCTL_SEI(x)	(((x) & 0x3) << 10)
//#define		EQEP_QEPCTL_IEI(x)	(((x) & 0x3) << 8)
//#define		EQEP_QEPCTL_SWI		(1 << 7)
//#define		EQEP_QEPCTL_SEL		(1 << 6)
//#define		EQEP_QEPCTL_IEL(x)	(((x) & 0x3) << 4)
//#define		EQEP_QEPCTL_PHEN	(1 << 3)
//#define		EQEP_QEPCTL_QCLM	(1 << 2)
//#define		EQEP_QEPCTL_UTE		(1 << 1)
//#define		EQEP_QEPCTL_WDE		(1 << 0)
//#define EQEP_QCAPCTL	0x2c
//#define EQEP_QPOSCTL	0x2e
//#define EQEP_QEINT	0x30
//#define EQEP_QFLG	0x32
//#define EQEP_QCLR	0x34
//#define EQEP_QFRC	0x36
//#define		EQEP_INT_UTO		(1 << 11) /* Same for all intr regs */
//#define		EQEP_INT_IEL		(1 << 10)
//#define		EQEP_INT_SEL		(1 << 9)
//#define		EQEP_INT_PCM		(1 << 8)
//#define		EQEP_INT_PCR		(1 << 7)
//#define		EQEP_INT_PCO		(1 << 6)
//#define		EQEP_INT_PCU		(1 << 5)
//#define		EQEP_INT_WTO		(1 << 4)
//#define		EQEP_INT_QDC		(1 << 3)
//#define		EQEP_INT_PHE		(1 << 2)
//#define		EQEP_INT_PCE		(1 << 1)
//#define		EQEP_INT_INT		(1 << 0)
//#define		EQEP_INT_ENABLE_ALL	(EQEP_INT_UTO | EQEP_INT_IEL \
//			| EQEP_INT_SEL | EQEP_INT_PCM | EQEP_INT_PCR \
//			| EQEP_INT_PCO | EQEP_INT_PCU | EQEP_INT_WTO \
//			| EQEP_INT_QDC | EQEP_INT_PHE | EQEP_INT_PCE)
//#define		EQEP_INT_MASK		(EQEP_INT_ENABLE_ALL | EQEP_INT_INT)
//#define EQEP_QEPSTS	0x38
//#define		EQEP_QEPSTS_UPEVNT	(1 << 7)
//#define		EQEP_QEPSTS_FDF		(1 << 6)
//#define		EQEP_QEPSTS_QDF		(1 << 5)
//#define		EQEP_QEPSTS_QDLF	(1 << 4)
//#define		EQEP_QEPSTS_COEF	(1 << 3)
//#define		EQEP_QEPSTS_CDEF	(1 << 2)
//#define		EQEP_QEPSTS_FIMF	(1 << 1)
//#define		EQEP_QEPSTS_PCEF	(1 << 0)
//#define EQEP_QCTMR	0x3a
//#define EQEP_QCPRD	0x3c
//#define EQEP_QCTMRLAT	0x3e
//#define EQEP_QCPRDLAT	0x40
//#define EQEP_REVID	0x5c
  
};

#endif /* end of include guard: BBB_EQEP_H */
