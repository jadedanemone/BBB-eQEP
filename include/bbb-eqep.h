/*! \file bbb-eqep.h
 * \brief Beaglebone Black eQEP header file.
 * Contains all of the method declarations for class eQEP.
**/

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
  uint32_t operator=();
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
    Index = 0,
    Strobe = 1
  };
  /**
   * Enable position-compare sync output.
   * 
   * /param pin Defines the pins to sync output. INDEX or STROBE.
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
  uint16_t getControl();
  void setControl(uint16_t);
  
  enum ECB {
    StopImmediately = 0, ///<
    WaitForRollover = 1, ///<
    NoStop = 2 ///<
  }
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
    Disabled = 0,
    /**
     * Initializes the position counter on rising edge of QEPS signal.
    **/
    RisingEdge = 2,
    /**
     * Clockwise Direction: Initializes the position counter on the rising
     * edge of QEPS strobe
     * Counter Clockwise Direction: Initializes the position counter on the
     * falling edge of QEPS strobe
    **/
    ConditionalEdge = 3
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
    Disabled = 0,
    /**
     * Initializes the position counter on the rising edge of the QEPI signal
     * (QPOSCNT = QPOSINIT)
    **/
    RisingEdge = 2,
    /**
     * Initializes the position counter on the falling edge of QEPI signal
     * (QPOSCNT = QPOSINIT)
    **/
    FallingEdge = 3
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
    RisingEdge = 0,
    /**
     * Clockwise Direction: Position counter is latched on rising edge of
     * QEPS strobe
     * Counter Clockwise Direction: Position counter is latched on falling edge
     * of QEPS strobe
    **/
    Conditional = 1,
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
    RisingEdge = 1,
    /**
     * Latches position counter on falling edge of the index signal
    **/
    FallingEdge = 2,
    /**
     * Software index marker. Latches the position counter and quadrature
     * direction flag on index event marker. The position counter is latched to
     * the QPOSILAT register and the direction flag is latched in the
     * QEPSTS[QDLF] bit. This mode is useful for software index marking.
    **/
    Software = 3,
  };
  /**
   * Set which index event should latch the position counter
  **/
  void setIndexEventPositionLatch(IEL mode);
  /**
   * Quadrature position counter enable/software reset
   * Reset the eQEP peripheral internal operating flags/read-only registers.
   * Control/configuration registers are not disturbed by a software reset.
  **/
  void reseteQEP();
  /**
   * Quadrature position counter enable after software reset
   * @see reseteQEP()
  **/
  void enableeQEP();
  3
  PHEN
  0 1
  Quadrature position counter enable/software reset
  Reset the eQEP peripheral internal operating flags/read-only registers. Control/configuration registers are not disturbed by a software reset.
  eQEP position counter is enabled
  2
  QCLM
  0 1
  eQEP capture latch mode
  Latch on position counter read by CPU. Capture timer and capture period values are latched into QCTMRLAT and QCPRDLAT registers when CPU reads the QPOSCNT register.
  Latch on unit time out. Position counter, capture timer and capture period values are latched into QPOSLAT, QCTMRLAT and QCPRDLAT registers on unit time out.
  1
  UTE
  0 1
  eQEP unit timer enable Disable eQEP unit timer Enable unit timer
  0
  WDE
  0 1
  eQEP watchdog enable
  Disable the eQEP watchdog timer Enable the eQEP watchdog timer
  // eQEP Capture Control Register
  uint16_t getCaptureControl();
  void setCaptureControl(uint16_t);
  
  // eQEP Position-Compare Control Register
  uint16_t getPositionCompareControl();
  void setPositionCompareControl(uint16_t);
  
  // eQEP Interrupt Enable Register
  uint16_t getInterruptEnable();
  void setInterruptEnable(uint16_t);
  
  // eQEP Interrupt Flag Register
  uint16_t getInterruptFlag();
  
  // eQEP Interrupt Clear Register
  uint16_t getInterruptClear();
  void setInterruptClear(uint16_t);
  
  // eQEP Interrupt Force Register
  uint16_t getInterruptForce();
  void setInterruptForce(uint16_t);
  
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
