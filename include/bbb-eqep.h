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

class eQEP
{
private:
  int eQEP_address_;
  int eQEPFd;
  
  bool active;
  
  void *pwm_addr;
  volatile uint32_t *position_p;
  volatile uint32_t *pos_init_p;
  volatile uint32_t *max_pos_p;
  
  void map_pwm_register();
		
public:
  eQEP(int eQEP_address);
  ~eQEP();
  
  // eQEP Position Counter Register
  uint32_t getPosition();
  void setPosition(uint32_t position);
  // eQEP Position Counter Initialization Register 
  uint32_t getPosInit();
  void setPosInit(uint32_t pos_init);
  
  // eQEP Maximum Position Count Register
  uint32_t getMaxPos();
  void setMaxPos(uint32_t max_pos);
  
  /* New Functions!! */
  // eQEP Position-Compare Register
  uint32_t getPositionCompare();
  void setPositionCompare(uint32_t);
  // eQEP Index Position Latch Register
  uint32_t getIndexPositionLatch();
  void setIndexPositionLatch(uint32_t);
  // eQEP Strobe Position Latch Register
  uint32_t getStrobePositionLatch();
  void setStrobePositionLatch(uint32_t);
  // eQEP Position Counter Latch Register
  uint32_t getPositionCounterLatch();
  void setPositionCounterLatch(uint32_t);
  // eQEP Unit Timer Register
  uint32_t getUnitTimer();
  void setUnitTimer(uint32_t);
  // eQEP Unit Period Register
  uint32_t getUnitPeriod();
  void setUnitPeriod(uint32_t);
  // eQEP Watchdog Timer Register
  uint32_t getWatchdogTimer();
  void setWatchdogTimer(uint32_t);
  // eQEP Watchdog Period Register
  uint32_t getWatchdogPeriod();
  void setWatchdogPeriod(uint32_t);
  // eQEP Decoder Control Register
  uint32_t getDecoderControl();
  void setDecoderControl(uint32_t);
  // eQEP Control Register
  uint32_t geteQEPControl();
  void seteQEPControl(uint32_t);
  // eQEP Capture Control Register
  uint32_t getCaptureControl();
  void setCaptureControl(uint32_t);
  // eQEP Position-Compare Control Register
  uint32_t getPositionCompareControl();
  void setPositionCompareControl(uint32_t);
  // eQEP Interrupt Enable Register
  uint32_t getInterruptEnable();
  void setInterruptEnable(uint32_t);
  // eQEP Interrupt Flag Register
  uint32_t getInterruptFlag();
  void setInterruptFlag(uint32_t);
  // eQEP Interrupt Clear Register
  uint32_t getInterruptClear();
  void setInterruptClear(uint32_t);
  // eQEP Interrupt Force Register
  uint32_t getInterruptForce();
  void setInterruptForce(uint32_t);
  // eQEP Status Register
  uint32_t getStatus();
  void setStatus(uint32_t);
  // eQEP Capture Timer Register
  uint32_t getCatureTimer();
  void setCaptureTimer(uint32_t);
  // eQEP Capture Period Register
  uint32_t getCapturePeriod();
  void setCapturePeriod(uint32_t);
  // eQEP Capture Timer Latch Register
  uint32_t getCaptureTimerLatch();
  void setCaptureTimerLatch(uint32_t);
  // eQEP Capture Period Latch Register
  uint32_t getCapturePeriodLatch();
  void setCapturePeriodLatch(uint32_t);
  // eQEP Revision ID Register
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
