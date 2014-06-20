#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/time.h>

#include <bbb-eqep.h>

using std::cout;
using std::endl;
using std::cerr;
using namespace BBB;

int main (int argc, char const *argv[])
{
  struct timeval tv1, tv2;
  int eqep_num;
  uint32_t eqep_pos, eqep0_pos;
  
  if(argc < 2)
  {
    cout << "Usage: " << argv[0] << " 0|1|2" << endl;
    cout << "Requires the number for which eQEP to open\n";
    return 1;
  }
  
  if (strtol(argv[1],NULL,0) >= 0 && strtol(argv[1],NULL,0) <= 2) {
    eqep_num = strtol(argv[1],NULL,0);
  } else {
    cout << "Try again." << endl;
    return 1;
  }
  
  eQEP eqep(eqep_num);
  
  printf("SYSCONFIG 0x%X\n", *(uint32_t*)(eqep.getPWMSSPointer() + PWM_SYSCONFIG));
  printf("CLKCONFIG 0x%X\n", *(uint32_t*)(eqep.getPWMSSPointer()+PWM_CLKCONFIG));
  printf("QEPCTL0   0x%X\n",  eqep.getControl());
  printf("QDECCTL0  0x%X\n",  eqep.getDecoderControl());
  printf("QEINT0    0x%X\n",  eqep.getInterruptEnable());
  printf("QUPRD0    0x%u\n", eqep.getUnitPeriod());
  printf("QPOSMAX0  0x%X\n", eqep.getMaxPos());
  printf("QEPSTS0   0x%X\n",  eqep.getStatus());
  
  eqep0_pos = eqep.getPosition();
  
  //time a read-loop to assess speed
  int num_reads = 1000000;
  int i;
  gettimeofday(&tv1,NULL);
  for(i=0;i<num_reads;i++){
    eqep_pos = eqep.getPosition();
  }
  gettimeofday(&tv2,NULL);
 
  //find difference between start and end time
  unsigned long dt_micros = (1000000 * tv2.tv_sec + tv2.tv_usec)-(1000000 * tv1.tv_sec + tv1.tv_usec);
  float time_per_read = (float)dt_micros/num_reads;
  double change_rate = ((uint32_t)(eqep_pos - eqep0_pos))/(dt_micros/1000000);
 
  printf("last position %i\n", eqep_pos);
  printf("micros per read %f\n", time_per_read);
  // printf("quadrature Hz %Lf\n",change_rate);
  printf("revid 0x%X (should read 44D31103)\n",eqep.getRevisionID());
  
  for(i=0;i<10000000;i++){
    printf("\reqep: %zu       ",eqep.getPosition());
    usleep(5000);
  }
  printf("\n");
  
  return 0;
}
