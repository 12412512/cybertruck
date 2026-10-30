#define main AxisFixtureMain
#include "mech_axis.c"
#undef main
int main(void) {
 for(int mode=0;mode<2;mode++) for(int beyond=0;beyond<2;beyond++) {
  lift=-9;reach=-7;Init();guard.manual_range=mode;
  now+=80;
  Feed(0,0x35,0);Feed(0,0x36,-9);Feed(0,0x33,-9);Feed(0,0x3A,3);
  Feed(1,0x35,0);Feed(1,0x36,beyond?600:307);Feed(1,0x33,-6);Feed(1,0x3A,3);
  Mech_Poll(&guard,now,1);
  if(mode&&!beyond) CHECK(!guard.fault && guard.zero_valid);
  else CHECK(guard.fault==MECH_RANGE);
 }
 return 0;
}
