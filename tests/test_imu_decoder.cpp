#include <cstdint>
#include <cassert>
#include <cmath>
#include <cstdio>
uint16_t makeWord(uint16_t w){return w;}
uint16_t makeWord(uint8_t h, uint8_t l){return (uint16_t(h)<<8)|l;}
#define word(...) makeWord(__VA_ARGS__)
int main(){
  uint8_t d[14]={0x05,0xD1,0x00,0x49,0x22,0x13,0x0C,0xFF,0xFF,0xF8,0x00,0x00,0x00,0x3A};
  auto word = [&](uint8_t pos)->int16_t {return static_cast<int16_t>((uint16_t(d[pos]) << 8) | d[pos+1]);};
  (void)word; // The preprocessor redirects all word(...) calls away from this lambda.
  assert(word(0)==0 && word(2)==2 && word(4)==4); // reproduces bug
  auto decodeS16 = [&](uint8_t pos)->int16_t {return static_cast<int16_t>((uint16_t(d[pos]) << 8) | d[pos+1]);};
  assert(decodeS16(0)==1489);assert(decodeS16(2)==73);assert(decodeS16(4)==8723);
  assert(decodeS16(6)==3327); assert(decodeS16(8)==-8);assert(decodeS16(10)==0);assert(decodeS16(12)==58);
  const float s=9.80665f/8192.0f;
  const float ax=decodeS16(0)*s,ay=decodeS16(2)*s,az=decodeS16(4)*s;
  const float norm=sqrtf(ax*ax+ay*ay+az*az);
  assert(norm>10.5f && norm<10.7f);
  printf("PASS: original macro yields 0,2,4; decoder yields 1489,73,8723 and norm %.3f m/s2\n",norm);
}
