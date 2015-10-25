// The following LGPL 2.1+ byteswap macros slightly modified from
// http://repo-genesis3.cbi.utsa.edu/crossref/ns-sli/usr/include/bits/byteswap.h.html
// These byteswaps are needed, as 16bit ints and 32bit longs in the Zigbee message byte order are byteswapped
// compared to the Arduino's requirements to conveniently use them as 16bit int or 32bit long values,
// rather than doing more work to deal with everything as individual bytes
#define BYTESWAP16(x) \
  (uint16_t)((((x) >> 8) & 0xff) | (((x) & 0xff) << 8))

#define BYTESWAP32(x) \
  (uint32_t)((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) |               \
             (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))

