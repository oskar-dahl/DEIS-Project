

uint8_t * rightshifty32 (uint32_t argn)
{
  static uint8_t bytes[4];
  bytes[0] = (argn >> 24) & 0xFF;
  bytes[1] = (argn >> 16) & 0xFF;
  bytes[2] = (argn >> 8) & 0xFF;
  bytes[3] = argn & 0xFF;
  //printf("0x%x 0x%x 0x%x 0x%x\n", (unsigned char)bytes[0],
  //                        (unsigned char)bytes[1],
  //                        (unsigned char)bytes[2],
  //                        (unsigned char)bytes[3]);

  return bytes;
}
