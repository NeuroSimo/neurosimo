#ifndef SERDES_H
#define SERDES_H

#include <stdint.h>
#include <vector>


#define MAX_SERIALIZED_MESSAGE_LENGTH 100

#define GET_BYTE(var,n) (uint8_t)((var) >> (8 * (n)))

#define ESCAPE_CHARACTER 0x1B
#define TRANSPARENCY_MODIFIER 0x20
#define START_OF_MESSAGE 0xFE
#define END_OF_MESSAGE 0xFF
#define CHANNEL_SWITCH_CHAR 0xFD

class SerializedMessage
{
private:
  uint8_t length_;

public:
  SerializedMessage();

  void init(uint8_t channel);
  void init();
  void add_byte(uint8_t byte);
  void add_uint16(uint16_t value);
  void add_uint32(uint32_t value);
  void add_uint64(uint64_t value);

  void finalize();

  uint8_t get_length();

  uint8_t length;
  std::vector<uint8_t> serialized_message;

};

#endif
