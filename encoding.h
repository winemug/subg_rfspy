#ifndef ENCODING_H
#define ENCODING_H

#include <stdint.h>
#include "manchester_state.h"

typedef enum {
  EncodingTypeNone = 0x0,
  EncodingTypeManchester = 0x1
  //EncodingType4b6b = 0x02
} EncodingType;

typedef struct {
  union {
    struct ManchesterEncoderState manchester;
    struct {
      uint8_t data;
      uint8_t count;
    } passthrough;
  };
} EncoderState;

typedef struct {
  union {
    struct ManchesterDecoderState manchester;
    struct {
      uint8_t data;
      uint8_t count;
    } passthrough;
  };
} DecoderState;

typedef struct {
  // Adds a byte to be encoded
  void (*add_raw_byte)(EncoderState *state, uint8_t raw) __reentrant;
  // encoded will be set to next available encoded byte.
  // Return value is 0 if no more bytes are available.
  uint8_t (*next_encoded_byte)(EncoderState *state, uint8_t *encoded) __reentrant;
} Encoder;

typedef struct {
  // Adds a byte for decoding. The return value will be 1 if the byte
  // contains encoding errors, otherwise it will be 0.
  uint8_t (*add_encoded_byte)(DecoderState *state, uint8_t raw) __reentrant;
  // decoded will be set to the next available decoded byte.
  // Return value is 0 if no more bytes are available.
  uint8_t (*next_decoded_byte)(DecoderState *state, uint8_t *decoded) __reentrant;
} Decoder;

void init_encoder(EncodingType encoding_type, Encoder *encoder, EncoderState *state);
void init_decoder(EncodingType encoding_type, Decoder *decoder, DecoderState *state);

#endif // ENCODING_H
