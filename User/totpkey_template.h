/*
 * totpkey.h
 *
 *  Rename it to "totpkey.h"
 *
 */

#ifndef USER_TOTPKEY_H_
#define USER_TOTPKEY_H_

// Number of TOTP tokens
#define TOTPKEYS 3

// TOTP Secrets
// 10 bytes each
// if your code is encoded by BASE32, decode it.

    const uint8_t hmacKey[] = {
            0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x21, 0xde, 0xad, 0xbe, 0xef ,
            0x4d, 0x79, 0x4c, 0x65, 0x67, 0x6f, 0x44, 0x6f, 0x6f, 0x72 ,
            0x4d, 0x79, 0x4c, 0x65, 0x67, 0x6f, 0x44, 0x6f, 0x6f, 0x72
    };

// TOTP labels

    const char *hmacLabel[] = {
            "totp sample",
            "Azure:yyyyyy",
            "Google:zzzzzz"
    };


#endif /* USER_TOTPKEY_H_ */
