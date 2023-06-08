#include <stdio.h>
#include <memory.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <cinttypes>

#define MAX_TYPE_LEN 10
#define MAX_VLAUE_LEN 100

#define PM3_EINVARG -2
#define PM3_SUCCESS 0


#define NOLF "\xff"

#define AEND  "\x1b[0m"

#define _BLUE_(s) "\x1b[34m" s AEND
#define _RED_(s) "\x1b[31m" s AEND
#define _GREEN_(s) "\x1b[32m" s "\x1b[0m"
#define _YELLOW_(s) "\x1b[33m" s AEND
#define _MAGENTA_(s) "\x1b[35m" s AEND
#define _CYAN_(s) "\x1b[36m" s AEND
#define _WHITE_(s) "\x1b[37m" s AEND

#define _CLEAR_ "\x1b[2J"
#define _TOP_   "\x1b[1;1f"
#define _RL_RED_(s) RL_ESC("\x1b[31m") s RL_ESC(AEND)
#define _RL_GREEN_(s) RL_ESC("\x1b[32m") s RL_ESC(AEND)
#define _RL_BOLD_RED_(s) RL_ESC("\x1b[1;31m") s RL_ESC(AEND)
#define _RL_BOLD_GREEN_(s) RL_ESC("\x1b[1;32m") s RL_ESC(AEND)
//parity

static const uint8_t g_odd_byte_parity[256] = {
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1
};

//extern const uint8_t OddByteParity[256];

#define ODD_PARITY8(x)   { g_odd_byte_parity[x] }
#define EVEN_PARITY8(x)  { !g_odd_byte_parity[x] }

static inline uint8_t oddparity8(const uint8_t x) {
    return g_odd_byte_parity[x];
}

static inline uint8_t evenparity8(const uint8_t x) {
    return !g_odd_byte_parity[x];
}

static inline uint8_t evenparity16(uint16_t x) {
#if !defined __GNUC__
    x ^= x >> 8;
    return EVEN_PARITY8(x) ;
#else
    return (__builtin_parity(x) & 0xFF);
#endif
}

static inline uint8_t oddparity16(uint16_t x) {
#if !defined __GNUC__
    x ^= x >> 8;
    return ODD_PARITY8(x);
#else
    return !__builtin_parity(x);
#endif
}

static inline uint8_t evenparity32(uint32_t x) {
#if !defined __GNUC__
    x ^= x >> 16;
    x ^= x >> 8;
    return EVEN_PARITY8(x);
#else
    return (__builtin_parity(x) & 0xFF);
#endif
}

static inline uint8_t oddparity32(uint32_t x) {
#if !defined __GNUC__
    x ^= x >> 16;
    x ^= x >> 8;
    return ODD_PARITY8(x);
#else
    return !__builtin_parity(x);
#endif
}


//end parity





typedef struct {
    bool hasCardNumber;
    bool hasFacilityCode;
    bool hasIssueLevel;
    bool hasOEMCode;
    bool hasParity;
} cardformatdescriptor_t;


enum{ BIN, HEX, DEC};
// Structure for packed wiegand messages
// Always align lowest value (last transmitted) bit to ordinal position 0 (lowest valued bit bottom)
typedef struct {
    uint8_t Length;   // Number of encoded bits in wiegand message (excluding headers and preamble)
    uint32_t Top;     // Bits in x<<64 positions
    uint32_t Mid;     // Bits in x<<32 positions
    uint32_t Bot;     // Lowest ordinal positions
} wiegand_message_t;

// Structure for unpacked wiegand card, like HID prox
typedef struct {
    uint32_t FacilityCode;
    uint64_t CardNumber;
    uint32_t IssueLevel;
    uint32_t OEM;
    bool ParityValid; // Only valid for responses
} wiegand_card_t;


typedef struct {
    const char *Name;
    bool (*Pack)(wiegand_card_t *card, wiegand_message_t *packed, bool preamble);
    bool (*Unpack)(wiegand_message_t *packed, wiegand_card_t *card);
    const char *Descrp;
    cardformatdescriptor_t Fields;
} cardformat_t;

typedef struct{
    const char Bits[50];
    const char SOffset[50];
    const char Slength[50];
    const char IOffset[50];
    const char Ilength[50];
    const char COffset[50];
    const char Clength[50];
    const char EvenP[100];
    const char Evenmp[80];
    const char OddnP[50];
    const char Oddmp[50];
}cardInfo_t;




//formatutils

uint8_t get_bit_by_position(wiegand_message_t *data, uint8_t pos) {
    if (pos >= data->Length) return false;
    pos = (data->Length - pos) - 1; // invert ordering; Indexing goes from 0 to 1. Subtract 1 for weight of bit.
    uint8_t result = 0;
    if (pos > 95)
        result = 0;
    else if (pos > 63)
        result = (data->Top >> (pos - 64)) & 1;
    else if (pos > 31)
        result = (data->Mid >> (pos - 32)) & 1;
    else
        result = (data->Bot >> pos) & 1;
    return result;
}
bool set_bit_by_position(wiegand_message_t *data, bool value, uint8_t pos) {
    if (pos >= data->Length) return false;
    pos = (data->Length - pos) - 1; // invert ordering; Indexing goes from 0 to 1. Subtract 1 for weight of bit.
    if (pos > 95) {
        return false;
    } else if (pos > 63) {
        if (value)
            data->Top |= (1UL << (pos - 64));
        else
            data->Top &= ~(1UL << (pos - 64));
        return true;
    } else if (pos > 31) {
        if (value)
            data->Mid |= (1UL << (pos - 32));
        else
            data->Mid &= ~(1UL << (pos - 32));
        return true;
    } else {
        if (value)
            data->Bot |= (1UL << pos);
        else
            data->Bot &= ~(1UL << pos);
        return true;
    }
}
/**
 * Safeguard the data by doing a manual deep copy
 *
 * At the time of the initial writing, the struct does not contain pointers. That doesn't
 * mean it won't eventually contain one, however. To prevent memory leaks and erroneous
 * aliasing, perform the copy function manually instead. Hence, this function.
 *
 * If the definition of the wiegand_message struct changes, this function must also
 * be updated to match.
 */
static void message_datacopy(wiegand_message_t *src, wiegand_message_t *dest) {
    dest->Bot = src->Bot;
    dest->Mid = src->Mid;
    dest->Top = src->Top;
    dest->Length = src->Length;
}
/**
 *
 * Yes, this is horribly inefficient for linear data.
 * The current code is a temporary measure to have a working function in place
 * until all the bugs shaken from the block/chunk version of the code.
 *
 */
uint64_t get_linear_field(wiegand_message_t *data, uint8_t firstBit, uint8_t length) {
    uint64_t result = 0;
    for (uint8_t i = 0; i < length; i++) {
        result = (result << 1) | get_bit_by_position(data, firstBit + i);
    }
    return result;
}
bool set_linear_field(wiegand_message_t *data, uint64_t value, uint8_t firstBit, uint8_t length) {
    wiegand_message_t tmpdata;
    message_datacopy(data, &tmpdata);
    bool result = true;
    for (int i = 0; i < length; i++) {
        result &= set_bit_by_position(&tmpdata, (value >> ((length - i) - 1)) & 1, firstBit + i);
    }
    if (result)
        message_datacopy(&tmpdata, data);

    return result;
}

uint64_t get_nonlinear_field(wiegand_message_t *data, uint8_t numBits, uint8_t *bits) {
    uint64_t result = 0;
    for (int i = 0; i < numBits; i++) {
        result = (result << 1) | get_bit_by_position(data, *(bits + i));
    }
    return result;
}
bool set_nonlinear_field(wiegand_message_t *data, uint64_t value, uint8_t numBits, uint8_t *bits) {

    wiegand_message_t tmpdata;
    message_datacopy(data, &tmpdata);

    bool result = true;
    for (int i = 0; i < numBits; i++) {
        result &= set_bit_by_position(&tmpdata, (value >> ((numBits - i) - 1)) & 1, *(bits + i));
    }

    if (result)
        message_datacopy(&tmpdata, data);

    return result;
}

static uint8_t get_length_from_header(wiegand_message_t *data) {
    /**
     * detect if message has "preamble" / "sentinel bit"
     *
     */


    uint8_t len = 0;
    uint32_t hfmt = 0; // for calculating card length

    if ((data->Top & 0x000FFFFF) > 0) { // > 64 bits
        hfmt = data->Top & 0x000FFFFF;
        len = 64;
    } else if ((data->Mid & 0xFFFFFFC0) > 0) { // < 63-38 bits
        hfmt = data->Mid & 0xFFFFFFC0;
        len = 32;
    } else if (data->Mid && (data->Mid & 0x00000020) == 0) { // 37 bits;
        hfmt = 0;
        len = 37;
    } else if ((data->Mid & 0x0000001F) > 0) { // 36-32 bits
        hfmt = data->Mid & 0x0000001F;
        len = 32;
    } else {
        hfmt = data->Bot;
        len = 0;
    }

    while (hfmt > 1) {
        hfmt >>= 1;
        len++;
    }
    if (len < 26)
        len = 26;
    return len;
}

wiegand_message_t initialize_message_object(uint32_t top, uint32_t mid, uint32_t bot, int n) {
    wiegand_message_t result;
    memset(&result, 0, sizeof(wiegand_message_t));

    result.Top = top;
    result.Mid = mid;
    result.Bot = bot;
    if (n > 0)
        result.Length = n;
    else
        result.Length = get_length_from_header(&result);
    return result;
}

bool add_HID_header(wiegand_message_t *data) {
    // Invalid value
    if (data->Length > 84 || data->Length == 0)
        return false;

    if (data->Length >= 64) {
        data->Top |= 0x09e00000; // Extended-length header
        data->Top |= 1U << (data->Length - 64); // leading 1: start bit
    } else if (data->Length > 37) {
        data->Top |= 0x09e00000; // Extended-length header
        data->Mid |= 1U << (data->Length - 32); // leading 1: start bit
    } else if (data->Length == 37) {
        // No header bits added to 37-bit cards
    } else if (data->Length >= 32) {
        data->Mid |= 0x20; // Bit 37; standard header
        data->Mid |= 1U << (data->Length - 32); // leading 1: start bit
    } else {
        data->Mid |= 0x20; // Bit 37; standard header
        data->Bot |= 1U << data->Length; // leading 1: start bit
    }
    return true;
}

//end formatutils





/**
 * Converts a hex string to component "hi2", "hi" and "lo" 32-bit integers
 * one nibble at a time.
 *
 * Returns the number of nibbles (4 bits) entered.
 */
int hexstring_to_u96(uint32_t *hi2, uint32_t *hi, uint32_t *lo, const char *str) {
    uint32_t n = 0, i = 0;

    while (sscanf(&str[i++], "%1x", &n) == 1) {
        *hi2 = (*hi2 << 4) | (*hi >> 28);
        *hi = (*hi << 4) | (*lo >> 28);
        *lo = (*lo << 4) | (n & 0xf);
    }
    return i - 1;
}

/**
 * Converts a binary string to component "hi2", "hi" and "lo" 32-bit integers,
 * one bit at a time.
 *
 * Returns the number of bits entered.
 */
int binstring_to_u96(uint32_t *hi2, uint32_t *hi, uint32_t *lo, const char *str) {
    uint32_t n = 0, i = 0;

    for (;;) {

        int res = sscanf(&str[i], "%1u", &n);
        if ((res != 1) || (n > 1))
            break;

        *hi2 = (*hi2 << 1) | (*hi >> 31);
        *hi = (*hi << 1) | (*lo >> 31);
        *lo = (*lo << 1) | (n & 0x1);

        i++;
    }
    return i;
}


/**
 * Converts a binary array to component "hi2", "hi" and "lo" 32-bit integers,
 * one bit at a time.
 *
 * Returns the number of bits entered.
 */
int binarray_to_u96(uint32_t *hi2, uint32_t *hi, uint32_t *lo, const uint8_t *arr, int arrlen) {
    int i = 0;
    for (; i < arrlen; i++) {
        uint8_t n = arr[i];
        if (n > 1)
            break;

        *hi2 = (*hi2 << 1) | (*hi >> 31);
        *hi = (*hi << 1) | (*lo >> 31);
        *lo = (*lo << 1) | (n & 0x1);
    }
    return i;
}



static bool Pack_H10301(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 26; // Set number of bits
    packed->Bot |= (card->CardNumber & 0xFFFF) << 1;
    packed->Bot |= (card->FacilityCode & 0xFF) << 17;
    packed->Bot |= oddparity32((packed->Bot >> 1) & 0xFFF);
    packed->Bot |= (evenparity32((packed->Bot >> 13) & 0xFFF)) << 25;
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_H10301(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));
    if (packed->Length != 26) return false; // Wrong length? Stop here.

    card->CardNumber = (packed->Bot >> 1) & 0xFFFF;
    card->FacilityCode = (packed->Bot >> 17) & 0xFF;
    card->ParityValid =
        (oddparity32((packed->Bot >> 1) & 0xFFF) == (packed->Bot & 1)) &&
        ((evenparity32((packed->Bot >> 13) & 0xFFF)) == ((packed->Bot >> 25) & 1));
    return true;
}

static bool Pack_ind26(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFFF) return false; // 12 bits
    if (card->CardNumber > 0xFFF) return false; // 12 bits
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false;  // Not used in this format

    packed->Length = 26; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 1, 12);
    set_linear_field(packed, card->CardNumber, 13, 12);

    set_bit_by_position(packed,
                        evenparity32(get_linear_field(packed, 1, 12))
                        , 0);
    set_bit_by_position(packed,
                        oddparity32(get_linear_field(packed, 13, 12))
                        , 25);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_ind26(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 26) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 12);
    card->CardNumber = get_linear_field(packed, 13, 12);

    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 12))) &&
        (get_bit_by_position(packed, 25) == oddparity32(get_linear_field(packed, 13, 12)));
    return true;
}

static bool Pack_Tecom27(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x7FF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 27;
    uint8_t field_data1[] = {15, 19, 24, 23, 22, 18, 6, 10, 14, 3, 2};
    set_nonlinear_field(packed, card->FacilityCode, 11, field_data1);
    uint8_t field_data2[] = {0, 1, 13, 12, 9, 26, 20, 16, 17, 21, 25, 7, 8, 11, 4, 5};
    set_nonlinear_field(packed, card->CardNumber, 16, field_data2);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_Tecom27(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 27) return false; // Wrong length? Stop here.

    uint8_t field_data1[] = {0, 1, 13, 12, 9, 26, 20, 16, 17, 21, 25, 7, 8, 11, 4, 5};
    card->CardNumber = get_nonlinear_field(packed, 16, field_data1);
    uint8_t field_data2[] = {15, 19, 24, 23, 22, 18, 6, 10, 14, 3, 2};
    card->FacilityCode = get_nonlinear_field(packed, 11, field_data2);
    return true;
}

static bool Pack_ind27(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x1FFF) return false; // 13 bits
    if (card->CardNumber > 0x3FFF) return false; // 14 bits
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // 4 bit

    packed->Length = 27; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 0, 13);
    set_linear_field(packed, card->CardNumber, 13, 14);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_ind27(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 27) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 0, 13);
    card->CardNumber = get_linear_field(packed, 13, 14);
    return true;
}

static bool Pack_indasc27(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x1FFF) return false; // 13 bits
    if (card->CardNumber > 0x3FFF) return false; // 14 bits
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 27;
    uint8_t field_data1[] = {9, 4, 6, 5, 0, 7, 19, 8, 10, 16, 24, 12, 22};
    set_nonlinear_field(packed, card->FacilityCode, 11, field_data1);
    uint8_t field_data2[] = {26, 1, 3, 15, 14, 17, 20, 13, 25, 2, 18, 21, 11, 23};
    set_nonlinear_field(packed, card->CardNumber, 14, field_data2);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_indasc27(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 27) return false; // Wrong length? Stop here.
    uint8_t field_data1[] = {9, 4, 6, 5, 0, 7, 19, 8, 10, 16, 24, 12, 22};
    card->FacilityCode = get_nonlinear_field(packed, 11, field_data1);
    uint8_t field_data2[] = {26, 1, 3, 15, 14, 17, 20, 13, 25, 2, 18, 21, 11, 23};
    card->CardNumber = get_nonlinear_field(packed, 14, field_data2);
    return true;
}

static bool Pack_2804W(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x0FF) return false; // Can't encode FC.
    if (card->CardNumber > 0x7FFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 28;
    set_linear_field(packed, card->FacilityCode, 4, 8);
    set_linear_field(packed, card->CardNumber, 12, 15);
    uint8_t field_data1[] = {4, 5, 7, 8, 10, 11, 13, 14, 16, 17, 19, 20, 22, 23, 25, 26};
    set_bit_by_position(packed,
    oddparity32(get_nonlinear_field(packed, 16, field_data1)) , 2);
    set_bit_by_position(packed,
                        evenparity32(get_linear_field(packed, 1, 13))
                        , 0);
    set_bit_by_position(packed,
                        oddparity32(get_linear_field(packed, 0, 27))
                        , 27);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_2804W(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 28) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 4, 8);
    card->CardNumber = get_linear_field(packed, 12, 15);
    uint8_t field_data1[] = {4, 5, 7, 8, 10, 11, 13, 14, 16, 17, 19, 20, 22, 23, 25, 26};
    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 13))) &&
    (get_bit_by_position(packed, 2) == oddparity32(get_nonlinear_field(packed, 16, field_data1))) &&
    (get_bit_by_position(packed, 27) == oddparity32(get_linear_field(packed, 0, 27)));
    return true;
}

static bool Pack_ind29(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x1FFF) return false; // 13 bits
    if (card->CardNumber > 0xFFFF) return false; // 16 bits
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // 4 bit

    packed->Length = 29; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 0, 13);
    set_linear_field(packed, card->CardNumber, 13, 16);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_ind29(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 29) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 0, 13);
    card->CardNumber = get_linear_field(packed, 13, 16);
    return true;
}

static bool Pack_ATSW30(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFFF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 30;
    set_linear_field(packed, card->FacilityCode, 1, 12);
    set_linear_field(packed, card->CardNumber, 13, 16);
    set_bit_by_position(packed,
                        evenparity32(get_linear_field(packed, 1, 12))
                        , 0);
    set_bit_by_position(packed,
                        oddparity32(get_linear_field(packed, 13, 16))
                        , 29);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_ATSW30(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 30) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 12);
    card->CardNumber = get_linear_field(packed, 13, 16);
    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 12))) &&
        (get_bit_by_position(packed, 29) == oddparity32(get_linear_field(packed, 13, 16)));
    return true;
}

static bool Pack_ADT31(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x0F) return false; // Can't encode FC.
    if (card->CardNumber > 0x7FFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 31;
    set_linear_field(packed, card->FacilityCode, 1, 4);
    set_linear_field(packed, card->CardNumber, 5, 23);
    // Parity not known, but 4 bits are unused.
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_ADT31(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 31) return false; // Wrong length? Stop here.
    card->FacilityCode = get_linear_field(packed, 1, 4);
    card->CardNumber = get_linear_field(packed, 5, 23);
    return true;
}

static bool Pack_hcp32(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0) return false; // Not used
    if (card->CardNumber > 0x3FFF) return false; // 24 bits
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used

    packed->Length = 32; // Set number of bits

    set_linear_field(packed, card->CardNumber, 1, 24);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_hcp32(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 32) return false; // Wrong length? Stop here.

    card->CardNumber = get_linear_field(packed, 1, 24);
    return true;
}

static bool Pack_hpp32(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFFF) return false; // 12 bits
    if (card->CardNumber > 0x1FFFFFFF) return false; // 29 bits
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used

    packed->Length = 32; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 1, 12);
    set_linear_field(packed, card->CardNumber, 13, 29);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_hpp32(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 32) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 12);
    card->CardNumber = get_linear_field(packed, 13, 29);
    return true;
}

static bool Pack_wie32(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFFF) return false; // 12 bits
    if (card->CardNumber > 0xFFFF) return false; // 16 bits
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used

    packed->Length = 32; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 4, 12);
    set_linear_field(packed, card->CardNumber, 16, 16);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_wie32(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 32) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 4, 12);
    card->CardNumber = get_linear_field(packed, 16, 16);
    return true;
}

static bool Pack_Kastle(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x00FF) return false; // Can't encode FC.
    if (card->CardNumber > 0x0000FFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0x001F) return false; // IL is only 5 bits.
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 32; // Set number of bits
    set_bit_by_position(packed, 1, 1); // Always 1
    set_linear_field(packed, card->IssueLevel, 2, 5);
    set_linear_field(packed, card->FacilityCode, 7, 8);
    set_linear_field(packed, card->CardNumber, 15, 16);
    set_bit_by_position(packed, evenparity32(get_linear_field(packed, 1, 16)), 0);
    set_bit_by_position(packed, oddparity32(get_linear_field(packed, 14, 17)), 31);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_Kastle(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 32) return false; // Wrong length? Stop here.
    if (get_bit_by_position(packed, 1) != 1) return false; // Always 1 in this format

    card->IssueLevel = get_linear_field(packed, 2, 5);
    card->FacilityCode = get_linear_field(packed, 7, 8);
    card->CardNumber = get_linear_field(packed, 15, 16);
    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 16))) &&
        (get_bit_by_position(packed, 31) == oddparity32(get_linear_field(packed, 14, 17)));
    return true;
}

static bool Pack_Kantech(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 32;
    set_linear_field(packed, card->FacilityCode, 7, 8);
    set_linear_field(packed, card->CardNumber, 15, 16);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_Kantech(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 32) return false; // Wrong length? Stop here.
    card->FacilityCode = get_linear_field(packed, 7, 8);
    card->CardNumber = get_linear_field(packed, 15, 16);
    return true;
}

static bool Pack_D10202(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x007F) return false; // Can't encode FC.
    if (card->CardNumber > 0x00FFFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 33; // Set number of bits
    set_linear_field(packed, card->FacilityCode, 1, 7);
    set_linear_field(packed, card->CardNumber, 8, 24);
    set_bit_by_position(packed, evenparity32(get_linear_field(packed, 1, 16)), 0);
    set_bit_by_position(packed, oddparity32(get_linear_field(packed, 16, 16)), 32);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_D10202(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 33) return false; // Wrong length? Stop here.

    card->CardNumber = get_linear_field(packed, 8, 24);
    card->FacilityCode = get_linear_field(packed, 1, 7);
    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 16))) &&
        (get_bit_by_position(packed, 32) == oddparity32(get_linear_field(packed, 16, 16)));
    return true;
}

static bool Pack_H10306(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFFFF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 34; // Set number of bits
    packed->Bot |= (card->CardNumber & 0xFFFF) << 1;
    packed->Bot |= (card->FacilityCode & 0x7FFF) << 17;
    packed->Mid |= (card->FacilityCode & 0x8000) >> 15;
    packed->Mid |= (evenparity32((packed->Mid & 0x00000001) ^ (packed->Bot & 0xFFFE0000))) << 1;
    packed->Bot |= (oddparity32(packed->Bot & 0x0001FFFE));
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_H10306(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 34) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 16);
    card->CardNumber = get_linear_field(packed, 17, 16);

    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 16))) &&
        (get_bit_by_position(packed, 33) == oddparity32(get_linear_field(packed, 17, 16)));

    return true;
}

static bool Pack_N10002(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFFFF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 34; // Set number of bits
    set_linear_field(packed, card->FacilityCode, 1, 16);
    set_linear_field(packed, card->CardNumber, 17, 16);

    set_bit_by_position(packed,
                        evenparity32(get_linear_field(packed, 1, 16))
                        , 0);
    set_bit_by_position(packed,
                        oddparity32(get_linear_field(packed, 17, 16))
                        , 33);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_N10002(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 34) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 16);
    card->CardNumber = get_linear_field(packed, 17, 16);

    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 16))) &&
        (get_bit_by_position(packed, 33) == oddparity32(get_linear_field(packed, 17, 16)));

    return true;
}

static bool Pack_C1k35s(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFFF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 35; // Set number of bits
    packed->Bot |= (card->CardNumber & 0x000FFFFF) << 1;
    packed->Bot |= (card->FacilityCode & 0x000007FF) << 21;
    packed->Mid |= (card->FacilityCode & 0x00000800) >> 11;
    packed->Mid |= (evenparity32((packed->Mid & 0x1) ^ (packed->Bot & 0xB6DB6DB6))) << 1;
    packed->Bot |= (oddparity32((packed->Mid & 0x3) ^ (packed->Bot & 0x6DB6DB6C)));
    packed->Mid |= (oddparity32((packed->Mid & 0x3) ^ (packed->Bot & 0xFFFFFFFF))) << 2;
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_C1k35s(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 35) return false; // Wrong length? Stop here.

    card->CardNumber = (packed->Bot >> 1) & 0x000FFFFF;
    card->FacilityCode = ((packed->Mid & 1) << 11) | ((packed->Bot >> 21));
    card->ParityValid =
        (evenparity32((packed->Mid & 0x1) ^ (packed->Bot & 0xB6DB6DB6)) == ((packed->Mid >> 1) & 1)) &&
        (oddparity32((packed->Mid & 0x3) ^ (packed->Bot & 0x6DB6DB6C)) == ((packed->Bot >> 0) & 1)) &&
        (oddparity32((packed->Mid & 0x3) ^ (packed->Bot & 0xFFFFFFFF)) == ((packed->Mid >> 2) & 1));
    return true;
}

static bool Pack_H10320(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0) return false; // Can't encode FC. (none in this format)
    if (card->CardNumber > 99999999) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 36; // Set number of bits
    // This card is BCD-encoded rather than binary. Set the 4-bit groups independently.
    for (uint32_t idx = 0; idx < 8; idx++) {
        set_linear_field(packed, (uint64_t)(card->CardNumber / pow(10, 7 - idx)) % 10, idx * 4, 4);
    }
    uint8_t field_data1[] = {0, 4, 8, 12, 16, 20, 24, 28};
    uint8_t field_data2[] = {1, 5, 9, 13, 17, 21, 25, 29};
    uint8_t field_data3[] = {2, 6, 10, 14, 18, 22, 28, 30};
    uint8_t field_data4[] = {3, 7, 11, 15, 19, 23, 29, 31};
    set_bit_by_position(packed, evenparity32(
    get_nonlinear_field(packed, 8, field_data1)
                        ), 32);
    set_bit_by_position(packed, oddparity32(
    get_nonlinear_field(packed, 8, field_data2)
                        ), 33);
    set_bit_by_position(packed, evenparity32(
    get_nonlinear_field(packed, 8, field_data3)
                        ), 34);
    set_bit_by_position(packed, evenparity32(
    get_nonlinear_field(packed, 8, field_data4)
                        ), 35);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_H10320(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 36) return false; // Wrong length? Stop here.

    // This card is BCD-encoded rather than binary. Get the 4-bit groups independently.
    for (uint32_t idx = 0; idx < 8; idx++) {
        uint64_t val = get_linear_field(packed, idx * 4, 4);
        if (val > 9) {
            // Violation of BCD; Zero and exit.
            card->CardNumber = 0;
            return false;
        } else {
            card->CardNumber += val * pow(10, 7 - idx);
        }
    }
    uint8_t field_data1[] = {0, 4, 8, 12, 16, 20, 24, 28};
    uint8_t field_data2[] = {1, 5, 9, 13, 17, 21, 25, 29};
    uint8_t field_data3[] = {2, 6, 10, 14, 18, 22, 28, 30};
    uint8_t field_data4[] = {3, 7, 11, 15, 19, 23, 29, 31};


    card->ParityValid =
    (get_bit_by_position(packed, 32) == evenparity32(get_nonlinear_field(packed, 8, field_data1))) &&
    (get_bit_by_position(packed, 33) ==  oddparity32(get_nonlinear_field(packed, 8, field_data2))) &&
    (get_bit_by_position(packed, 34) == evenparity32(get_nonlinear_field(packed, 8, field_data3))) &&
    (get_bit_by_position(packed, 35) == evenparity32(get_nonlinear_field(packed, 8, field_data4)));
    return true;
}

static bool Pack_S12906(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFF) return false; // Can't encode FC.
    if (card->IssueLevel > 0x03) return false; // Can't encode IL.
    if (card->CardNumber > 0x00FFFFFF) return false; // Can't encode CN.
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 36; // Set number of bits
    set_linear_field(packed, card->FacilityCode, 1, 8);
    set_linear_field(packed, card->IssueLevel, 9, 2);
    set_linear_field(packed, card->CardNumber, 11, 24);
    set_bit_by_position(packed, oddparity32(get_linear_field(packed, 1, 17)), 0);
    set_bit_by_position(packed, oddparity32(get_linear_field(packed, 17, 18)), 35);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_S12906(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 36) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 8);
    card->IssueLevel = get_linear_field(packed, 9, 2);
    card->CardNumber = get_linear_field(packed, 11, 24);
    card->ParityValid =
        (get_bit_by_position(packed, 0) == oddparity32(get_linear_field(packed, 1, 17))) &&
        (get_bit_by_position(packed, 35) == oddparity32(get_linear_field(packed, 17, 18)));
    return true;
}

static bool Pack_Sie36(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x0003FFFF) return false; // Can't encode FC.
    if (card->CardNumber > 0x0000FFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 36; // Set number of bits
    set_linear_field(packed, card->FacilityCode, 1, 18);
    set_linear_field(packed, card->CardNumber, 19, 16);
    uint8_t field_data1[] = {1, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 18, 19, 21, 22, 24, 25, 27, 28, 30, 31, 33, 34};
    set_bit_by_position(packed,
    oddparity32(get_nonlinear_field(packed, 23, field_data1)) , 0);
    uint8_t field_data2[] = {1, 2, 4, 5, 7, 8, 10, 11, 13, 14, 16, 17, 19, 20, 22, 23, 25, 26, 28, 29, 31, 32, 34};
    set_bit_by_position(packed,
    evenparity32(get_nonlinear_field(packed, 23, field_data2))
    , 35);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_Sie36(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 36) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 18);
    card->CardNumber = get_linear_field(packed, 19, 16);
    uint8_t field_data1[] = {1, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 18, 19, 21, 22, 24, 25, 27, 28, 30, 31, 33, 34};
    uint8_t field_data2[] = {1, 2, 4, 5, 7, 8, 10, 11, 13, 14, 16, 17, 19, 20, 22, 23, 25, 26, 28, 29, 31, 32, 34};
    card->ParityValid =
    (get_bit_by_position(packed, 0) == oddparity32(get_nonlinear_field(packed, 23, field_data1))) &&
    (get_bit_by_position(packed, 35) == oddparity32(get_nonlinear_field(packed, 23, field_data2)));
    return true;
}

static bool Pack_C15001(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x000000FF) return false; // Can't encode FC.
    if (card->CardNumber > 0x0000FFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0x000003FF) return false; // Can't encode OEM.

    if (card->OEM == 0)
        card->OEM = 900;

    packed->Length = 36; // Set number of bits
    set_linear_field(packed, card->OEM, 1, 10);
    set_linear_field(packed, card->FacilityCode, 11, 8);
    set_linear_field(packed, card->CardNumber, 19, 16);
    set_bit_by_position(packed, evenparity32(get_linear_field(packed, 1, 17)), 0);
    set_bit_by_position(packed, oddparity32(get_linear_field(packed, 18, 17)), 35);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_C15001(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 36) return false; // Wrong length? Stop here.

    card->OEM = get_linear_field(packed, 1, 10);
    card->FacilityCode = get_linear_field(packed, 11, 8);
    card->CardNumber = get_linear_field(packed, 19, 16);
    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 17))) &&
        (get_bit_by_position(packed, 35) == oddparity32(get_linear_field(packed, 18, 17)));
    return true;
}

static bool Pack_H10302(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0) return false; // Can't encode FC. (none in this format)
    if (card->CardNumber > 0x00000007FFFFFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 37; // Set number of bits
    set_linear_field(packed, card->CardNumber, 1, 35);
    set_bit_by_position(packed, evenparity32(get_linear_field(packed, 1, 18)), 0);
    set_bit_by_position(packed, oddparity32(get_linear_field(packed, 18, 18)), 36);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_H10302(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 37) return false; // Wrong length? Stop here.

    card->CardNumber = get_linear_field(packed, 1, 35);
    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 18))) &&
        (get_bit_by_position(packed, 36) == oddparity32(get_linear_field(packed, 18, 18)));
    return true;
}

static bool Pack_P10004(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x00001FFF) return false; // Can't encode FC.
    if (card->CardNumber > 0x0003FFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 37; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 1, 13);
    set_linear_field(packed, card->CardNumber, 14, 18);
    // unknown parity scheme
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_P10004(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 37) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 13);
    card->CardNumber = get_linear_field(packed, 14, 18);
    // unknown parity scheme
    return true;
}

static bool Pack_H10304(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x0000FFFF) return false; // Can't encode FC.
    if (card->CardNumber > 0x0007FFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 37; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 1, 16);
    set_linear_field(packed, card->CardNumber, 17, 19);

    set_bit_by_position(packed, evenparity32(get_linear_field(packed, 1, 18)), 0);
    set_bit_by_position(packed, oddparity32(get_linear_field(packed, 18, 18)), 36);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_H10304(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 37) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 16);
    card->CardNumber = get_linear_field(packed, 17, 19);
    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 18))) &&
        (get_bit_by_position(packed, 36) == oddparity32(get_linear_field(packed, 18, 18)));
    return true;
}

static bool Pack_HGeneric37(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0) return false; // Not used in this format
    if (card->CardNumber > 0x0007FFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 37; // Set number of bits

    set_linear_field(packed, card->CardNumber, 4, 32);

    set_bit_by_position(packed, 1, 36); // Always 1

    // even1

    uint8_t field_data1[] = {4, 8, 12, 16, 20, 24, 28, 32};
    uint8_t field_data2[] = {6, 10, 14, 18, 22, 26, 30, 34};
    uint8_t field_data3[] = {7, 11, 15, 19, 23, 27, 31, 35};

    set_bit_by_position(packed,
                        evenparity32(
    get_nonlinear_field(packed, 8, field_data1))  , 0  );
    // odd1
    set_bit_by_position(packed,
                        oddparity32(
    get_nonlinear_field(packed, 8, field_data2))  , 2 );
    // even2
    set_bit_by_position(packed,
                        evenparity32(
    get_nonlinear_field(packed, 8, field_data3))  , 3);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_HGeneric37(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 37) return false; // Wrong length? Stop here.
    if (get_bit_by_position(packed, 36) != 1) return false; // Always 1 in this format

    card->CardNumber = get_linear_field(packed, 4, 32);
    uint8_t field_data1[] = {4, 8, 12, 16, 20, 24, 28, 32};
    uint8_t field_data2[] = {6, 10, 14, 18, 22, 28, 30, 34};
    uint8_t field_data3[] = {7, 11, 15, 19, 23, 27, 31, 35};
    card->ParityValid =
    (get_bit_by_position(packed, 0) == evenparity32(get_nonlinear_field(packed, 8, field_data1))) &&
    (get_bit_by_position(packed, 2) ==  oddparity32(get_nonlinear_field(packed, 8, field_data2))) &&
    (get_bit_by_position(packed, 3) == evenparity32(get_nonlinear_field(packed, 8, field_data3)))
    ;
    return true;
}

static bool Pack_MDI37(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {
    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x0000F) return false; // Can't encode FC.
    if (card->CardNumber > 0x1FFFFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 37; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 3, 4);
    set_linear_field(packed, card->CardNumber, 7, 29);

    set_bit_by_position(packed, evenparity32(get_linear_field(packed, 1, 18)), 0);
    set_bit_by_position(packed, oddparity32(get_linear_field(packed, 18, 18)), 36);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_MDI37(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 37) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 3, 4);;
    card->CardNumber = get_linear_field(packed, 7, 29);

    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 18))) &&
        (get_bit_by_position(packed, 36) == oddparity32(get_linear_field(packed, 18, 18)))
        ;
    return true;
}

static bool Pack_P10001(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFFF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 40; // Set number of bits
    set_linear_field(packed, 0xF, 0, 4);
    set_linear_field(packed, card->FacilityCode, 4, 12);
    set_linear_field(packed, card->CardNumber, 16, 16);
    set_linear_field(packed,
                     get_linear_field(packed, 0, 8) ^
                     get_linear_field(packed, 8, 8) ^
                     get_linear_field(packed, 16, 8) ^
                     get_linear_field(packed, 24, 8)
                     , 32, 8);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_P10001(wiegand_message_t *packed, wiegand_card_t *card) {

    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 40) return false; // Wrong length? Stop here.

    card->CardNumber = get_linear_field(packed, 16, 16);
    card->FacilityCode = get_linear_field(packed, 4, 12);
    card->ParityValid = (
                            get_linear_field(packed, 0, 8) ^
                            get_linear_field(packed, 8, 8) ^
                            get_linear_field(packed, 16, 8) ^
                            get_linear_field(packed, 24, 8)
                        ) == get_linear_field(packed, 32, 8);
    return true;
}

static bool Pack_C1k48s(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x003FFFFF) return false; // Can't encode FC.
    if (card->CardNumber > 0x007FFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 48; // Set number of bits
    packed->Bot |= (card->CardNumber & 0x007FFFFF) << 1;
    packed->Bot |= (card->FacilityCode & 0x000000FF) << 24;
    packed->Mid |= (card->FacilityCode & 0x003FFF00) >> 8;
    packed->Mid |= (evenparity32((packed->Mid & 0x00001B6D) ^ (packed->Bot & 0xB6DB6DB6))) << 14;
    packed->Bot |= (oddparity32((packed->Mid & 0x000036DB) ^ (packed->Bot & 0x6DB6DB6C)));
    packed->Mid |= (oddparity32((packed->Mid & 0x00007FFF) ^ (packed->Bot & 0xFFFFFFFF))) << 15;
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_C1k48s(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 48) return false; // Wrong length? Stop here.

    card->CardNumber = (packed->Bot >> 1) & 0x007FFFFF;
    card->FacilityCode = ((packed->Mid & 0x00003FFF) << 8) | ((packed->Bot >> 24));
    card->ParityValid =
        (evenparity32((packed->Mid & 0x00001B6D) ^ (packed->Bot & 0xB6DB6DB6)) == ((packed->Mid >> 14) & 1)) &&
        (oddparity32((packed->Mid & 0x000036DB) ^ (packed->Bot & 0x6DB6DB6C)) == ((packed->Bot >> 0) & 1)) &&
        (oddparity32((packed->Mid & 0x00007FFF) ^ (packed->Bot & 0xFFFFFFFF)) == ((packed->Mid >> 15) & 1));
    return true;
}

static bool Pack_CasiRusco40(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFFFFFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 40; // Set number of bits
    set_linear_field(packed, card->CardNumber, 1, 38);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_CasiRusco40(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 40) return false; // Wrong length? Stop here.

    card->CardNumber = get_linear_field(packed, 1, 38);
    return true;
}

static bool Pack_Optus(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x3FF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 34; // Set number of bits
    set_linear_field(packed, card->CardNumber, 1, 16);
    set_linear_field(packed, card->FacilityCode, 22, 11);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_Optus(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 34) return false; // Wrong length? Stop here.

    card->CardNumber = get_linear_field(packed, 1, 16);
    card->FacilityCode = get_linear_field(packed, 22, 11);
    return true;
}

static bool Pack_Smartpass(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x3FF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0x7) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 34; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 1, 13);
    set_linear_field(packed, card->IssueLevel, 14, 3);
    set_linear_field(packed, card->CardNumber, 17, 16);
    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_Smartpass(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 34) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 13);
    card->IssueLevel = get_linear_field(packed, 14, 3);
    card->CardNumber = get_linear_field(packed, 17, 16);
    return true;
}

static bool Pack_bqt34(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 34; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 1, 8);
    set_linear_field(packed, card->CardNumber, 9, 24);

    set_bit_by_position(packed,
                        evenparity32(get_linear_field(packed, 1, 16))
                        , 0);
    set_bit_by_position(packed,
                        oddparity32(get_linear_field(packed, 17, 16))
                        , 33);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_bqt34(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 34) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 8);
    card->CardNumber = get_linear_field(packed, 9, 24);

    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 16))) &&
        (get_bit_by_position(packed, 33) == oddparity32(get_linear_field(packed, 17, 16)));
    return true;
}

static bool Pack_bqt38(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFFF) return false; // 12 bits
    if (card->CardNumber > 0x3FFFF) return false; // 19 bits
    if (card->IssueLevel > 0x7) return false; // 4 bit
    if (card->OEM > 0) return false; // Not used in this format

    packed->Length = 38; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 24, 13);
    set_linear_field(packed, card->CardNumber, 1, 19);
    set_linear_field(packed, card->IssueLevel, 20, 4);

    set_bit_by_position(packed,
                        evenparity32(get_linear_field(packed, 1, 18))
                        , 0);
    set_bit_by_position(packed,
                        oddparity32(get_linear_field(packed, 19, 18))
                        , 37);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_bqt38(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 38) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 24, 13);
    card->CardNumber = get_linear_field(packed, 1, 19);
    card->IssueLevel = get_linear_field(packed, 20, 4);

    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 18))) &&
        (get_bit_by_position(packed, 37) == oddparity32(get_linear_field(packed, 19, 18)));
    return true;
}

static bool Pack_iscs38(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0x3FF) return false; // 12 bits
    if (card->CardNumber > 0xFFFFFF) return false; // 19 bits
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0x7) return false; // 4 bit

    packed->Length = 38; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 5, 10);
    set_linear_field(packed, card->CardNumber, 15, 22);
    set_linear_field(packed, card->IssueLevel, 1, 4);

    set_bit_by_position(packed,
                        evenparity32(get_linear_field(packed, 1, 18))
                        , 0);
    set_bit_by_position(packed,
                        oddparity32(get_linear_field(packed, 19, 18))
                        , 37);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_iscs38(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 38) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 5, 10);
    card->CardNumber = get_linear_field(packed, 15, 22);
    card->OEM = get_linear_field(packed, 1, 4);

    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 18))) &&
        (get_bit_by_position(packed, 37) == oddparity32(get_linear_field(packed, 19, 18)));
    return true;
}

static bool Pack_pw39(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFFFF) return false; // 12 bits
    if (card->CardNumber > 0xFFFFF) return false; // 19 bits
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0) return false; // 4 bit

    packed->Length = 39; // Set number of bits

    set_linear_field(packed, card->FacilityCode, 1, 17);
    set_linear_field(packed, card->CardNumber, 18, 20);

    set_bit_by_position(packed,
                        evenparity32(get_linear_field(packed, 1, 18))
                        , 0);
    set_bit_by_position(packed,
                        oddparity32(get_linear_field(packed, 19, 19))
                        , 38);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_pw39(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 39) return false; // Wrong length? Stop here.

    card->FacilityCode = get_linear_field(packed, 1, 17);
    card->CardNumber = get_linear_field(packed, 18, 20);

    card->ParityValid =
        (get_bit_by_position(packed, 0) == evenparity32(get_linear_field(packed, 1, 18))) &&
        (get_bit_by_position(packed, 38) == oddparity32(get_linear_field(packed, 19, 19)));
    return true;
}


static bool Pack_bc40(wiegand_card_t *card, wiegand_message_t *packed, bool preamble) {

    memset(packed, 0, sizeof(wiegand_message_t));

    if (card->FacilityCode > 0xFFF) return false; // Can't encode FC.
    if (card->CardNumber > 0xFFFFF) return false; // Can't encode CN.
    if (card->IssueLevel > 0) return false; // Not used in this format
    if (card->OEM > 0x7F) return false; // Not used in this format

    packed->Length = 39; // Set number of bits

    set_linear_field(packed, card->OEM, 0, 7);

    // cost center 12
    set_linear_field(packed, card->FacilityCode, 7, 12);
    set_linear_field(packed, card->CardNumber, 19, 19);

    set_bit_by_position(packed,
                        oddparity32(get_linear_field(packed, 19, 19))
                        , 39);

    if (preamble)
        return add_HID_header(packed);
    return true;
}

static bool Unpack_bc40(wiegand_message_t *packed, wiegand_card_t *card) {
    memset(card, 0, sizeof(wiegand_card_t));

    if (packed->Length != 39) return false; // Wrong length? Stop here.

    card->OEM = get_linear_field(packed, 0, 7);
    card->FacilityCode = get_linear_field(packed, 7, 12);
    card->CardNumber = get_linear_field(packed, 19, 19);

    card->ParityValid =
        (get_bit_by_position(packed, 39) == oddparity32(get_linear_field(packed, 19, 19)));
    return true;
}
static const cardformat_t FormatTable[] = {
    {"H10301",  Pack_H10301,  Unpack_H10301,  "HID H10301 26-bit",          {1, 1, 0, 0, 1}}, // imported from old pack/unpack
    {"ind26",   Pack_ind26,   Unpack_ind26,   "Indala 26-bit",              {1, 1, 0, 0, 1}}, // from cardinfo.barkweb.com.au
    {"ind27",   Pack_ind27,   Unpack_ind27,   "Indala 27-bit",              {1, 1, 0, 0, 0}}, // from cardinfo.barkweb.com.au
    {"indasc27", Pack_indasc27, Unpack_indasc27, "Indala ASC 27-bit",       {1, 1, 0, 0, 0}}, // from cardinfo.barkweb.com.au
    {"Tecom27", Pack_Tecom27, Unpack_Tecom27, "Tecom 27-bit",               {1, 1, 0, 0, 1}}, // from cardinfo.barkweb.com.au
    {"2804W",   Pack_2804W,   Unpack_2804W,   "2804 Wiegand 28-bit",        {1, 1, 0, 0, 1}}, // from cardinfo.barkweb.com.au
    {"ind29",   Pack_ind29,   Unpack_ind29,   "Indala 29-bit",              {1, 1, 0, 0, 0}}, // from cardinfo.barkweb.com.au
    {"ATSW30",  Pack_ATSW30,  Unpack_ATSW30,  "ATS Wiegand 30-bit",         {1, 1, 0, 0, 1}}, // from cardinfo.barkweb.com.au
    {"ADT31",   Pack_ADT31,   Unpack_ADT31,   "HID ADT 31-bit",             {1, 1, 0, 0, 0}}, // from cardinfo.barkweb.com.au
    {"HCP32",   Pack_hcp32,   Unpack_hcp32,   "HID Check Point 32-bit",     {1, 1, 0, 0, 0}}, // from cardinfo.barkweb.com.au
    {"HPP32",   Pack_hpp32,   Unpack_hpp32,   "HID Hewlett-Packard 32-bit", {1, 1, 0, 0, 0}}, // from cardinfo.barkweb.com.au
    {"Kastle",  Pack_Kastle,  Unpack_Kastle,  "Kastle 32-bit",              {1, 1, 1, 0, 1}}, // from @xilni; PR #23 on RfidResearchGroup/proxmark3
    {"Kantech", Pack_Kantech, Unpack_Kantech, "Indala/Kantech KFS 32-bit",  {1, 1, 0, 0, 0}}, // from cardinfo.barkweb.com.au
    {"WIE32",   Pack_wie32,   Unpack_wie32,   "Wiegand 32-bit",             {1, 1, 0, 0, 0}}, // from cardinfo.barkweb.com.au
    {"D10202",  Pack_D10202,  Unpack_D10202,  "HID D10202 33-bit",          {1, 1, 0, 0, 1}}, // from cardinfo.barkweb.com.au
    {"H10306",  Pack_H10306,  Unpack_H10306,  "HID H10306 34-bit",          {1, 1, 0, 0, 1}}, // imported from old pack/unpack
    {"N10002",  Pack_N10002,  Unpack_N10002,  "Honeywell/Northern N10002 34-bit", {1, 1, 0, 0, 1}}, // from proxclone.com
    {"Optus34", Pack_Optus,   Unpack_Optus,   "Indala Optus 34-bit",        {1, 1, 0, 0, 0}}, // from cardinfo.barkweb.com.au
    {"SMP34",   Pack_Smartpass, Unpack_Smartpass, "Cardkey Smartpass 34-bit", {1, 1, 1, 0, 0}}, // from cardinfo.barkweb.com.au
    {"BQT34",   Pack_bqt34,   Unpack_bqt34,   "BQT 34-bit",                 {1, 1, 0, 0, 1}}, // from cardinfo.barkweb.com.au
    {"C1k35s",  Pack_C1k35s,  Unpack_C1k35s,  "HID Corporate 1000 35-bit std", {1, 1, 0, 0, 1}}, // imported from old pack/unpack
    {"C15001",  Pack_C15001,  Unpack_C15001,  "HID KeyScan 36-bit",         {1, 1, 0, 1, 1}}, // from Proxmark forums
    {"S12906",  Pack_S12906,  Unpack_S12906,  "HID Simplex 36-bit",         {1, 1, 1, 0, 1}}, // from cardinfo.barkweb.com.au
    {"Sie36",   Pack_Sie36,   Unpack_Sie36,   "HID 36-bit Siemens",         {1, 1, 0, 0, 1}}, // from cardinfo.barkweb.com.au
    {"H10320",  Pack_H10320,  Unpack_H10320,  "HID H10320 36-bit BCD",      {1, 0, 0, 0, 1}}, // from Proxmark forums
    {"H10302",  Pack_H10302,  Unpack_H10302,  "HID H10302 37-bit huge ID",  {1, 0, 0, 0, 1}}, // from Proxmark forums
    {"H10304",  Pack_H10304,  Unpack_H10304,  "HID H10304 37-bit",          {1, 1, 0, 0, 1}}, // from cardinfo.barkweb.com.au
    {"P10004",  Pack_P10004,  Unpack_P10004,  "HID P10004 37-bit PCSC",     {1, 1, 0, 0, 0}}, // from @bthedorff; PR #1559
    {"HGen37",  Pack_HGeneric37, Unpack_HGeneric37,  "HID Generic 37-bit", {1, 0, 0, 0, 1}}, // from cardinfo.barkweb.com.au
    {"MDI37",   Pack_MDI37,   Unpack_MDI37,   "PointGuard MDI 37-bit",         {1, 1, 0, 0, 1}}, // from cardinfo.barkweb.com.au
    {"BQT38",   Pack_bqt38,   Unpack_bqt38,   "BQT 38-bit",                    {1, 1, 1, 0, 1}}, // from cardinfo.barkweb.com.au
    {"ISCS",    Pack_iscs38,  Unpack_iscs38,  "ISCS 38-bit",                   {1, 1, 0, 1, 1}}, // from cardinfo.barkweb.com.au
    {"PW39",    Pack_pw39,    Unpack_pw39,    "Pyramid 39-bit wiegand format", {1, 1, 0, 0, 1}},  // from cardinfo.barkweb.com.au
    {"P10001",  Pack_P10001,  Unpack_P10001,  "HID P10001 Honeywell 40-bit",   {1, 1, 0, 1, 0}}, // from cardinfo.barkweb.com.au
    {"Casi40",  Pack_CasiRusco40, Unpack_CasiRusco40, "Casi-Rusco 40-bit",     {1, 0, 0, 0, 0}}, // from cardinfo.barkweb.com.au
    {"C1k48s",  Pack_C1k48s,  Unpack_C1k48s,  "HID Corporate 1000 48-bit std", {1, 1, 0, 0, 1}}, // imported from old pack/unpack
    {"BC40",    Pack_bc40,    Unpack_bc40,    "Bundy TimeClock 40-bit",     {1, 1, 0, 1, 1}}, // from
    {NULL, NULL, NULL, NULL, {0, 0, 0, 0, 0}} // Must null terminate array
};

const cardInfo_t cardinfotable[]={
    { "26", "8", "16", "None", "None", "1", "24", "1, 7, 13, 19", "13", " 12", "2-9, 12-17, 20-25"},
    { "26", "16", " 16", "None", "None", "9", "24", "2,8, 14, 20", "14", " 24", "2-9, 12-17, 20-25"},
    { "27", "20", " 20", "None", "None", "1", "24", "1, 7, 13, 20", "14", " 26", "2-9, 12-17, 20-26"},
    { "27", "16", " 24", "None", "None", "1", "24", "1, 7, 13", "14", " 26", "2-9, 12-17, 20-26"},
    { "27", "16", " 24", "None", "None", "1", "24", "5, 11, 17, 23", "14", " 26", "2-9, 12-17, 20-27"},
    { "28", "8", " 16", "None", "None", "1", "24", "2, 6, 10, 14, 18, 22, 26, 28", "14", " 12", "2-9, 12-17, 20-28"},
    { "29", "24", " 24", "None", "None", "1", "24", "1, 8, 15, 22, 29", "14", " 28", "2-9, 12-17, 20-29"},
    { "30", "8", " 16", "None", "None", "1", "24", "1, 30", "14", " 12", " 2-9, 12-17, 20-30"},
    { "31", " 8 ", " 16", "None", "None", "1", "24", "3,9,15,21", "14", " 12", " 2-9, 12-17, 20-31"},
    { "32", " 8 ", " 16", "None", "None", "1", "24", "5, 12, 19, 26, 32", "14", " 12", "2-9, 12-17, 20-31"},
    { "32", " 8 ", " 16", "None", "None", "1", "24", "5, 12, 19, 26, 32", "14", " 12", "2-9, 12-17, 20-31"},
    { "32", " 8 ", " 16", "None", "None", "1", "24", " 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37", "14", " 12", "2-9, 12-17, 20-31"},
    { "32", " 16", " 16", "None", "None", "1", "24", "1, 8, 17, 26", "14", " 26", "2-9, 12-17, 20-31"},
    { "32", " 8 ", " 16", "None", "None", "1", "24", "2, 9, 16, 23", "14", " 12", "2-9, 12-17, 20-31"},
    { "33", " 8 ", " 16", "8", "24", "1", "24", "2, 6, 10, 14, 18, 22, 26, 30", "14", " 12", "2-9, 12-17, 20-32"},
    { "34", " 8 ", " 16", "8", "24", "1", "24", "3, 9, 15, 21, 27, 30, 33, 34", "27", " 12", "2-9, 12-17, 20-33"},
    { "34", " 8 ", " 16", "8", "24", "1", "24", "1, 3, 5, 7, 9, 11, 13, 15, 17, 19", "27", " 12", "2-9, 12-17, 20-33"},
    { "34", " 16", " 20", "8", "24", "1", "24", "2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24", "27", " 26", "2-9, 12-17, 20-33"},
    { "34", " 8 ", " 20", "8", "24", "1", "24", "2, 6, 10, 14, 18, 22, 26, 30", "27", " 12", "2-9, 12-17, 20-33"},
    { "34", " 16", " 20", "8", "24", "1", "24", "3, 5, 7, 9, 11, 30, 31, 32", "27", " 12", "2-9, 12-17, 20-33"},
    { "35", " 16", " 20", "8", "20", "1", "24", "3, 5, 7, 9, 11, 13, 31, 33", "14", " 12", "2-9, 12-17, 20-34"},
    { "36", " 8 ", " 20", "8", "24", "1", "24", "3, 9, 15, 21, 27, 30, 32, 36", "35", " 12", "2-9, 12-17, 20-35"},
    { "36", " 8 ", " 20", "8", "24", "1", "24", "2, 8, 18, 24", "35", " 12", "2-9, 12-17, 20-35"},
    { "36", " 8 ", " 20", "8", "24", "1", "24", "3, 9, 15, 21, 27, 30, 32, 36", "35", " 12", "2-9, 12-17, 20-35"},
    { "36", " 8 ", " 20", "8", "24", "1", "24", "4, 12, 20, 28", "35", " 12", "2-9, 12-17, 20-35"},
    { "37", "16", " 24", "16", "24", "1", "32", "3, 5, 7, 9, 11, 13, 15, 17", "27", " 12", "2-9, 12-17, 20-36"},
    { "37", "8 ", " 24", "16", "24", "1", "32", "3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25", "27", " 12", "2-9, 12-17, 20-36"},
    { "37", "8 ", " 24", "16", "24", "1", "32", "3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25", "27", " 12", "2-9, 12-17, 20-36"},
    { "37", "16", " 24", "16", "24", "1", "32", "3, 5, 7, 9, 11, 13, 15, 17", "27", " 12", "2-9, 12-17, 20-36"},
    { "37", "8 ", " 24", "16", "24", "1", "32", "4, ,6 , 8, 10, 12, 14, 16, 18", "27", " 12", "2-9, 12-17, 20-36"},
    { "38", "16", " 24", "8", "24", "1", "32", "3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25", "27", " 12", " 2-9, 12-17, 20-37"},
    { "38", "16", " 24", "8", "24", "1", "32", "2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24", "27", " 12", " 2-9, 12-17, 20-37"},
    { "39", "16", " 24", "16", "24", "1", "32", "3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25", "27", " 12", "2-9, 12-17, 20-38"},
    { "40", "16", " 24", "16", "24", "1", "32", "3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33", "27", " 12", "2-9, 12-17, 20-39"},
    { "40", "16", " 24", "16", "24", "1", "32", "1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31", "27", " 12", "2-9, 12-17, 20-39"},
    { "48", "16", " 24", "16", "24", "1", "32", "3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51", "14", " 12", "2-9, 12-17, 20-27, 30-37, 40-47"},
    { "40", "16", " 24", "16", "24", "1", "32", " 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32", "21", " 12", "2-9, 12-17, 20-39"}

};

static void hid_print_card(wiegand_card_t *card, const cardformat_t format, const cardInfo_t table) {

    /*
        PrintAndLogEx(SUCCESS, "       Format: %s (%s)", format.Name, format.Descrp);

        if (format.Fields.hasFacilityCode)
            PrintAndLogEx(SUCCESS, "Facility Code: %d",card->FacilityCode);

        if (format.Fields.hasCardNumber)
            PrintAndLogEx(SUCCESS, "  Card Number: %d",card->CardNumber);

        if (format.Fields.hasIssueLevel)
            PrintAndLogEx(SUCCESS, "  Issue Level: %d",card->IssueLevel);

        if (format.Fields.hasOEMCode)
            PrintAndLogEx(SUCCESS, "     OEM Code: %d",card->OEM);

        if (format.Fields.hasParity)
            PrintAndLogEx(SUCCESS, "       Parity: %s",card->ParityValid ? "Valid" : "Invalid");
    */
    //printf("%s\t %s\t %s\t %s\t %s\n","CardName", "FacilityCode",  "CardNumber", "Issue", "OEM");
    printf("CardName: %s\n", format.Name);
    char s[110] = {0};
    if (format.Fields.hasFacilityCode)
        //snprintf(s, sizeof(s), "FacilityCode: %u\n", card->FacilityCode);
        printf("\tFacilityCode: %u\n", card->FacilityCode);
    else printf("\tFacilityCode: %s\n", "None");

    if (format.Fields.hasCardNumber)
        //snprintf(s + strlen(s), sizeof(s) - strlen(s), "  CardNumber: %I64d", (__int64)card->CardNumber);
        printf("\tCardNumber: %u\n", card->CardNumber);
    else printf("\tCardNumber: %s\n", "None");

    if (format.Fields.hasIssueLevel)
        //snprintf(s + strlen(s), sizeof(s) - strlen(s), "  Issue: %u", card->IssueLevel);
        printf("\tIssueLevel:%u\n", card->IssueLevel);
    else printf("\tIssueLevel: %s\n", "None");

    if (format.Fields.hasOEMCode)
      //  snprintf(s + strlen(s), sizeof(s) - strlen(s), "  OEM: %u", card->OEM);
        printf("\tOEM: %u\n", card->OEM);
    else printf("\tOEM: %s\n", "None");

    if (format.Fields.hasParity)
        //snprintf(s + strlen(s), sizeof(s) - strlen(s), "  parity ( %s )", card->ParityValid ? "ok" : "fail");
        printf("\tParity: %s\n", "ok");
    else printf("\tParity: %s\n", "None");

    //printf("SUCCESS, [%-8s] %-32s %s\n", format.Name, format.Descrp, s);

    printf("\tBits: %s\n", table.Bits);
    printf("\tSite Code Offset: %s\n", table.SOffset);
    printf("\tSite Code Length: %s\n", table.Slength);
    printf("\tIssue Number Offset: %s\n", table.IOffset);
    printf("\tIssue Number Length:%s\n", table.Ilength);
    printf("\tCard Number Offset: %s\n", table.COffset);
    printf("\tCard Number Length: %s\n", table.Clength);
    printf("\tEven Parity Position: %s\n", table.EvenP);
    printf("\tBits for even Parity mask/position: %s\n", table.Evenmp);
    printf("\tOdd Parity Position: %s\n", table.OddnP);
    printf("\tBits for odd Parity mask/position: %s\n", table.Oddmp);

    printf("\n");
}
bool HIDTryUnpack(wiegand_message_t *packed) {
    if (FormatTable[0].Name == NULL)
        return false;

    int i = 0;
    wiegand_card_t card;
    memset(&card, 0, sizeof(wiegand_card_t));
    uint8_t found_cnt = 0, found_invalid_par = 0;

    while (FormatTable[i].Name) {
        if (FormatTable[i].Unpack(packed, &card)) {

            found_cnt++;
            hid_print_card(&card, FormatTable[i], cardinfotable[i]);

            if (FormatTable[i].Fields.hasParity || card.ParityValid == false)
                found_invalid_par++;
        }
        ++i;
    }

    if (found_cnt) {
        //PrintAndLogEx(INFO, "found %u matching format%c", found_cnt, (found_cnt > 1) ? 's' : ' ');
        printf("INFO: found %u matching format%c\n", found_cnt, (found_cnt > 1) ? 's' : ' ');
    }

    if (packed->Length && found_invalid_par == 0) {
        //PrintAndLogEx(WARNING, "Wiegand unknown bit len %d", packed->Length);
        printf("Warning: Wiegand unknown bit len %d\n", packed->Length);
        //PrintAndLogEx(HINT, "Try 0xFFFF's http://cardinfo.barkweb.com.au/");
        printf("HINT: Try 0xFFFF's http://cardinfo.barkweb.com.au/\n");
    }

    return ((found_cnt - found_invalid_par) > 0);
}

int getDecodedData(char *data, int type){
    uint32_t top = 0, mid = 0, bot = 0;
    int len = 0, res;
    len = strlen(data);
    wiegand_message_t packed;
    char hex[40];
    switch(type){
    case BIN:
        if(len){
            res = binstring_to_u96(&top, &mid, &bot, data);
            //printf("top: %d mid: %d bot: %d\n",top, mid, bot);
            if(res != len){
                printf("ERR Binary string contains none <0|1> chars\n");
                return PM3_EINVARG;
            }
        }else{
            printf("ERR empty input");
            return PM3_EINVARG;
        }
        packed = initialize_message_object(top, mid, bot, len);
        HIDTryUnpack(&packed);
        break;
    case HEX:
        if(len){
            res = hexstring_to_u96(&top, &mid, &bot, data);
            //printf("top: %d mid: %d bot: %d\n",top, mid, bot);
            if(res != len){
                printf("hex string contains none hex chars");
                return PM3_EINVARG;
            }
        }else{
            printf("ERR empty input");
            return PM3_EINVARG;
        }
        packed = initialize_message_object(top, mid, bot, len * 4);
        HIDTryUnpack(&packed);
        break;
    case DEC:
        __int64 value = 0ll;
        __int64 p = 1ll;
        __int64 temp;
        for(int i = 0; i < len; i++){
            value += (data[len - i -1]-'0') * p;
            p *= 10;
        }
        sprintf(hex, "%x", value);
        len = strlen(hex);
        res = hexstring_to_u96(&top, &mid, &bot, data);
        packed = initialize_message_object(top, mid, bot, len * 4);
        HIDTryUnpack(&packed);
        break;
    }

    return PM3_SUCCESS;
}

int main(){
    char cmd[MAX_TYPE_LEN], strvalue[MAX_VLAUE_LEN];
    while(printf("Enter the type of value you want to enter: ")){
        gets(cmd);
        if(strcmp(cmd, "exit") == 0){
            break;
        }else if(strcmp(cmd, "bin") == 0){
            printf("Enter the value: ");
            gets(strvalue);
            getDecodedData(strvalue, BIN);
        }else if(strcmp(cmd, "hex") == 0){
            printf("Enter the value: ");
            gets(strvalue);
            getDecodedData(strvalue, HEX);
        }else if(strcmp(cmd, "dec") == 0){
            printf("Enter the value: ");
            gets(strvalue);
            getDecodedData(strvalue, DEC);
        }else{
            continue;
        }
    }
    return 0;
}
