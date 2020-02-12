#pragma once

#include "hebi.h"

#include "bit_set.hpp"
#include "color.hpp"

namespace hebi {

HebiStatusCode floatGetter(const HebiCommandRef& ref, int field, float* value);
HebiStatusCode floatGetter(const HebiFeedbackRef& ref, int field, float* value);
HebiStatusCode floatGetter(const HebiInfoRef& ref, int field, float* value);

HebiStatusCode highResAngleGetter(const HebiCommandRef& ref, int field, int64_t* revs, float* offset);
HebiStatusCode highResAngleGetter(const HebiFeedbackRef& ref, int field, int64_t* revs, float* offset);
HebiStatusCode highResAngleGetter(const HebiInfoRef& ref, int field, int64_t* revs, float* offset);

HebiStatusCode numberedFloatGetter(const HebiCommandRef& ref, int field, size_t number, float* value);
HebiStatusCode numberedFloatGetter(const HebiFeedbackRef& ref, int field, size_t number, float* value);
HebiStatusCode numberedFloatGetter(const HebiInfoRef& ref, int field, size_t number, float* value);

HebiStatusCode vector3fGetter(const HebiCommandRef& ref, int field, HebiVector3f* value);
HebiStatusCode vector3fGetter(const HebiFeedbackRef& ref, int field, HebiVector3f* value);
HebiStatusCode vector3fGetter(const HebiInfoRef& ref, int field, HebiVector3f* value);

HebiStatusCode quaternionfGetter(const HebiCommandRef& ref, int field, HebiQuaternionf* value);
HebiStatusCode quaternionfGetter(const HebiFeedbackRef& ref, int field, HebiQuaternionf* value);
HebiStatusCode quaternionfGetter(const HebiInfoRef& ref, int field, HebiQuaternionf* value);

HebiStatusCode floatIoPinGetter(const HebiCommandRef& ref, int index, size_t pin_number, float* value);
HebiStatusCode floatIoPinGetter(const HebiFeedbackRef& ref, int index, size_t pin_number, float* value);
HebiStatusCode floatIoPinGetter(const HebiInfoRef& ref, int index, size_t pin_number, float* value);

HebiStatusCode intIoPinGetter(const HebiCommandRef& ref, int index, size_t pin_number, int64_t* value);
HebiStatusCode intIoPinGetter(const HebiFeedbackRef& ref, int index, size_t pin_number, int64_t* value);
HebiStatusCode intIoPinGetter(const HebiInfoRef& ref, int index, size_t pin_number, int64_t* value);

bool flagGetter(const HebiCommandRef& ref, int field);
bool flagGetter(const HebiFeedbackRef& ref, int field);
bool flagGetter(const HebiInfoRef& ref, int field);

HebiStatusCode boolGetter(const HebiCommandRef& ref, int field, bool* value);
HebiStatusCode boolGetter(const HebiFeedbackRef& ref, int field, bool* value);
HebiStatusCode boolGetter(const HebiInfoRef& ref, int field, bool* value);

HebiStatusCode enumGetter(const HebiCommandRef& ref, int field, int32_t* value);
HebiStatusCode enumGetter(const HebiFeedbackRef& ref, int field, int32_t* value);
HebiStatusCode enumGetter(const HebiInfoRef& ref, int field, int32_t* value);

HebiStatusCode uint64Getter(const HebiCommandRef& ref, int field, uint64_t* value);
HebiStatusCode uint64Getter(const HebiFeedbackRef& ref, int field, uint64_t* value);
HebiStatusCode uint64Getter(const HebiInfoRef& ref, int field, uint64_t* value);

HebiStatusCode ledGetter(const HebiCommandRef ref, int field, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* a);
HebiStatusCode ledGetter(const HebiFeedbackRef ref, int field, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* a);
HebiStatusCode ledGetter(const HebiInfoRef ref, int field, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* a);

void hebiCommandSetFloat(HebiCommandRef& command, HebiCommandFloatField field, const float* value);

void hebiCommandSetHighResAngle(HebiCommandRef& command, HebiCommandHighResAngleField field,
                                       const int64_t* int_part, const float* dec_part);

void hebiCommandSetNumberedFloat(HebiCommandRef& command, HebiCommandNumberedFloatField field, size_t number,
                                        const float* value);

void hebiCommandSetIoPinFloat(HebiCommandRef& command, HebiCommandIoPinBank bank, size_t pin_number,
                                     const float* value);

void hebiCommandSetIoPinInt(HebiCommandRef& command, HebiCommandIoPinBank bank, size_t pin_number,
                                   const int64_t* value);

void hebiCommandSetFlag(HebiCommandRef& command, HebiCommandFlagField field, int32_t value);

void hebiCommandSetBool(HebiCommandRef& command, HebiCommandBoolField field, const int32_t* value);

void hebiCommandSetEnum(HebiCommandRef& command, HebiCommandEnumField field, const int32_t* value);

void hebiCommandSetLed(HebiCommandRef& command, HebiCommandLedField field, const Color* color);

} // namespace hebi