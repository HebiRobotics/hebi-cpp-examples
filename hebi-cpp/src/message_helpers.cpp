#include "message_helpers.hpp"

// Statically initialize the metadata for each message type.
static HebiCommandMetadata command_metadata;
static HebiFeedbackMetadata feedback_metadata;
static HebiInfoMetadata info_metadata;

struct Init {
  Init() {
    hebiCommandGetMetadata(&command_metadata);
    hebiFeedbackGetMetadata(&feedback_metadata);
    hebiInfoGetMetadata(&info_metadata);
  }
};

static Init initter;

namespace hebi {

template<typename RefT, typename MetadataT>
HebiStatusCode floatGetter(const RefT& ref, MetadataT& metadata, int field, float* value) {
  auto index = static_cast<int32_t>(field);
  if (index > metadata.float_field_count_ || index < 0) {
    return HebiStatusInvalidArgument;
  }
  auto has_offset = static_cast<size_t>(index + metadata.float_field_bitfield_offset_);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  if (!has_bits.get(has_offset)) {
    return HebiStatusValueNotSet;
  }
  if (value != nullptr) {
    *value = ref.float_fields_[index];
  }
  return HebiStatusSuccess;
}

HebiStatusCode floatGetter(const HebiCommandRef& ref, int field, float* value) {
  return floatGetter(ref, command_metadata, field, value);
}

HebiStatusCode floatGetter(const HebiFeedbackRef& ref, int field, float* value) {
  return floatGetter(ref, feedback_metadata, field, value);
}

HebiStatusCode floatGetter(const HebiInfoRef& ref, int field, float* value) {
  return floatGetter(ref, info_metadata, field, value);
}

template<typename RefT, typename MetadataT>
HebiStatusCode highResAngleGetter(const RefT& ref, MetadataT& metadata, int field, int64_t* revs, float* offset) {
  auto index = static_cast<int32_t>(field);
  if (index > metadata.high_res_angle_field_count_ || index < 0) {
    return HebiStatusInvalidArgument;
  }
  auto has_offset = static_cast<size_t>(index + metadata.high_res_angle_field_bitfield_offset_);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  if (!has_bits.get(has_offset)) {
    return HebiStatusValueNotSet;
  }
  if (revs != nullptr && offset != nullptr) {
    const auto& val = ref.high_res_angle_fields_[index];
    *revs = val.revolutions_;
    *offset = val.offset_;
  }
  return HebiStatusSuccess;
}

HebiStatusCode highResAngleGetter(const HebiCommandRef& ref, int field, int64_t* revs, float* offset) {
  return highResAngleGetter(ref, command_metadata, field, revs, offset);
}

HebiStatusCode highResAngleGetter(const HebiFeedbackRef& ref, int field, int64_t* revs, float* offset) {
  return highResAngleGetter(ref, feedback_metadata, field, revs, offset);
}

HebiStatusCode highResAngleGetter(const HebiInfoRef& ref, int field, int64_t* revs, float* offset) {
  return highResAngleGetter(ref, info_metadata, field, revs, offset);
}

template<typename RefT, typename MetadataT>
HebiStatusCode numberedFloatGetter(const RefT& ref, MetadataT& metadata, int field, size_t number, float* value) {
  auto index = static_cast<int32_t>(field);
  if (index > metadata.numbered_float_field_count_ || index < 0) {
    return HebiStatusInvalidArgument;
  }
  if (number == 0 || number > metadata.numbered_float_field_sizes_[index] ) {
    return HebiStatusValueNotSet;
  }
  const auto relativeOffset = static_cast<size_t>(metadata.numbered_float_relative_offsets_[index] + number - 1);
  const auto has_offset = static_cast<size_t>(metadata.numbered_float_field_bitfield_offset_ + relativeOffset);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  if (!has_bits.get(has_offset)) {
    return HebiStatusValueNotSet;
  }
  if (value != nullptr) {
    *value = ref.numbered_float_fields_[relativeOffset];
  }
  return HebiStatusSuccess;
}

HebiStatusCode numberedFloatGetter(const HebiCommandRef& ref, int field, size_t number, float* value) {
  return numberedFloatGetter(ref, command_metadata, field, number, value);
}

HebiStatusCode numberedFloatGetter(const HebiFeedbackRef& ref, int field, size_t number, float* value) {
  return numberedFloatGetter(ref, feedback_metadata, field, number, value);
}

HebiStatusCode numberedFloatGetter(const HebiInfoRef& ref, int field, size_t number, float* value) {
  return numberedFloatGetter(ref, info_metadata, field, number, value);
}

template<typename RefT, typename MetadataT>
HebiStatusCode vector3fGetter(const RefT& ref, MetadataT& metadata, int field, HebiVector3f* value) {
  auto index = static_cast<int32_t>(field);
  if (index > metadata.vector3f_field_count_ || index < 0) {
    return HebiStatusInvalidArgument;
  }
  auto has_offset = static_cast<size_t>(index + metadata.vector3f_field_bitfield_offset_);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  if (!has_bits.get(has_offset)) {
    return HebiStatusValueNotSet;
  }
  if (value != nullptr) {
    *value = ref.vector3f_fields_[index];
  }
  return HebiStatusSuccess;
}

HebiStatusCode vector3fGetter(const HebiCommandRef& ref, int field, HebiVector3f* value) {
  return vector3fGetter(ref, command_metadata, field, value);
}

HebiStatusCode vector3fGetter(const HebiFeedbackRef& ref, int field, HebiVector3f* value) {
  return vector3fGetter(ref, feedback_metadata, field, value);
}

HebiStatusCode vector3fGetter(const HebiInfoRef& ref, int field, HebiVector3f* value) {
  return vector3fGetter(ref, info_metadata, field, value);
}

template<typename RefT, typename MetadataT>
HebiStatusCode quaternionfGetter(const RefT& ref, MetadataT& metadata, int field, HebiQuaternionf* value) {
  auto index = static_cast<int32_t>(field);
  if (index > metadata.quaternionf_field_count_ || index < 0) {
    return HebiStatusInvalidArgument;
  }
  auto has_offset = static_cast<size_t>(index + metadata.quaternionf_field_bitfield_offset_);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  if (!has_bits.get(has_offset)) {
    return HebiStatusValueNotSet;
  }
  if (value != nullptr) {
    *value = ref.quaternionf_fields_[index];
  }
  return HebiStatusSuccess;
}

HebiStatusCode quaternionfGetter(const HebiCommandRef& ref, int field, HebiQuaternionf* value) {
  return quaternionfGetter(ref, command_metadata, field, value);
}

HebiStatusCode quaternionfGetter(const HebiFeedbackRef& ref, int field, HebiQuaternionf* value) {
  return quaternionfGetter(ref, feedback_metadata, field, value);
}

HebiStatusCode quaternionfGetter(const HebiInfoRef& ref, int field, HebiQuaternionf* value) {
  return quaternionfGetter(ref, info_metadata, field, value);
}

template<typename RefT, typename MetadataT>
HebiStatusCode floatIoPinGetter(const RefT& ref, MetadataT& metadata, int index, size_t pin_number, float* value) {
  if (index > metadata.io_field_count_ || index < 0) {
    return HebiStatusInvalidArgument;
  }
  if (pin_number == 0 || pin_number > metadata.io_field_sizes_[index] ) {
    return HebiStatusInvalidArgument;
  }
  const auto relativeOffset = static_cast<size_t>(metadata.io_relative_offsets_[index] + pin_number - 1);
  const auto has_offset = static_cast<size_t>(metadata.io_field_bitfield_offset_ + relativeOffset);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  if (!has_bits.get(has_offset)) {
    return HebiStatusValueNotSet;
  }
  // Additionally, need to check the value of the field (since it holds the variant state)
  const auto& field = ref.io_fields_[relativeOffset];
  if (field.stored_type_ != HebiIoBankPinResidentTypeFloat) {
    return HebiStatusValueNotSet;
  }
  if (value != nullptr) {
    *value = field.float_value_;
  }
  return HebiStatusSuccess;
}

HebiStatusCode floatIoPinGetter(const HebiCommandRef& ref, int index, size_t pin_number, float* value) {
  return floatIoPinGetter(ref, command_metadata, index, pin_number, value);
}

HebiStatusCode floatIoPinGetter(const HebiFeedbackRef& ref, int index, size_t pin_number, float* value) {
  return floatIoPinGetter(ref, feedback_metadata, index, pin_number, value);
}

HebiStatusCode floatIoPinGetter(const HebiInfoRef& ref, int index, size_t pin_number, float* value) {
  return floatIoPinGetter(ref, info_metadata, index, pin_number, value);
}

template<typename RefT, typename MetadataT>
HebiStatusCode intIoPinGetter(const RefT& ref, MetadataT& metadata, int index, size_t pin_number, int64_t* value) {
  if (index > metadata.io_field_count_ || index < 0) {
    return HebiStatusInvalidArgument;
  }
  if (pin_number == 0 || pin_number > metadata.io_field_sizes_[index] ) {
    return HebiStatusInvalidArgument;
  }
  const auto relativeOffset = static_cast<size_t>(metadata.io_relative_offsets_[index] + pin_number - 1);
  const auto has_offset = static_cast<size_t>(metadata.io_field_bitfield_offset_ + relativeOffset);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  if (!has_bits.get(has_offset)) {
    return HebiStatusValueNotSet;
  }
  // Additionally, need to check the value of the field (since it holds the variant state)
  const auto& field = ref.io_fields_[relativeOffset];
  if (field.stored_type_ != HebiIoBankPinResidentTypeInteger) {
    return HebiStatusValueNotSet;
  }
  if (value != nullptr) {
    *value = field.int_value_;
  }
  return HebiStatusSuccess;
}

HebiStatusCode intIoPinGetter(const HebiCommandRef& ref, int index, size_t pin_number, int64_t* value) {
  return intIoPinGetter(ref, command_metadata, index, pin_number, value);
}

HebiStatusCode intIoPinGetter(const HebiFeedbackRef& ref, int index, size_t pin_number, int64_t* value) {
  return intIoPinGetter(ref, feedback_metadata, index, pin_number, value);
}

HebiStatusCode intIoPinGetter(const HebiInfoRef& ref, int index, size_t pin_number, int64_t* value) {
  return intIoPinGetter(ref, info_metadata, index, pin_number, value);
}

template<typename RefT, typename MetadataT>
bool flagGetter(const RefT& ref, MetadataT& metadata, int field) {
  auto index = static_cast<size_t>(field);
  if (index > metadata.flag_field_count_ || index < 0) {
    return 0;
  }
  auto has_offset = static_cast<size_t>(index + metadata.flag_field_bitfield_offset_);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  return has_bits.get(has_offset);
}

bool flagGetter(const HebiCommandRef& ref, int field) {
  return flagGetter(ref, command_metadata, field);
}

bool flagGetter(const HebiFeedbackRef& ref, int field) {
  return flagGetter(ref, feedback_metadata, field);
}

bool flagGetter(const HebiInfoRef& ref, int field) {
  return flagGetter(ref, info_metadata, field);
}

template<typename RefT, typename MetadataT>
HebiStatusCode boolGetter(const RefT& ref, MetadataT& metadata, int field, bool* value) {
  auto index = static_cast<int32_t>(field);
  if (index > metadata.bool_field_count_ || index < 0) {
    return HebiStatusInvalidArgument;
  }
  auto has_offset = static_cast<size_t>(index + metadata.bool_field_bitfield_offset_);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  if (!has_bits.get(has_offset)) {
    return HebiStatusValueNotSet;
  }
  if (value != nullptr) {
    *value = ref.bool_fields_[index];
  }
  return HebiStatusSuccess;
}

HebiStatusCode boolGetter(const HebiCommandRef& ref, int field, bool* value) {
  return boolGetter(ref, command_metadata, field, value);
}

HebiStatusCode boolGetter(const HebiFeedbackRef& ref, int field, bool* value) {
  return boolGetter(ref, feedback_metadata, field, value);
}

HebiStatusCode boolGetter(const HebiInfoRef& ref, int field, bool* value) {
  return boolGetter(ref, info_metadata, field, value);
}

template<typename RefT, typename MetadataT>
HebiStatusCode enumGetter(const RefT& ref, MetadataT& metadata, int field, int32_t* value) {
  auto index = static_cast<int32_t>(field);
  if (index > metadata.enum_field_count_ || index < 0) {
    return HebiStatusInvalidArgument;
  }
  auto has_offset = static_cast<size_t>(index + metadata.enum_field_bitfield_offset_);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  if (!has_bits.get(has_offset)) {
    return HebiStatusValueNotSet;
  }
  if (value != nullptr) {
    *value = ref.enum_fields_[index];
  }
  return HebiStatusSuccess;
}

HebiStatusCode enumGetter(const HebiCommandRef& ref, int field, int32_t* value) {
  return enumGetter(ref, command_metadata, field, value);
}

HebiStatusCode enumGetter(const HebiFeedbackRef& ref, int field, int32_t* value) {
  return enumGetter(ref, feedback_metadata, field, value);
}

HebiStatusCode enumGetter(const HebiInfoRef& ref, int field, int32_t* value) {
  return enumGetter(ref, info_metadata, field, value);
}

template<typename RefT, typename MetadataT>
HebiStatusCode uint64Getter(const RefT& ref, MetadataT& metadata, int field, uint64_t* value) {
  auto index = static_cast<int32_t>(field);
  if (index > metadata.uint64_field_count_ || index < 0) {
    return HebiStatusInvalidArgument;
  }
  auto has_offset = static_cast<size_t>(index + metadata.uint64_field_bitfield_offset_);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  if (!has_bits.get(has_offset)) {
    return HebiStatusValueNotSet;
  }
  if (value != nullptr) {
    *value = ref.uint64_fields_[index];
  }
  return HebiStatusSuccess;
}

HebiStatusCode uint64Getter(const HebiCommandRef& ref, int field, uint64_t* value) {
  return uint64Getter(ref, command_metadata, field, value);
}

HebiStatusCode uint64Getter(const HebiFeedbackRef& ref, int field, uint64_t* value) {
  return uint64Getter(ref, feedback_metadata, field, value);
}

HebiStatusCode uint64Getter(const HebiInfoRef& ref, int field, uint64_t* value) {
  return uint64Getter(ref, info_metadata, field, value);
}

template<typename RefT, typename MetadataT>
HebiStatusCode ledGetter(const RefT& ref, MetadataT& metadata, int field, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* a) {
  auto index = static_cast<int32_t>(field);
  if (index > metadata.led_field_count_ || index < 0) {
    return HebiStatusInvalidArgument;
  }
  auto has_offset = static_cast<size_t>(index + metadata.led_field_bitfield_offset_);
  hebi::ProxyBitSet has_bits(ref.message_bitfield_, metadata.message_bitfield_count_);
  if (!has_bits.get(has_offset)) {
    return HebiStatusValueNotSet;
  }
  if (r != nullptr && g != nullptr && b != nullptr && a != nullptr) {
    uint32_t color = ref.led_fields_[index];
    *r = (color & (0xFF << 24)) >> 24;
    *g = (color & (0xFF << 16)) >> 16;
    *b = (color & (0xFF << 8)) >> 8;
    *a = (color & 0xFF);
  }
  return HebiStatusSuccess;
}

HebiStatusCode ledGetter(const HebiCommandRef ref, int field, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* a) {
  return ledGetter(ref, command_metadata, field, r, g, b, a);
}

HebiStatusCode ledGetter(const HebiFeedbackRef ref, int field, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* a) {
  return ledGetter(ref, feedback_metadata, field, r, g, b, a);
}

HebiStatusCode ledGetter(const HebiInfoRef ref, int field, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* a) {
  return ledGetter(ref, info_metadata, field, r, g, b, a);
}

void hebiCommandSetEnum(HebiCommandRef& command, HebiCommandEnumField field, const int32_t* value) {
  auto index = static_cast<size_t>(field);
  if (index > command_metadata.enum_field_count_ || index < 0) {
    return;
  }
  auto hasOffset = static_cast<size_t>(index + command_metadata.enum_field_bitfield_offset_);
  hebi::MutableProxyBitSet hasBits(command.message_bitfield_, command_metadata.message_bitfield_count_);
  if (value == nullptr) {
    hasBits.reset(hasOffset);
  } else {
    hasBits.set(hasOffset);
    command.enum_fields_[index] = *value;
  }
}

void hebiCommandSetFloat(HebiCommandRef& command, HebiCommandFloatField field, const float* value) {
  auto index = static_cast<size_t>(field);
  if (index > command_metadata.float_field_count_ || index < 0) {
    return;
  }
  auto has_offset = static_cast<size_t>(index + command_metadata.float_field_bitfield_offset_);
  hebi::MutableProxyBitSet has_bits(command.message_bitfield_, command_metadata.message_bitfield_count_);
  if (value == nullptr) {
    has_bits.reset(has_offset);
  } else {
    has_bits.set(has_offset);
    command.float_fields_[index] = *value;
  }
}

void hebiCommandSetHighResAngle(HebiCommandRef& command, HebiCommandHighResAngleField field, const int64_t* int_part,
                                const float* dec_part) {
  auto index = static_cast<size_t>(field);
  if (index > command_metadata.high_res_angle_field_count_ || index < 0) {
    return;
  }
  auto has_offset = static_cast<size_t>(index + command_metadata.high_res_angle_field_bitfield_offset_);
  hebi::MutableProxyBitSet has_bits(command.message_bitfield_, command_metadata.message_bitfield_count_);
  if (int_part != nullptr && dec_part != nullptr) {
    has_bits.set(has_offset);
    auto& val = command.high_res_angle_fields_[index];
    val.revolutions_ = *int_part;
    val.offset_ = *dec_part;
  } else {
    has_bits.reset(has_offset);
  }
}

void hebiCommandSetNumberedFloat(HebiCommandRef& command, HebiCommandNumberedFloatField field, size_t number,
                                 const float* value) {
  auto index = static_cast<size_t>(field);
  if (index > command_metadata.numbered_float_field_count_ || index < 0) {
    return;
  }
  if (number > command_metadata.numbered_float_field_sizes_[index] || number < 1) {
    return;
  }
  // Special cased: also account for `numbered_float_relative_offsets_`, along with the actual number within the field
  const auto relativeOffset = static_cast<size_t>(command_metadata.numbered_float_relative_offsets_[index] + number - 1);
  const auto has_offset = static_cast<size_t>(command_metadata.numbered_float_field_bitfield_offset_ + relativeOffset);
  hebi::MutableProxyBitSet has_bits(command.message_bitfield_, command_metadata.message_bitfield_count_);
  if (value != nullptr) {
    has_bits.set(has_offset);
    command.numbered_float_fields_[relativeOffset] = *value;
  } else {
    has_bits.reset(has_offset);
  }
}

void hebiCommandSetBool(HebiCommandRef& command, HebiCommandBoolField field, const int32_t* value) {
  auto index = static_cast<size_t>(field);
  if (index > command_metadata.bool_field_count_ || index < 0) {
    return;
  }
  auto has_offset = static_cast<size_t>(index + command_metadata.bool_field_bitfield_offset_);
  hebi::MutableProxyBitSet has_bits(command.message_bitfield_, command_metadata.message_bitfield_count_);
  if (value == nullptr) {
    has_bits.reset(has_offset);
  } else {
    has_bits.set(has_offset);
    command.bool_fields_[index] = static_cast<bool>(*value);
  }
}

void hebiCommandSetFlag(HebiCommandRef& command, HebiCommandFlagField field, int32_t value) {
  auto index = static_cast<size_t>(field);
  if (index > command_metadata.flag_field_count_ || index < 0) {
    return;
  }
  auto has_offset = static_cast<size_t>(index + command_metadata.flag_field_bitfield_offset_);
  hebi::MutableProxyBitSet has_bits(command.message_bitfield_, command_metadata.message_bitfield_count_);
  if (value != 0) {
    has_bits.set(has_offset);
  } else {
    has_bits.reset(has_offset);
  }
}

void hebiCommandSetIoPinInt(HebiCommandRef& command, HebiCommandIoPinBank bank, size_t pin_number, const int64_t* value) {
  auto index = static_cast<size_t>(bank);
  if (index > command_metadata.io_field_count_ || index < 0) {
    return;
  }
  if (pin_number > 8 || pin_number < 1) {
    return;
  }
  const auto relativeOffset = static_cast<size_t>(command_metadata.io_relative_offsets_[index] + pin_number - 1);
  const auto has_offset = static_cast<size_t>(command_metadata.io_field_bitfield_offset_ + relativeOffset);
  hebi::MutableProxyBitSet has_bits(command.message_bitfield_, command_metadata.message_bitfield_count_);
  auto& field = command.io_fields_[relativeOffset];
  if (value == nullptr) {
    has_bits.reset(has_offset);
    field.stored_type_ = HebiIoBankPinResidentTypeNone;
  } else {
    has_bits.set(has_offset);
    field.stored_type_ = HebiIoBankPinResidentTypeInteger;
    field.int_value_ = *value;
  }
}

void hebiCommandSetIoPinFloat(HebiCommandRef& command, HebiCommandIoPinBank bank, size_t pin_number, const float* value) {
  auto index = static_cast<size_t>(bank);
  if (index > command_metadata.io_field_count_ || index < 0) {
    return;
  }
  if (pin_number > 8 || pin_number < 1) {
    return;
  }
  const auto relativeOffset = static_cast<size_t>(command_metadata.io_relative_offsets_[index] + pin_number - 1);
  const auto has_offset = static_cast<size_t>(command_metadata.io_field_bitfield_offset_ + relativeOffset);
  hebi::MutableProxyBitSet has_bits(command.message_bitfield_, command_metadata.message_bitfield_count_);
  auto& field = command.io_fields_[relativeOffset];
  if (value == nullptr) {
    has_bits.reset(has_offset);
    field.stored_type_ = HebiIoBankPinResidentTypeNone;
  } else {
    has_bits.set(has_offset);
    field.stored_type_ = HebiIoBankPinResidentTypeFloat;
    field.float_value_ = *value;
  }
}

void hebiCommandSetLed(HebiCommandRef& command, HebiCommandLedField field, const Color* color) {
  auto index = static_cast<size_t>(field);
  if (index > command_metadata.led_field_count_ || index < 0) {
    return;
  }
  auto has_offset = static_cast<size_t>(index + command_metadata.led_field_bitfield_offset_);
  hebi::MutableProxyBitSet has_bits(command.message_bitfield_, command_metadata.message_bitfield_count_);

  if (color == nullptr) {
    // Clear LED
    has_bits.reset(has_offset);
    command.led_fields_[index] = 0;
  } else {
    // Note -- 255 alpha is "override", 0 alpha is "module control" 
    has_bits.set(has_offset);
    auto& val = command.led_fields_[index];
    val = (static_cast<int>(color->getRed()) << 24) |
          (static_cast<int>(color->getGreen()) << 16) |
          (static_cast<int>(color->getBlue()) << 8) |
          (static_cast<int>(color->getAlpha()));
  }
}


} // namespace hebi
