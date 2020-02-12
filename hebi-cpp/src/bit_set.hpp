#pragma once

#include <array>

namespace hebi {

static constexpr size_t BitsInDataWord = sizeof(int) * 8;

static inline bool extractBit(unsigned int index, int word) {
  return static_cast<bool>((word >> index) & 1);
}

static inline int setBit(unsigned int index, int word) {
  const auto val = 1 << index;
  return word | val;
}

static inline int clearBit(unsigned int index, int word) {
  const auto val = 1 << index;
  return word & ~val;
}

class MutableProxyBitSet {
public:
  MutableProxyBitSet(int* data, size_t bits)
    : data_(data), bit_count_(bits), data_word_count_((bits / BitsInDataWord) + 1 * ( (bits % BitsInDataWord) != 0 )) {}

  bool get(size_t index) const {
    const auto wordIdx = index / BitsInDataWord;
    const auto word = data_[wordIdx];
    const auto relIdx = index % BitsInDataWord;
    return extractBit(relIdx, word);
  }

  void set(size_t index) {
    const auto wordIdx = index / BitsInDataWord;
    const auto word = data_[wordIdx];
    const auto relIdx = index % BitsInDataWord;
    data_[wordIdx] = setBit(relIdx, word);
  }

  void reset(size_t index) {
    const auto wordIdx = index / BitsInDataWord;
    const auto word = data_[wordIdx];
    const auto relIdx = index % BitsInDataWord;
    data_[wordIdx] = clearBit(relIdx, word);
  }

  void reset() {
    for (size_t i = 0; i < data_word_count_; i++) {
      data_[i] = 0;
    }
  }

  int* data() {
    return data_;
  }

  const int* data() const {
    return data_;
  }

private:
  int* data_;
  const size_t bit_count_;
  const size_t data_word_count_;
};

class ProxyBitSet {
public:
  ProxyBitSet(const int* data, size_t bits)
    : data_(data), bit_count_(bits), data_word_count_((bits / BitsInDataWord) + 1 * ( (bits % BitsInDataWord) != 0 )) {}

  bool get(size_t index) const {
    const auto wordIdx = index / BitsInDataWord;
    const auto word = data_[wordIdx];
    const auto relIdx = index % BitsInDataWord;
    return extractBit(relIdx, word);
  }

  const int* data() const {
    return data_;
  }

private:
  const int* data_;
  const size_t bit_count_;
  const size_t data_word_count_;
};

}
