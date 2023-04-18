// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COLLISION_CHECKING_TYPED_ID_INT_H_
#define COLLISION_CHECKING_TYPED_ID_INT_H_

#include <limits>
#include <type_traits>
#include <utility>

namespace collision_checking {

// A template for strongly typed ids based on unsigned integral types.
// Only implements currently required operators, add more as needed.
template <typename IntType>
class TypedIdInt {
 public:
  using ValueType = IntType;
  static constexpr TypedIdInt Max() {
    return TypedIdInt(std::numeric_limits<ValueType>::max());
  }
  static constexpr TypedIdInt Min() {
    return TypedIdInt(std::numeric_limits<ValueType>::min());
  }

  constexpr TypedIdInt() = default;
  constexpr explicit TypedIdInt(ValueType value) : value_{value} {
    static_assert(std::is_integral_v<ValueType>);
  }
  constexpr ValueType value() const { return value_; }

  template <typename T>
  constexpr explicit operator T() const {
    return value_;
  }
  constexpr TypedIdInt operator+(const TypedIdInt& other) const {
    return TypedIdInt(value_ + other.value_);
  }
  constexpr TypedIdInt operator-(const TypedIdInt& other) const {
    return TypedIdInt(value_ - other.value_);
  }
  constexpr bool operator==(const TypedIdInt& other) const {
    return value_ == other.value_;
  }
  constexpr bool operator!=(const TypedIdInt& other) const {
    return !(value_ == other.value_);
  }
  constexpr bool operator<(const TypedIdInt& other) const {
    return value_ < other.value_;
  }
  constexpr bool operator<=(const TypedIdInt& other) const {
    return value_ <= other.value_;
  }
  constexpr bool operator>(const TypedIdInt& other) const {
    return value_ > other.value_;
  }
  constexpr bool operator>=(const TypedIdInt& other) const {
    return value_ >= other.value_;
  }
  constexpr bool operator==(const ValueType& other) const {
    return value_ == other;
  }
  constexpr bool operator!=(const ValueType& other) const {
    return !(value_ == other);
  }

  TypedIdInt& operator++() {
    ++value_;
    return *this;
  }
  constexpr TypedIdInt operator++(int) {
    TypedIdInt temp(*this);
    ++value_;
    return temp;
  }
  constexpr TypedIdInt& operator--() {
    --value_;
    return *this;
  }
  constexpr TypedIdInt operator--(int postfix_flag) {
    TypedIdInt temp(*this);
    --value_;
    return temp;
  }

  template <typename H>
  friend H AbslHashValue(H h, const TypedIdInt& i) {
    return H::combine(std::move(h), i.value_);
  }

 private:
  ValueType value_ = ValueType{0};
};
}  // namespace collision_checking

#endif  // COLLISION_CHECKING_TYPED_ID_INT_H_
