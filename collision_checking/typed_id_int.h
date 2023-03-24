#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_TYPED_ID_INT_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_TYPED_ID_INT_H_

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

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_TYPED_ID_INT_H_
