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

#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_VECTOR_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_VECTOR_H_

#include <cstddef>
#include <cstdlib>

#include "collision_checking/inlining.h"

// A simple vector implementation suitable for CPU and GPU code.
namespace collision_checking {

// Traits for a CPU-only vector. Use this for CPU-only code.
struct DefaultAllocatorTraits {
  static void* Malloc(std::size_t size) { return std::malloc(size); }
  static void Free(void* pointer) { std::free(pointer); }
};

template <typename AllocatorTraits>
class ParametrizedNewDelete {
 public:
  void* operator new(size_t size) { return AllocatorTraits::Malloc(size); }
  void operator delete(void* pointer) { AllocatorTraits::Free(pointer); }
};

// Put allocation and new in base policy class!
// A vector templated on its element type and allocator.
template <typename ElementType, typename AllocatorTraits>
class Vector : public ParametrizedNewDelete<AllocatorTraits> {
 public:
  using value_type = ElementType;
  using const_iterator = const ElementType*;
  using iterator = const ElementType*;

  Vector() = default;
  explicit Vector(std::size_t size) { resize(size); }
  Vector(const Vector& other) { *this = other; }
  ~Vector() { DeallocateAndDestroy(); }

  CC_INLINE Vector& operator=(const Vector& other) {
    resize(other.size());
    for (int i = 0; i < size_; i++) {
      data_[i] = other.data_[i];
    }
    size_ = other.size_;
    return *this;
  }

  CC_INLINE void resize(std::size_t size) {
    if (size == size_) {
      return;
    }

    ElementType* current_data = data_;
    const std::size_t current_size = size_;
    const std::size_t maintain_data_until = size > size_ ? size_ : size;

    AllocateAndCreate(size);
    for (int i = 0; i < maintain_data_until; ++i) {
      data_[i] = current_data[i];
    }
    DeallocateAndDestroy(current_data, current_size);
  }

  CC_INLINE ElementType* data() { return data_; }
  CC_INLINE const ElementType* data() const { return data_; }
  CC_INLINE ElementType* begin() { return data_; }
  CC_INLINE const ElementType* begin() const { return data_; }
  CC_INLINE ElementType* end() { return data_ + size_; }
  CC_INLINE const ElementType* end() const { return data_ + size_; }
  CC_INLINE size_t size() const { return size_; }
  CC_INLINE bool empty() const { return size_ == 0; }
  CC_INLINE const ElementType& operator[](int i) const {
    return data_[i];
  }
  CC_INLINE ElementType& operator[](int i) { return data_[i]; }

 private:
  CC_INLINE void DeallocateAndDestroy() {
    DeallocateAndDestroy(data_, size_);
  }
  CC_INLINE void DeallocateAndDestroy(ElementType* data,
                                             std::size_t size) {
    if (data == nullptr) {
      return;
    }
    for (int index = size - 1; index >= 0; index--) {
      data[index].~ElementType();
    }
    AllocatorTraits::Free(data);
    data = nullptr;
    size = 0;
  }
  CC_INLINE void AllocateAndCreate(std::size_t size) {
    size_ = size;
    data_ = static_cast<ElementType*>(
        AllocatorTraits::Malloc(sizeof(ElementType) * size_));
    for (ElementType* ptr = begin(); ptr != end(); ++ptr) {
      ::new (static_cast<void*>(ptr)) ElementType();
    }
  }
  ElementType* data_ = nullptr;
  std::size_t size_ = 0;
};
}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_VECTOR_H_
