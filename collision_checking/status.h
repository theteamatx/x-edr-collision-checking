#ifndef EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_STATUS_H_
#define EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_STATUS_H_
#include "experimental/users/buschmann/collision_checking/inlining.h"
#include "third_party/absl/status/status.h"

namespace collision_checking {
// A simple status that will work for regular CPU C++ and CUDA kernel code.
class Status {
 public:
  enum ErrorMessageEnum {
    kNone = 0,
    kResultIsNullptr = 1,
    kResultSizeWrong = 2,
    kResultOptionsWrong = 3,
    kInputJointVectorSizeWrong = 4,
    kScratchSizeWrong = 5,
    kUnknownObject = 6,
    kInvalidObjectIdSet = 7,
    kInvalidObjectName = 8
  };

  CC_INLINE Status() = default;

  CC_INLINE Status(absl::StatusCode code, ErrorMessageEnum message_enum)
      : code_(code), message_enum_(message_enum) {}

  CC_INLINE absl::StatusCode code() const { return code_; }
  CC_INLINE ErrorMessageEnum message_enum() const {
    return message_enum_;
  }

  CC_INLINE bool ok() const { return code_ == absl::StatusCode::kOk; }

  // Returns an absl::Status corresponding to this one.
  // Useful for interfacing with non-realtime host code.
  absl::Status ToAbslStatus() const {
    switch (message_enum_) {
      case kNone:
        return absl::Status(code_, "");
      case kResultIsNullptr:
        return absl::Status(code_, "result == nullptr.");
      case kResultSizeWrong:
        return absl::Status(
            code_, "Result object size doesn't match moving objects size.");
      case kResultOptionsWrong:
        return absl::Status(code_,
                            "Result object doesn't match query options.");
      case kInputJointVectorSizeWrong:
        return absl::Status(code_, "Input joint vector size wrong.");
      case kScratchSizeWrong:
        return absl::Status(code_, "Scratch size wrong.");
      case kUnknownObject:
        return absl::Status(code_, "Unknown ObjectId.");
      case kInvalidObjectIdSet:
        return absl::Status(code_, "Invalid ObjectIdSet.");
      case kInvalidObjectName:
        return absl::Status(code_, "Invalid object name.");
      default:
        break;
    }
    return absl::Status(code_,
                        absl::StrCat("Invalid message_enum: ", message_enum_));
  }

 private:
  // The status code.
  absl::StatusCode code_ = absl::StatusCode::kOk;
  // The enum value for a corresponding status message.
  ErrorMessageEnum message_enum_ = ErrorMessageEnum::kNone;
};

CC_INLINE Status OkStatus() { return Status(); }

}  // namespace collision_checking

#endif  // EXPERIMENTAL_USERS_BUSCHMANN_COLLISION_CHECKING_STATUS_H_
