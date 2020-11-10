// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vssref_placement.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_vssref_5fplacement_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_vssref_5fplacement_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3012000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3012003 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "vssref_common.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_vssref_5fplacement_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_vssref_5fplacement_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_vssref_5fplacement_2eproto;
namespace VSSRef {
namespace team_to_ref {
class VSSRef_Placement;
class VSSRef_PlacementDefaultTypeInternal;
extern VSSRef_PlacementDefaultTypeInternal _VSSRef_Placement_default_instance_;
}  // namespace team_to_ref
}  // namespace VSSRef
PROTOBUF_NAMESPACE_OPEN
template<> ::VSSRef::team_to_ref::VSSRef_Placement* Arena::CreateMaybeMessage<::VSSRef::team_to_ref::VSSRef_Placement>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace VSSRef {
namespace team_to_ref {

// ===================================================================

class VSSRef_Placement PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:VSSRef.team_to_ref.VSSRef_Placement) */ {
 public:
  inline VSSRef_Placement() : VSSRef_Placement(nullptr) {};
  virtual ~VSSRef_Placement();

  VSSRef_Placement(const VSSRef_Placement& from);
  VSSRef_Placement(VSSRef_Placement&& from) noexcept
    : VSSRef_Placement() {
    *this = ::std::move(from);
  }

  inline VSSRef_Placement& operator=(const VSSRef_Placement& from) {
    CopyFrom(from);
    return *this;
  }
  inline VSSRef_Placement& operator=(VSSRef_Placement&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const VSSRef_Placement& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const VSSRef_Placement* internal_default_instance() {
    return reinterpret_cast<const VSSRef_Placement*>(
               &_VSSRef_Placement_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(VSSRef_Placement& a, VSSRef_Placement& b) {
    a.Swap(&b);
  }
  inline void Swap(VSSRef_Placement* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(VSSRef_Placement* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline VSSRef_Placement* New() const final {
    return CreateMaybeMessage<VSSRef_Placement>(nullptr);
  }

  VSSRef_Placement* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<VSSRef_Placement>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const VSSRef_Placement& from);
  void MergeFrom(const VSSRef_Placement& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(VSSRef_Placement* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "VSSRef.team_to_ref.VSSRef_Placement";
  }
  protected:
  explicit VSSRef_Placement(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_vssref_5fplacement_2eproto);
    return ::descriptor_table_vssref_5fplacement_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kWorldFieldNumber = 1,
  };
  // .VSSRef.Frame world = 1;
  bool has_world() const;
  private:
  bool _internal_has_world() const;
  public:
  void clear_world();
  const ::VSSRef::Frame& world() const;
  ::VSSRef::Frame* release_world();
  ::VSSRef::Frame* mutable_world();
  void set_allocated_world(::VSSRef::Frame* world);
  private:
  const ::VSSRef::Frame& _internal_world() const;
  ::VSSRef::Frame* _internal_mutable_world();
  public:
  void unsafe_arena_set_allocated_world(
      ::VSSRef::Frame* world);
  ::VSSRef::Frame* unsafe_arena_release_world();

  // @@protoc_insertion_point(class_scope:VSSRef.team_to_ref.VSSRef_Placement)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::VSSRef::Frame* world_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_vssref_5fplacement_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// VSSRef_Placement

// .VSSRef.Frame world = 1;
inline bool VSSRef_Placement::_internal_has_world() const {
  return this != internal_default_instance() && world_ != nullptr;
}
inline bool VSSRef_Placement::has_world() const {
  return _internal_has_world();
}
inline const ::VSSRef::Frame& VSSRef_Placement::_internal_world() const {
  const ::VSSRef::Frame* p = world_;
  return p != nullptr ? *p : *reinterpret_cast<const ::VSSRef::Frame*>(
      &::VSSRef::_Frame_default_instance_);
}
inline const ::VSSRef::Frame& VSSRef_Placement::world() const {
  // @@protoc_insertion_point(field_get:VSSRef.team_to_ref.VSSRef_Placement.world)
  return _internal_world();
}
inline void VSSRef_Placement::unsafe_arena_set_allocated_world(
    ::VSSRef::Frame* world) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(world_);
  }
  world_ = world;
  if (world) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:VSSRef.team_to_ref.VSSRef_Placement.world)
}
inline ::VSSRef::Frame* VSSRef_Placement::release_world() {
  
  ::VSSRef::Frame* temp = world_;
  world_ = nullptr;
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::VSSRef::Frame* VSSRef_Placement::unsafe_arena_release_world() {
  // @@protoc_insertion_point(field_release:VSSRef.team_to_ref.VSSRef_Placement.world)
  
  ::VSSRef::Frame* temp = world_;
  world_ = nullptr;
  return temp;
}
inline ::VSSRef::Frame* VSSRef_Placement::_internal_mutable_world() {
  
  if (world_ == nullptr) {
    auto* p = CreateMaybeMessage<::VSSRef::Frame>(GetArena());
    world_ = p;
  }
  return world_;
}
inline ::VSSRef::Frame* VSSRef_Placement::mutable_world() {
  // @@protoc_insertion_point(field_mutable:VSSRef.team_to_ref.VSSRef_Placement.world)
  return _internal_mutable_world();
}
inline void VSSRef_Placement::set_allocated_world(::VSSRef::Frame* world) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(world_);
  }
  if (world) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(world)->GetArena();
    if (message_arena != submessage_arena) {
      world = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, world, submessage_arena);
    }
    
  } else {
    
  }
  world_ = world;
  // @@protoc_insertion_point(field_set_allocated:VSSRef.team_to_ref.VSSRef_Placement.world)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace team_to_ref
}  // namespace VSSRef

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_vssref_5fplacement_2eproto