// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vssref_placement.proto

#include "vssref_placement.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
extern PROTOBUF_INTERNAL_EXPORT_vssref_5fcommon_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_Frame_vssref_5fcommon_2eproto;
namespace VSSRef {
namespace team_to_ref {
class VSSRef_PlacementDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<VSSRef_Placement> _instance;
} _VSSRef_Placement_default_instance_;
}  // namespace team_to_ref
}  // namespace VSSRef
static void InitDefaultsscc_info_VSSRef_Placement_vssref_5fplacement_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::VSSRef::team_to_ref::_VSSRef_Placement_default_instance_;
    new (ptr) ::VSSRef::team_to_ref::VSSRef_Placement();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::VSSRef::team_to_ref::VSSRef_Placement::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_VSSRef_Placement_vssref_5fplacement_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_VSSRef_Placement_vssref_5fplacement_2eproto}, {
      &scc_info_Frame_vssref_5fcommon_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_vssref_5fplacement_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_vssref_5fplacement_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_vssref_5fplacement_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_vssref_5fplacement_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::VSSRef::team_to_ref::VSSRef_Placement, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::VSSRef::team_to_ref::VSSRef_Placement, world_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::VSSRef::team_to_ref::VSSRef_Placement)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::VSSRef::team_to_ref::_VSSRef_Placement_default_instance_),
};

const char descriptor_table_protodef_vssref_5fplacement_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\026vssref_placement.proto\022\022VSSRef.team_to"
  "_ref\032\023vssref_common.proto\"0\n\020VSSRef_Plac"
  "ement\022\034\n\005world\030\001 \001(\0132\r.VSSRef.Frameb\006pro"
  "to3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_vssref_5fplacement_2eproto_deps[1] = {
  &::descriptor_table_vssref_5fcommon_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_vssref_5fplacement_2eproto_sccs[1] = {
  &scc_info_VSSRef_Placement_vssref_5fplacement_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_vssref_5fplacement_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_vssref_5fplacement_2eproto = {
  false, false, descriptor_table_protodef_vssref_5fplacement_2eproto, "vssref_placement.proto", 123,
  &descriptor_table_vssref_5fplacement_2eproto_once, descriptor_table_vssref_5fplacement_2eproto_sccs, descriptor_table_vssref_5fplacement_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_vssref_5fplacement_2eproto::offsets,
  file_level_metadata_vssref_5fplacement_2eproto, 1, file_level_enum_descriptors_vssref_5fplacement_2eproto, file_level_service_descriptors_vssref_5fplacement_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_vssref_5fplacement_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_vssref_5fplacement_2eproto)), true);
namespace VSSRef {
namespace team_to_ref {

// ===================================================================

void VSSRef_Placement::InitAsDefaultInstance() {
  ::VSSRef::team_to_ref::_VSSRef_Placement_default_instance_._instance.get_mutable()->world_ = const_cast< ::VSSRef::Frame*>(
      ::VSSRef::Frame::internal_default_instance());
}
class VSSRef_Placement::_Internal {
 public:
  static const ::VSSRef::Frame& world(const VSSRef_Placement* msg);
};

const ::VSSRef::Frame&
VSSRef_Placement::_Internal::world(const VSSRef_Placement* msg) {
  return *msg->world_;
}
void VSSRef_Placement::clear_world() {
  if (GetArena() == nullptr && world_ != nullptr) {
    delete world_;
  }
  world_ = nullptr;
}
VSSRef_Placement::VSSRef_Placement(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:VSSRef.team_to_ref.VSSRef_Placement)
}
VSSRef_Placement::VSSRef_Placement(const VSSRef_Placement& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_world()) {
    world_ = new ::VSSRef::Frame(*from.world_);
  } else {
    world_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:VSSRef.team_to_ref.VSSRef_Placement)
}

void VSSRef_Placement::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_VSSRef_Placement_vssref_5fplacement_2eproto.base);
  world_ = nullptr;
}

VSSRef_Placement::~VSSRef_Placement() {
  // @@protoc_insertion_point(destructor:VSSRef.team_to_ref.VSSRef_Placement)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void VSSRef_Placement::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  if (this != internal_default_instance()) delete world_;
}

void VSSRef_Placement::ArenaDtor(void* object) {
  VSSRef_Placement* _this = reinterpret_cast< VSSRef_Placement* >(object);
  (void)_this;
}
void VSSRef_Placement::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void VSSRef_Placement::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const VSSRef_Placement& VSSRef_Placement::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_VSSRef_Placement_vssref_5fplacement_2eproto.base);
  return *internal_default_instance();
}


void VSSRef_Placement::Clear() {
// @@protoc_insertion_point(message_clear_start:VSSRef.team_to_ref.VSSRef_Placement)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArena() == nullptr && world_ != nullptr) {
    delete world_;
  }
  world_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* VSSRef_Placement::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // .VSSRef.Frame world = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_world(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* VSSRef_Placement::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:VSSRef.team_to_ref.VSSRef_Placement)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .VSSRef.Frame world = 1;
  if (this->has_world()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::world(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:VSSRef.team_to_ref.VSSRef_Placement)
  return target;
}

size_t VSSRef_Placement::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:VSSRef.team_to_ref.VSSRef_Placement)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .VSSRef.Frame world = 1;
  if (this->has_world()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *world_);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void VSSRef_Placement::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:VSSRef.team_to_ref.VSSRef_Placement)
  GOOGLE_DCHECK_NE(&from, this);
  const VSSRef_Placement* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<VSSRef_Placement>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:VSSRef.team_to_ref.VSSRef_Placement)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:VSSRef.team_to_ref.VSSRef_Placement)
    MergeFrom(*source);
  }
}

void VSSRef_Placement::MergeFrom(const VSSRef_Placement& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:VSSRef.team_to_ref.VSSRef_Placement)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_world()) {
    _internal_mutable_world()->::VSSRef::Frame::MergeFrom(from._internal_world());
  }
}

void VSSRef_Placement::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:VSSRef.team_to_ref.VSSRef_Placement)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void VSSRef_Placement::CopyFrom(const VSSRef_Placement& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:VSSRef.team_to_ref.VSSRef_Placement)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool VSSRef_Placement::IsInitialized() const {
  return true;
}

void VSSRef_Placement::InternalSwap(VSSRef_Placement* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(world_, other->world_);
}

::PROTOBUF_NAMESPACE_ID::Metadata VSSRef_Placement::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace team_to_ref
}  // namespace VSSRef
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::VSSRef::team_to_ref::VSSRef_Placement* Arena::CreateMaybeMessage< ::VSSRef::team_to_ref::VSSRef_Placement >(Arena* arena) {
  return Arena::CreateMessageInternal< ::VSSRef::team_to_ref::VSSRef_Placement >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>