// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vssref_common.proto

#include "vssref_common.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_vssref_5fcommon_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Robot_vssref_5fcommon_2eproto;
namespace VSSRef {
class RobotDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Robot> _instance;
} _Robot_default_instance_;
class FrameDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Frame> _instance;
} _Frame_default_instance_;
}  // namespace VSSRef
static void InitDefaultsscc_info_Frame_vssref_5fcommon_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::VSSRef::_Frame_default_instance_;
    new (ptr) ::VSSRef::Frame();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::VSSRef::Frame::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_Frame_vssref_5fcommon_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_Frame_vssref_5fcommon_2eproto}, {
      &scc_info_Robot_vssref_5fcommon_2eproto.base,}};

static void InitDefaultsscc_info_Robot_vssref_5fcommon_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::VSSRef::_Robot_default_instance_;
    new (ptr) ::VSSRef::Robot();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::VSSRef::Robot::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Robot_vssref_5fcommon_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_Robot_vssref_5fcommon_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_vssref_5fcommon_2eproto[2];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_vssref_5fcommon_2eproto[4];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_vssref_5fcommon_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_vssref_5fcommon_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::VSSRef::Robot, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::VSSRef::Robot, robot_id_),
  PROTOBUF_FIELD_OFFSET(::VSSRef::Robot, x_),
  PROTOBUF_FIELD_OFFSET(::VSSRef::Robot, y_),
  PROTOBUF_FIELD_OFFSET(::VSSRef::Robot, orientation_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::VSSRef::Frame, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::VSSRef::Frame, teamcolor_),
  PROTOBUF_FIELD_OFFSET(::VSSRef::Frame, robots_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::VSSRef::Robot)},
  { 9, -1, sizeof(::VSSRef::Frame)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::VSSRef::_Robot_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::VSSRef::_Frame_default_instance_),
};

const char descriptor_table_protodef_vssref_5fcommon_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\023vssref_common.proto\022\006VSSRef\"D\n\005Robot\022\020"
  "\n\010robot_id\030\001 \001(\r\022\t\n\001x\030\002 \001(\001\022\t\n\001y\030\003 \001(\001\022\023"
  "\n\013orientation\030\004 \001(\001\"H\n\005Frame\022 \n\tteamColo"
  "r\030\001 \001(\0162\r.VSSRef.Color\022\035\n\006robots\030\002 \003(\0132\r"
  ".VSSRef.Robot*i\n\004Foul\022\r\n\tFREE_KICK\020\000\022\020\n\014"
  "PENALTY_KICK\020\001\022\r\n\tGOAL_KICK\020\002\022\r\n\tFREE_BA"
  "LL\020\003\022\013\n\007KICKOFF\020\004\022\010\n\004STOP\020\005\022\013\n\007GAME_ON\020\006"
  "*\'\n\005Color\022\010\n\004BLUE\020\000\022\n\n\006YELLOW\020\001\022\010\n\004NONE\020"
  "\002*[\n\010Quadrant\022\017\n\013NO_QUADRANT\020\000\022\016\n\nQUADRA"
  "NT_1\020\001\022\016\n\nQUADRANT_2\020\002\022\016\n\nQUADRANT_3\020\003\022\016"
  "\n\nQUADRANT_4\020\004*4\n\004Half\022\013\n\007NO_HALF\020\000\022\016\n\nF"
  "IRST_HALF\020\001\022\017\n\013SECOND_HALF\020\002b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_vssref_5fcommon_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_vssref_5fcommon_2eproto_sccs[2] = {
  &scc_info_Frame_vssref_5fcommon_2eproto.base,
  &scc_info_Robot_vssref_5fcommon_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_vssref_5fcommon_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_vssref_5fcommon_2eproto = {
  false, false, descriptor_table_protodef_vssref_5fcommon_2eproto, "vssref_common.proto", 476,
  &descriptor_table_vssref_5fcommon_2eproto_once, descriptor_table_vssref_5fcommon_2eproto_sccs, descriptor_table_vssref_5fcommon_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_vssref_5fcommon_2eproto::offsets,
  file_level_metadata_vssref_5fcommon_2eproto, 2, file_level_enum_descriptors_vssref_5fcommon_2eproto, file_level_service_descriptors_vssref_5fcommon_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_vssref_5fcommon_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_vssref_5fcommon_2eproto)), true);
namespace VSSRef {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Foul_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_vssref_5fcommon_2eproto);
  return file_level_enum_descriptors_vssref_5fcommon_2eproto[0];
}
bool Foul_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
      return true;
    default:
      return false;
  }
}

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Color_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_vssref_5fcommon_2eproto);
  return file_level_enum_descriptors_vssref_5fcommon_2eproto[1];
}
bool Color_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Quadrant_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_vssref_5fcommon_2eproto);
  return file_level_enum_descriptors_vssref_5fcommon_2eproto[2];
}
bool Quadrant_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
      return true;
    default:
      return false;
  }
}

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Half_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_vssref_5fcommon_2eproto);
  return file_level_enum_descriptors_vssref_5fcommon_2eproto[3];
}
bool Half_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}


// ===================================================================

void Robot::InitAsDefaultInstance() {
}
class Robot::_Internal {
 public:
};

Robot::Robot(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:VSSRef.Robot)
}
Robot::Robot(const Robot& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&x_, &from.x_,
    static_cast<size_t>(reinterpret_cast<char*>(&robot_id_) -
    reinterpret_cast<char*>(&x_)) + sizeof(robot_id_));
  // @@protoc_insertion_point(copy_constructor:VSSRef.Robot)
}

void Robot::SharedCtor() {
  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&robot_id_) -
      reinterpret_cast<char*>(&x_)) + sizeof(robot_id_));
}

Robot::~Robot() {
  // @@protoc_insertion_point(destructor:VSSRef.Robot)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void Robot::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
}

void Robot::ArenaDtor(void* object) {
  Robot* _this = reinterpret_cast< Robot* >(object);
  (void)_this;
}
void Robot::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Robot::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Robot& Robot::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Robot_vssref_5fcommon_2eproto.base);
  return *internal_default_instance();
}


void Robot::Clear() {
// @@protoc_insertion_point(message_clear_start:VSSRef.Robot)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&robot_id_) -
      reinterpret_cast<char*>(&x_)) + sizeof(robot_id_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Robot::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // uint32 robot_id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          robot_id_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // double x = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          x_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // double y = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 25)) {
          y_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // double orientation = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 33)) {
          orientation_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
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

::PROTOBUF_NAMESPACE_ID::uint8* Robot::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:VSSRef.Robot)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // uint32 robot_id = 1;
  if (this->robot_id() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1, this->_internal_robot_id(), target);
  }

  // double x = 2;
  if (!(this->x() <= 0 && this->x() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_x(), target);
  }

  // double y = 3;
  if (!(this->y() <= 0 && this->y() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_y(), target);
  }

  // double orientation = 4;
  if (!(this->orientation() <= 0 && this->orientation() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_orientation(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:VSSRef.Robot)
  return target;
}

size_t Robot::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:VSSRef.Robot)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double x = 2;
  if (!(this->x() <= 0 && this->x() >= 0)) {
    total_size += 1 + 8;
  }

  // double y = 3;
  if (!(this->y() <= 0 && this->y() >= 0)) {
    total_size += 1 + 8;
  }

  // double orientation = 4;
  if (!(this->orientation() <= 0 && this->orientation() >= 0)) {
    total_size += 1 + 8;
  }

  // uint32 robot_id = 1;
  if (this->robot_id() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
        this->_internal_robot_id());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Robot::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:VSSRef.Robot)
  GOOGLE_DCHECK_NE(&from, this);
  const Robot* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Robot>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:VSSRef.Robot)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:VSSRef.Robot)
    MergeFrom(*source);
  }
}

void Robot::MergeFrom(const Robot& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:VSSRef.Robot)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (!(from.x() <= 0 && from.x() >= 0)) {
    _internal_set_x(from._internal_x());
  }
  if (!(from.y() <= 0 && from.y() >= 0)) {
    _internal_set_y(from._internal_y());
  }
  if (!(from.orientation() <= 0 && from.orientation() >= 0)) {
    _internal_set_orientation(from._internal_orientation());
  }
  if (from.robot_id() != 0) {
    _internal_set_robot_id(from._internal_robot_id());
  }
}

void Robot::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:VSSRef.Robot)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Robot::CopyFrom(const Robot& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:VSSRef.Robot)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Robot::IsInitialized() const {
  return true;
}

void Robot::InternalSwap(Robot* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Robot, robot_id_)
      + sizeof(Robot::robot_id_)
      - PROTOBUF_FIELD_OFFSET(Robot, x_)>(
          reinterpret_cast<char*>(&x_),
          reinterpret_cast<char*>(&other->x_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Robot::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void Frame::InitAsDefaultInstance() {
}
class Frame::_Internal {
 public:
};

Frame::Frame(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena),
  robots_(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:VSSRef.Frame)
}
Frame::Frame(const Frame& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      robots_(from.robots_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  teamcolor_ = from.teamcolor_;
  // @@protoc_insertion_point(copy_constructor:VSSRef.Frame)
}

void Frame::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_Frame_vssref_5fcommon_2eproto.base);
  teamcolor_ = 0;
}

Frame::~Frame() {
  // @@protoc_insertion_point(destructor:VSSRef.Frame)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void Frame::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
}

void Frame::ArenaDtor(void* object) {
  Frame* _this = reinterpret_cast< Frame* >(object);
  (void)_this;
}
void Frame::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Frame::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Frame& Frame::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Frame_vssref_5fcommon_2eproto.base);
  return *internal_default_instance();
}


void Frame::Clear() {
// @@protoc_insertion_point(message_clear_start:VSSRef.Frame)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  robots_.Clear();
  teamcolor_ = 0;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Frame::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // .VSSRef.Color teamColor = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_teamcolor(static_cast<::VSSRef::Color>(val));
        } else goto handle_unusual;
        continue;
      // repeated .VSSRef.Robot robots = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_robots(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
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

::PROTOBUF_NAMESPACE_ID::uint8* Frame::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:VSSRef.Frame)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .VSSRef.Color teamColor = 1;
  if (this->teamcolor() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1, this->_internal_teamcolor(), target);
  }

  // repeated .VSSRef.Robot robots = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_robots_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, this->_internal_robots(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:VSSRef.Frame)
  return target;
}

size_t Frame::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:VSSRef.Frame)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .VSSRef.Robot robots = 2;
  total_size += 1UL * this->_internal_robots_size();
  for (const auto& msg : this->robots_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // .VSSRef.Color teamColor = 1;
  if (this->teamcolor() != 0) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_teamcolor());
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Frame::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:VSSRef.Frame)
  GOOGLE_DCHECK_NE(&from, this);
  const Frame* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Frame>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:VSSRef.Frame)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:VSSRef.Frame)
    MergeFrom(*source);
  }
}

void Frame::MergeFrom(const Frame& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:VSSRef.Frame)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  robots_.MergeFrom(from.robots_);
  if (from.teamcolor() != 0) {
    _internal_set_teamcolor(from._internal_teamcolor());
  }
}

void Frame::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:VSSRef.Frame)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Frame::CopyFrom(const Frame& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:VSSRef.Frame)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Frame::IsInitialized() const {
  return true;
}

void Frame::InternalSwap(Frame* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  robots_.InternalSwap(&other->robots_);
  swap(teamcolor_, other->teamcolor_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Frame::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace VSSRef
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::VSSRef::Robot* Arena::CreateMaybeMessage< ::VSSRef::Robot >(Arena* arena) {
  return Arena::CreateMessageInternal< ::VSSRef::Robot >(arena);
}
template<> PROTOBUF_NOINLINE ::VSSRef::Frame* Arena::CreateMaybeMessage< ::VSSRef::Frame >(Arena* arena) {
  return Arena::CreateMessageInternal< ::VSSRef::Frame >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
