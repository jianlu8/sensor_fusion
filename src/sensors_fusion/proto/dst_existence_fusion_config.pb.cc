// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dst_existence_fusion_config.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "dst_existence_fusion_config.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace apollo {
namespace perception {
class CameraValidDistDefaultTypeInternal : public ::google::protobuf::internal::ExplicitlyConstructed<CameraValidDist> {
} _CameraValidDist_default_instance_;
class DstExistenceFusionConfigDefaultTypeInternal : public ::google::protobuf::internal::ExplicitlyConstructed<DstExistenceFusionConfig> {
} _DstExistenceFusionConfig_default_instance_;

namespace protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto {


namespace {

::google::protobuf::Metadata file_level_metadata[2];

}  // namespace

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTableField
    const TableStruct::entries[] = {
  {0, 0, 0, ::google::protobuf::internal::kInvalidMask, 0, 0},
};

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::AuxillaryParseTableField
    const TableStruct::aux[] = {
  ::google::protobuf::internal::AuxillaryParseTableField(),
};
PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTable const
    TableStruct::schema[] = {
  { NULL, NULL, 0, -1, -1, false },
  { NULL, NULL, 0, -1, -1, false },
};

const ::google::protobuf::uint32 TableStruct::offsets[] = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CameraValidDist, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CameraValidDist, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CameraValidDist, camera_name_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CameraValidDist, valid_dist_),
  0,
  1,
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(DstExistenceFusionConfig, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(DstExistenceFusionConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(DstExistenceFusionConfig, track_object_max_match_distance_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(DstExistenceFusionConfig, camera_valid_dist_),
  0,
  ~0u,
};

static const ::google::protobuf::internal::MigrationSchema schemas[] = {
  { 0, 7, sizeof(CameraValidDist)},
  { 9, 16, sizeof(DstExistenceFusionConfig)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_CameraValidDist_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&_DstExistenceFusionConfig_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "dst_existence_fusion_config.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 2);
}

}  // namespace

void TableStruct::Shutdown() {
  _CameraValidDist_default_instance_.Shutdown();
  delete file_level_metadata[0].reflection;
  _DstExistenceFusionConfig_default_instance_.Shutdown();
  delete file_level_metadata[1].reflection;
}

void TableStruct::InitDefaultsImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::internal::InitProtobufDefaults();
  _CameraValidDist_default_instance_.DefaultConstruct();
  _DstExistenceFusionConfig_default_instance_.DefaultConstruct();
}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] = {
      "\n!dst_existence_fusion_config.proto\022\021apo"
      "llo.perception\"\?\n\017CameraValidDist\022\025\n\013cam"
      "era_name\030\001 \001(\t:\000\022\025\n\nvalid_dist\030\002 \001(\001:\0010\""
      "\205\001\n\030DstExistenceFusionConfig\022*\n\037track_ob"
      "ject_max_match_distance\030\001 \001(\001:\0014\022=\n\021came"
      "ra_valid_dist\030\002 \003(\0132\".apollo.perception."
      "CameraValidDist"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 255);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "dst_existence_fusion_config.proto", &protobuf_RegisterTypes);
  ::google::protobuf::internal::OnShutdown(&TableStruct::Shutdown);
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;

}  // namespace protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int CameraValidDist::kCameraNameFieldNumber;
const int CameraValidDist::kValidDistFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

CameraValidDist::CameraValidDist()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.perception.CameraValidDist)
}
CameraValidDist::CameraValidDist(const CameraValidDist& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  camera_name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_camera_name()) {
    camera_name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.camera_name_);
  }
  valid_dist_ = from.valid_dist_;
  // @@protoc_insertion_point(copy_constructor:apollo.perception.CameraValidDist)
}

void CameraValidDist::SharedCtor() {
  _cached_size_ = 0;
  camera_name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  valid_dist_ = 0;
}

CameraValidDist::~CameraValidDist() {
  // @@protoc_insertion_point(destructor:apollo.perception.CameraValidDist)
  SharedDtor();
}

void CameraValidDist::SharedDtor() {
  camera_name_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

void CameraValidDist::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* CameraValidDist::descriptor() {
  protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const CameraValidDist& CameraValidDist::default_instance() {
  protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::InitDefaults();
  return *internal_default_instance();
}

CameraValidDist* CameraValidDist::New(::google::protobuf::Arena* arena) const {
  CameraValidDist* n = new CameraValidDist;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void CameraValidDist::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.CameraValidDist)
  if (has_camera_name()) {
    GOOGLE_DCHECK(!camera_name_.IsDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited()));
    (*camera_name_.UnsafeRawStringPointer())->clear();
  }
  valid_dist_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool CameraValidDist::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.perception.CameraValidDist)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional string camera_name = 1 [default = ""];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_camera_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->camera_name().data(), this->camera_name().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "apollo.perception.CameraValidDist.camera_name");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional double valid_dist = 2 [default = 0];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u)) {
          set_has_valid_dist();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &valid_dist_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:apollo.perception.CameraValidDist)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.perception.CameraValidDist)
  return false;
#undef DO_
}

void CameraValidDist::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.perception.CameraValidDist)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string camera_name = 1 [default = ""];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->camera_name().data(), this->camera_name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.CameraValidDist.camera_name");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->camera_name(), output);
  }

  // optional double valid_dist = 2 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->valid_dist(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.perception.CameraValidDist)
}

::google::protobuf::uint8* CameraValidDist::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.CameraValidDist)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string camera_name = 1 [default = ""];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->camera_name().data(), this->camera_name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "apollo.perception.CameraValidDist.camera_name");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->camera_name(), target);
  }

  // optional double valid_dist = 2 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->valid_dist(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.CameraValidDist)
  return target;
}

size_t CameraValidDist::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.CameraValidDist)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  if (_has_bits_[0 / 32] & 3u) {
    // optional string camera_name = 1 [default = ""];
    if (has_camera_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->camera_name());
    }

    // optional double valid_dist = 2 [default = 0];
    if (has_valid_dist()) {
      total_size += 1 + 8;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void CameraValidDist::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.perception.CameraValidDist)
  GOOGLE_DCHECK_NE(&from, this);
  const CameraValidDist* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const CameraValidDist>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.perception.CameraValidDist)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.perception.CameraValidDist)
    MergeFrom(*source);
  }
}

void CameraValidDist::MergeFrom(const CameraValidDist& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.CameraValidDist)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_camera_name();
      camera_name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.camera_name_);
    }
    if (cached_has_bits & 0x00000002u) {
      valid_dist_ = from.valid_dist_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void CameraValidDist::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.perception.CameraValidDist)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CameraValidDist::CopyFrom(const CameraValidDist& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.CameraValidDist)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CameraValidDist::IsInitialized() const {
  return true;
}

void CameraValidDist::Swap(CameraValidDist* other) {
  if (other == this) return;
  InternalSwap(other);
}
void CameraValidDist::InternalSwap(CameraValidDist* other) {
  camera_name_.Swap(&other->camera_name_);
  std::swap(valid_dist_, other->valid_dist_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata CameraValidDist::GetMetadata() const {
  protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// CameraValidDist

// optional string camera_name = 1 [default = ""];
bool CameraValidDist::has_camera_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void CameraValidDist::set_has_camera_name() {
  _has_bits_[0] |= 0x00000001u;
}
void CameraValidDist::clear_has_camera_name() {
  _has_bits_[0] &= ~0x00000001u;
}
void CameraValidDist::clear_camera_name() {
  camera_name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_camera_name();
}
const ::std::string& CameraValidDist::camera_name() const {
  // @@protoc_insertion_point(field_get:apollo.perception.CameraValidDist.camera_name)
  return camera_name_.GetNoArena();
}
void CameraValidDist::set_camera_name(const ::std::string& value) {
  set_has_camera_name();
  camera_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.perception.CameraValidDist.camera_name)
}
#if LANG_CXX11
void CameraValidDist::set_camera_name(::std::string&& value) {
  set_has_camera_name();
  camera_name_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.perception.CameraValidDist.camera_name)
}
#endif
void CameraValidDist::set_camera_name(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_camera_name();
  camera_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.perception.CameraValidDist.camera_name)
}
void CameraValidDist::set_camera_name(const char* value, size_t size) {
  set_has_camera_name();
  camera_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.perception.CameraValidDist.camera_name)
}
::std::string* CameraValidDist::mutable_camera_name() {
  set_has_camera_name();
  // @@protoc_insertion_point(field_mutable:apollo.perception.CameraValidDist.camera_name)
  return camera_name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
::std::string* CameraValidDist::release_camera_name() {
  // @@protoc_insertion_point(field_release:apollo.perception.CameraValidDist.camera_name)
  clear_has_camera_name();
  return camera_name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
void CameraValidDist::set_allocated_camera_name(::std::string* camera_name) {
  if (camera_name != NULL) {
    set_has_camera_name();
  } else {
    clear_has_camera_name();
  }
  camera_name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), camera_name);
  // @@protoc_insertion_point(field_set_allocated:apollo.perception.CameraValidDist.camera_name)
}

// optional double valid_dist = 2 [default = 0];
bool CameraValidDist::has_valid_dist() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void CameraValidDist::set_has_valid_dist() {
  _has_bits_[0] |= 0x00000002u;
}
void CameraValidDist::clear_has_valid_dist() {
  _has_bits_[0] &= ~0x00000002u;
}
void CameraValidDist::clear_valid_dist() {
  valid_dist_ = 0;
  clear_has_valid_dist();
}
double CameraValidDist::valid_dist() const {
  // @@protoc_insertion_point(field_get:apollo.perception.CameraValidDist.valid_dist)
  return valid_dist_;
}
void CameraValidDist::set_valid_dist(double value) {
  set_has_valid_dist();
  valid_dist_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.CameraValidDist.valid_dist)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int DstExistenceFusionConfig::kTrackObjectMaxMatchDistanceFieldNumber;
const int DstExistenceFusionConfig::kCameraValidDistFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

DstExistenceFusionConfig::DstExistenceFusionConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.perception.DstExistenceFusionConfig)
}
DstExistenceFusionConfig::DstExistenceFusionConfig(const DstExistenceFusionConfig& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0),
      camera_valid_dist_(from.camera_valid_dist_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  track_object_max_match_distance_ = from.track_object_max_match_distance_;
  // @@protoc_insertion_point(copy_constructor:apollo.perception.DstExistenceFusionConfig)
}

void DstExistenceFusionConfig::SharedCtor() {
  _cached_size_ = 0;
  track_object_max_match_distance_ = 4;
}

DstExistenceFusionConfig::~DstExistenceFusionConfig() {
  // @@protoc_insertion_point(destructor:apollo.perception.DstExistenceFusionConfig)
  SharedDtor();
}

void DstExistenceFusionConfig::SharedDtor() {
}

void DstExistenceFusionConfig::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* DstExistenceFusionConfig::descriptor() {
  protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const DstExistenceFusionConfig& DstExistenceFusionConfig::default_instance() {
  protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::InitDefaults();
  return *internal_default_instance();
}

DstExistenceFusionConfig* DstExistenceFusionConfig::New(::google::protobuf::Arena* arena) const {
  DstExistenceFusionConfig* n = new DstExistenceFusionConfig;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void DstExistenceFusionConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.perception.DstExistenceFusionConfig)
  camera_valid_dist_.Clear();
  track_object_max_match_distance_ = 4;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool DstExistenceFusionConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.perception.DstExistenceFusionConfig)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double track_object_max_match_distance = 1 [default = 4];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u)) {
          set_has_track_object_max_match_distance();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &track_object_max_match_distance_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_camera_valid_dist()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:apollo.perception.DstExistenceFusionConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.perception.DstExistenceFusionConfig)
  return false;
#undef DO_
}

void DstExistenceFusionConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.perception.DstExistenceFusionConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double track_object_max_match_distance = 1 [default = 4];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->track_object_max_match_distance(), output);
  }

  // repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
  for (unsigned int i = 0, n = this->camera_valid_dist_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->camera_valid_dist(i), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.perception.DstExistenceFusionConfig)
}

::google::protobuf::uint8* DstExistenceFusionConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.perception.DstExistenceFusionConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double track_object_max_match_distance = 1 [default = 4];
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->track_object_max_match_distance(), target);
  }

  // repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
  for (unsigned int i = 0, n = this->camera_valid_dist_size(); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        2, this->camera_valid_dist(i), deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.perception.DstExistenceFusionConfig)
  return target;
}

size_t DstExistenceFusionConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.perception.DstExistenceFusionConfig)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  // repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
  {
    unsigned int count = this->camera_valid_dist_size();
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->camera_valid_dist(i));
    }
  }

  // optional double track_object_max_match_distance = 1 [default = 4];
  if (has_track_object_max_match_distance()) {
    total_size += 1 + 8;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void DstExistenceFusionConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.perception.DstExistenceFusionConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const DstExistenceFusionConfig* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const DstExistenceFusionConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.perception.DstExistenceFusionConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.perception.DstExistenceFusionConfig)
    MergeFrom(*source);
  }
}

void DstExistenceFusionConfig::MergeFrom(const DstExistenceFusionConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.perception.DstExistenceFusionConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  camera_valid_dist_.MergeFrom(from.camera_valid_dist_);
  if (from.has_track_object_max_match_distance()) {
    set_track_object_max_match_distance(from.track_object_max_match_distance());
  }
}

void DstExistenceFusionConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.perception.DstExistenceFusionConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DstExistenceFusionConfig::CopyFrom(const DstExistenceFusionConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.perception.DstExistenceFusionConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DstExistenceFusionConfig::IsInitialized() const {
  return true;
}

void DstExistenceFusionConfig::Swap(DstExistenceFusionConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void DstExistenceFusionConfig::InternalSwap(DstExistenceFusionConfig* other) {
  camera_valid_dist_.InternalSwap(&other->camera_valid_dist_);
  std::swap(track_object_max_match_distance_, other->track_object_max_match_distance_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata DstExistenceFusionConfig::GetMetadata() const {
  protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_dst_5fexistence_5ffusion_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// DstExistenceFusionConfig

// optional double track_object_max_match_distance = 1 [default = 4];
bool DstExistenceFusionConfig::has_track_object_max_match_distance() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void DstExistenceFusionConfig::set_has_track_object_max_match_distance() {
  _has_bits_[0] |= 0x00000001u;
}
void DstExistenceFusionConfig::clear_has_track_object_max_match_distance() {
  _has_bits_[0] &= ~0x00000001u;
}
void DstExistenceFusionConfig::clear_track_object_max_match_distance() {
  track_object_max_match_distance_ = 4;
  clear_has_track_object_max_match_distance();
}
double DstExistenceFusionConfig::track_object_max_match_distance() const {
  // @@protoc_insertion_point(field_get:apollo.perception.DstExistenceFusionConfig.track_object_max_match_distance)
  return track_object_max_match_distance_;
}
void DstExistenceFusionConfig::set_track_object_max_match_distance(double value) {
  set_has_track_object_max_match_distance();
  track_object_max_match_distance_ = value;
  // @@protoc_insertion_point(field_set:apollo.perception.DstExistenceFusionConfig.track_object_max_match_distance)
}

// repeated .apollo.perception.CameraValidDist camera_valid_dist = 2;
int DstExistenceFusionConfig::camera_valid_dist_size() const {
  return camera_valid_dist_.size();
}
void DstExistenceFusionConfig::clear_camera_valid_dist() {
  camera_valid_dist_.Clear();
}
const ::apollo::perception::CameraValidDist& DstExistenceFusionConfig::camera_valid_dist(int index) const {
  // @@protoc_insertion_point(field_get:apollo.perception.DstExistenceFusionConfig.camera_valid_dist)
  return camera_valid_dist_.Get(index);
}
::apollo::perception::CameraValidDist* DstExistenceFusionConfig::mutable_camera_valid_dist(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.perception.DstExistenceFusionConfig.camera_valid_dist)
  return camera_valid_dist_.Mutable(index);
}
::apollo::perception::CameraValidDist* DstExistenceFusionConfig::add_camera_valid_dist() {
  // @@protoc_insertion_point(field_add:apollo.perception.DstExistenceFusionConfig.camera_valid_dist)
  return camera_valid_dist_.Add();
}
::google::protobuf::RepeatedPtrField< ::apollo::perception::CameraValidDist >*
DstExistenceFusionConfig::mutable_camera_valid_dist() {
  // @@protoc_insertion_point(field_mutable_list:apollo.perception.DstExistenceFusionConfig.camera_valid_dist)
  return &camera_valid_dist_;
}
const ::google::protobuf::RepeatedPtrField< ::apollo::perception::CameraValidDist >&
DstExistenceFusionConfig::camera_valid_dist() const {
  // @@protoc_insertion_point(field_list:apollo.perception.DstExistenceFusionConfig.camera_valid_dist)
  return camera_valid_dist_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace perception
}  // namespace apollo

// @@protoc_insertion_point(global_scope)
