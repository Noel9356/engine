#ifndef ENGINE_MODULES_ASSET_GLTF_JSON_HPP
#define ENGINE_MODULES_ASSET_GLTF_JSON_HPP

// A minimal JSON reader for glTF 2.0 files.
//
// The engine's dependency policy (cmake/engine_dependencies.cmake) pins
// exactly the dependencies the runtime needs; a JSON library would be one
// more fetched tree for what is, honestly, a narrow read problem: the glTF
// importer only consumes the JSON document's structural parts (nodes,
// meshes, accessors, materials) - geometry stays in the binary buffers,
// which this importer never decodes (bounding boxes come from the
// accessor.min/max fields, which live in the JSON itself). That narrow
// problem is ~200 lines of recursive descent, no exceptions, no locale
// surprises, and one precise error message per malformed document.
//
// The parser is deliberately strict about what it accepts (RFC 8259 subset:
// no comments, no trailing commas, no NaN/Infinity) because glTF files are
// machine written; leniency here would hide exporter bugs.

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace engine
{

class JsonValue;
using JsonObjectMember = std::pair<std::string, JsonValue>;

// One JSON value. Owned children (unique_ptr) keep the tree allocation
// simple; glTF documents are parsed once per import, so shared ownership
// would buy nothing.
class JsonValue
{
public:
    enum class Type
    {
        Null,
        Boolean,
        Number,
        String,
        Array,
        Object
    };

    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string string;
    std::vector<JsonValue> array;
    // Object members in document order plus an index for key lookup: glTF
    // access is key driven and repeated lookups should not scan.
    std::vector<JsonObjectMember> members;
    std::map<std::string, std::size_t> memberIndex;

    bool isNull() const { return type == Type::Null; }
    bool isBoolean() const { return type == Type::Boolean; }
    bool isNumber() const { return type == Type::Number; }
    bool isString() const { return type == Type::String; }
    bool isArray() const { return type == Type::Array; }
    bool isObject() const { return type == Type::Object; }

    const JsonValue *find(const std::string &key) const
    {
        if (type != Type::Object)
        {
            return nullptr;
        }
        const auto found = memberIndex.find(key);
        return found == memberIndex.end() ? nullptr : &members[found->second].second;
    }

    // Convenience typed getters with defaults; they never throw.
    double numberOr(double fallback) const { return isNumber() ? number : fallback; }
    bool booleanOr(bool fallback) const { return isBoolean() ? boolean : fallback; }
    std::string stringOr(const std::string &fallback) const { return isString() ? string : fallback; }
};

// Parses one JSON document. On failure returns nullptr and fills errorMessage
// with the position and the problem - the caller reports the file name around
// it.
std::unique_ptr<JsonValue> jsonParse(
    const std::string &text, std::string &errorMessage
);

} // namespace engine

#endif // ENGINE_MODULES_ASSET_GLTF_JSON_HPP
