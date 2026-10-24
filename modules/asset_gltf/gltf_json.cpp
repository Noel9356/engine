#include "gltf_json.hpp"

#include <cctype>
#include <cstdlib>
#include <sstream>

namespace engine
{

namespace
{

class JsonParser
{
public:
    JsonParser(
        const std::string &text, std::string &errorMessage
    )
        : text_(text), errorMessage_(errorMessage)
    {
    }

    std::unique_ptr<JsonValue> parseDocument()
    {
        try
        {
            skipWhitespace();
            std::unique_ptr<JsonValue> value = parseValue(0);
            if (value == nullptr)
            {
                return nullptr;
            }
            skipWhitespace();
            if (position_ != text_.size())
            {
                return fail("unexpected trailing content after the document");
            }
            return value;
        }
        catch (const JsonParseAbort &)
        {
            return nullptr;
        }
    }

private:
    static constexpr int maxDepth = 64;

    const std::string &text_;
    std::string       &errorMessage_;
    std::size_t        position_ = 0;

    [[noreturn]] std::unique_ptr<JsonValue> fail(
        const std::string &problem
    )
    {
        std::ostringstream stream;
        stream << "JSON parse error at byte " << position_ << ": " << problem;
        errorMessage_ = stream.str();
        // The call sites always check for nullptr right after; the throw
        // unwinds without them needing per-step error plumbing.
        throw JsonParseAbort();
    }

    struct JsonParseAbort
    {
    };

    void skipWhitespace()
    {
        while (position_ < text_.size())
        {
            const char character = text_[position_];
            if (character == ' ' || character == '\t' || character == '\n' || character == '\r')
            {
                ++position_;
            }
            else
            {
                break;
            }
        }
    }

    bool consume(
        char expected
    )
    {
        if (position_ < text_.size() && text_[position_] == expected)
        {
            ++position_;
            return true;
        }
        return false;
    }

    void expect(
        char expected
    )
    {
        if (!consume(expected))
        {
            fail(std::string("expected '") + expected + "'");
        }
    }

    std::unique_ptr<JsonValue> parseValue(
        int depth
    )
    {
        if (depth > maxDepth)
        {
            fail("nesting deeper than 64 levels");
        }
        try
        {
            skipWhitespace();
            if (position_ >= text_.size())
            {
                return fail("unexpected end of input");
            }
            const char character = text_[position_];
            switch (character)
            {
                case '{':
                    return parseObject(depth);
                case '[':
                    return parseArray(depth);
                case '"':
                {
                    auto value    = std::make_unique<JsonValue>();
                    value->type   = JsonValue::Type::String;
                    value->string = parseString();
                    return value;
                }
                case 't':
                    return parseLiteral("true", JsonValue::Type::Boolean, true);
                case 'f':
                    return parseLiteral("false", JsonValue::Type::Boolean, false);
                case 'n':
                    return parseLiteral("null", JsonValue::Type::Null, false);
                default:
                    return parseNumber();
            }
        }
        catch (const JsonParseAbort &)
        {
            return nullptr;
        }
    }

    std::unique_ptr<JsonValue> parseObject(
        int depth
    )
    {
        expect('{');
        auto value = std::make_unique<JsonValue>();
        value->type = JsonValue::Type::Object;
        skipWhitespace();
        if (consume('}'))
        {
            return value;
        }
        while (true)
        {
            skipWhitespace();
            if (position_ >= text_.size() || text_[position_] != '"')
            {
                fail("expected a string key");
            }
            const std::string key = parseString();
            skipWhitespace();
            expect(':');
            auto memberValue = parseValue(depth + 1);
            if (memberValue == nullptr)
            {
                fail("invalid value");
            }
            value->memberIndex[key] = value->members.size();
            value->members.emplace_back(key, std::move(*memberValue));
            skipWhitespace();
            if (consume(','))
            {
                continue;
            }
            expect('}');
            break;
        }
        return value;
    }

    std::unique_ptr<JsonValue> parseArray(
        int depth
    )
    {
        expect('[');
        auto value = std::make_unique<JsonValue>();
        value->type = JsonValue::Type::Array;
        skipWhitespace();
        if (consume(']'))
        {
            return value;
        }
        while (true)
        {
            auto element = parseValue(depth + 1);
            if (element == nullptr)
            {
                fail("invalid array element");
            }
            value->array.push_back(std::move(*element));
            skipWhitespace();
            if (consume(','))
            {
                continue;
            }
            expect(']');
            break;
        }
        return value;
    }

    std::string parseString()
    {
        expect('"');
        std::string result;
        while (position_ < text_.size())
        {
            const char character = text_[position_++];
            if (character == '"')
            {
                return result;
            }
            if (character == '\\')
            {
                if (position_ >= text_.size())
                {
                    fail("unterminated escape sequence");
                }
                const char escape = text_[position_++];
                switch (escape)
                {
                    case '"': result.push_back('"'); break;
                    case '\\': result.push_back('\\'); break;
                    case '/': result.push_back('/'); break;
                    case 'b': result.push_back('\b'); break;
                    case 'f': result.push_back('\f'); break;
                    case 'n': result.push_back('\n'); break;
                    case 'r': result.push_back('\r'); break;
                    case 't': result.push_back('\t'); break;
                    case 'u':
                    {
                        // glTF uses \uXXXX for non-ASCII; the importer only
                        // reads ASCII names, so decode to UTF-8 for the BMP
                        // range and refuse surrogates (they never appear in
                        // the fields this parser is used for).
                        if (position_ + 4 > text_.size())
                        {
                            fail("truncated \\u escape");
                        }
                        const std::string hex = text_.substr(position_, 4);
                        position_ += 4;
                        const unsigned long codePoint = std::strtoul(hex.c_str(), nullptr, 16);
                        if (codePoint >= 0xD800ul && codePoint <= 0xDFFFul)
                        {
                            fail("surrogate \\u escapes are not supported");
                        }
                        if (codePoint < 0x80ul)
                        {
                            result.push_back(static_cast<char>(codePoint));
                        }
                        else if (codePoint < 0x800ul)
                        {
                            result.push_back(static_cast<char>(0xC0ul | (codePoint >> 6)));
                            result.push_back(static_cast<char>(0x80ul | (codePoint & 0x3Ful)));
                        }
                        else
                        {
                            result.push_back(static_cast<char>(0xE0ul | (codePoint >> 12)));
                            result.push_back(static_cast<char>(0x80ul | ((codePoint >> 6) & 0x3Ful)));
                            result.push_back(static_cast<char>(0x80ul | (codePoint & 0x3Ful)));
                        }
                        break;
                    }
                    default:
                        fail("unknown escape sequence");
                }
                continue;
            }
            if (static_cast<unsigned char>(character) < 0x20)
            {
                fail("raw control character inside a string");
            }
            result.push_back(character);
        }
        fail("unterminated string");
    }

    std::unique_ptr<JsonValue> parseLiteral(
        const char *literal, JsonValue::Type type, bool boolean
    )
    {
        const std::size_t length = std::char_traits<char>::length(literal);
        if (text_.compare(position_, length, literal) != 0)
        {
            fail("invalid literal");
        }
        position_ += length;
        auto value    = std::make_unique<JsonValue>();
        value->type   = type;
        value->boolean = boolean;
        return value;
    }

    std::unique_ptr<JsonValue> parseNumber()
    {
        const std::size_t start = position_;
        if (position_ < text_.size() && (text_[position_] == '-' || text_[position_] == '+'))
        {
            ++position_;
        }
        bool digits = false;
        while (position_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[position_])))
        {
            ++position_;
            digits = true;
        }
        if (position_ < text_.size() && text_[position_] == '.')
        {
            ++position_;
            while (position_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[position_])))
            {
                ++position_;
                digits = true;
            }
        }
        if (position_ < text_.size() && (text_[position_] == 'e' || text_[position_] == 'E'))
        {
            ++position_;
            if (position_ < text_.size() && (text_[position_] == '-' || text_[position_] == '+'))
            {
                ++position_;
            }
            while (position_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[position_])))
            {
                ++position_;
            }
        }
        if (!digits)
        {
            fail("invalid number");
        }
        const std::string numberText = text_.substr(start, position_ - start);
        char              *end       = nullptr;
        const double       parsed    = std::strtod(numberText.c_str(), &end);
        if (end == nullptr || *end != '\0' || !std::isfinite(parsed))
        {
            fail("number is not a finite double");
        }
        auto value  = std::make_unique<JsonValue>();
        value->type = JsonValue::Type::Number;
        value->number = parsed;
        return value;
    }
};

} // namespace

std::unique_ptr<JsonValue> jsonParse(
    const std::string &text, std::string &errorMessage
)
{
    JsonParser parser(text, errorMessage);
    return parser.parseDocument();
}

} // namespace engine
