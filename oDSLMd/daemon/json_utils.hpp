/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * json_utils.hpp - Extremely small JSON helpers for the rpicam daemon.
 */

#pragma once

#include <cctype>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace rpicam
{

struct JsonValue
{
        std::string text;
        bool is_string = false;

        std::optional<double> asNumber() const
        {
                if (is_string)
                        return std::nullopt;
                try
                {
                        size_t processed = 0;
                        double value = std::stod(text, &processed);
                        if (processed != text.size())
                                return std::nullopt;
                        return value;
                }
                catch (...) { return std::nullopt; }
        }

        std::optional<bool> asBool() const
        {
                if (is_string)
                        return std::nullopt;
                if (text == "true")
                        return true;
                if (text == "false")
                        return false;
                return std::nullopt;
        }

        std::optional<std::string> asString() const
        {
                if (!is_string)
                        return std::nullopt;
                return text;
        }
};

using JsonObject = std::unordered_map<std::string, JsonValue>;

inline void skipWhitespace(std::string const &body, size_t &index)
{
        while (index < body.size() && std::isspace(static_cast<unsigned char>(body[index])))
                ++index;
}

inline bool expectChar(std::string const &body, size_t &index, char expected)
{
        skipWhitespace(body, index);
        if (index >= body.size() || body[index] != expected)
                return false;
        ++index;
        return true;
}

inline std::optional<std::string> parseString(std::string const &body, size_t &index)
{
        skipWhitespace(body, index);
        if (index >= body.size() || body[index] != '"')
                return std::nullopt;
        ++index;
        std::string value;
        while (index < body.size())
        {
                char ch = body[index++];
                if (ch == '"')
                        return value;
                if (ch == '\\' && index < body.size())
                {
                        char escape = body[index++];
                        switch (escape)
                        {
                        case '"': value.push_back('"'); break;
                        case '\\': value.push_back('\\'); break;
                        case 'n': value.push_back('\n'); break;
                        case 'r': value.push_back('\r'); break;
                        case 't': value.push_back('\t'); break;
                        default: value.push_back(escape); break;
                        }
                }
                else
                        value.push_back(ch);
        }
        return std::nullopt;
}

inline JsonObject parseJsonObject(std::string const &body)
{
        JsonObject result;
        size_t index = 0;
        if (!expectChar(body, index, '{'))
                return result;

        while (index < body.size())
        {
                skipWhitespace(body, index);
                if (index < body.size() && body[index] == '}')
                {
                        ++index;
                        break;
                }

                auto key = parseString(body, index);
                if (!key)
                        break;
                if (!expectChar(body, index, ':'))
                        break;

                skipWhitespace(body, index);
                JsonValue value;
                if (index < body.size() && body[index] == '"')
                {
                        auto parsed = parseString(body, index);
                        if (!parsed)
                                break;
                        value.text = *parsed;
                        value.is_string = true;
                }
                else
                {
                        size_t start = index;
                        while (index < body.size() && body[index] != ',' && body[index] != '}')
                                ++index;
                        size_t end = index;
                        while (end > start && std::isspace(static_cast<unsigned char>(body[end - 1])))
                                --end;
                        value.text = body.substr(start, end - start);
                }

                result[*key] = value;

                skipWhitespace(body, index);
                if (index < body.size() && body[index] == ',')
                {
                        ++index;
                        continue;
                }
                if (index < body.size() && body[index] == '}')
                {
                        ++index;
                        break;
                }
        }

        return result;
}

inline std::string jsonEscape(std::string const &input)
{
        std::string output;
        output.reserve(input.size());
        for (char c : input)
        {
                switch (c)
                {
                case '"': output += "\\\""; break;
                case '\\': output += "\\\\"; break;
                case '\n': output += "\\n"; break;
                case '\r': output += "\\r"; break;
                case '\t': output += "\\t"; break;
                default: output.push_back(c); break;
                }
        }
        return output;
}

inline std::string jsonString(std::string const &value)
{
        return "\"" + jsonEscape(value) + "\"";
}

} // namespace rpicam

