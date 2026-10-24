#include "arguments.hpp"
#include "../io/json.hpp"
#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <memory>
#include <ranges>
#include <utility>

namespace bike::cli {
namespace {
#include "argument_contracts.inc"
const io::Value& member(const io::Value& value, std::string_view key) {
    const auto* found = io::find_member(io::as_object(value, "cli contract"), key);
    if (!found) throw std::logic_error("missing CLI contract member: " + std::string(key));
    return *found;
}
std::string scalar(const io::Value& value) {
    if (const auto* text = std::get_if<std::string>(&value.data)) return *text;
    if (const auto* num = std::get_if<io::Number>(&value.data)) return num->lexeme;
    if (const auto* flag = std::get_if<bool>(&value.data)) return *flag ? "true" : "false";
    throw std::logic_error("non-scalar CLI default");
}
const std::string* lookup(const Arguments& args, std::string_view key) {
    for (const auto& [name, value] : args.values) if (name == key) return &value;
    return nullptr;
}
std::string number_spelling(std::string_view value) {
    while(!value.empty()&&std::isspace(static_cast<unsigned char>(value.front())))value.remove_prefix(1);
    while(!value.empty()&&std::isspace(static_cast<unsigned char>(value.back())))value.remove_suffix(1);
    std::string result;result.reserve(value.size());
    for(std::size_t i=0;i<value.size();++i) {
        if(value[i]=='_'&&i>0&&i+1<value.size()&&value[i-1]>='0'&&value[i-1]<='9'&&value[i+1]>='0'&&value[i+1]<='9')continue;
        result.push_back(value[i]);
    }
    return result;
}
std::string integer_token(std::string_view input) {
    auto value=number_spelling(input);bool negative=false;
    if(!value.empty()&&(value.front()=='+'||value.front()=='-')) {
        negative=value.front()=='-';value.erase(0,1);
    }
    if(value.empty()||!std::ranges::all_of(value,[](char ch){return ch>='0'&&ch<='9';}))
        throw ArgumentError("invalid integer value: '"+std::string(input)+"'");
    const auto first=value.find_first_not_of('0');
    if(first==std::string::npos)return "0";
    value.erase(0,first);if(negative)value.insert(value.begin(),'-');
    return value;
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
double real_token(std::string_view value, std::string_view key) {
    const auto spelling=number_spelling(value);value=spelling;
    if (!value.empty() && value.front() == '+') value.remove_prefix(1);
    double result = 0.;
    const auto end = std::to_address(value.cend());
    const auto parsed = std::from_chars(value.data(), end, result); // NOLINT(bugprone-suspicious-stringview-data-usage) std::from_chars parses the (data, data+size) pointer range, not a C string
    if (parsed.ec != std::errc{} || parsed.ptr != end || !std::isfinite(result))
        throw ArgumentError("argument --" + std::string(key) + ": expected a finite number");
    return result;
}
void validate_token(const OptionSpec& option, const std::string& token) {
    if (token.find('\0') != std::string::npos) throw ArgumentError("argument contains NUL");
    if (option.type == "int") (void)integer_token(token);
    if (option.type == "float") (void)real_token(token, option.dest);
    if (!option.choices.empty() && std::ranges::find(option.choices, token) == option.choices.end()) {
        std::string message = "argument " + option.names.front() + ": invalid choice: '" + token + "' (choose from ";
        for (std::size_t i = 0; i < option.choices.size(); ++i) message += (i ? ", " : "") + option.choices[i];
        throw ArgumentError(message + ")");
    }
}
} // namespace

ArgumentError::~ArgumentError() = default;
bool Arguments::present(std::string_view key) const { return std::ranges::find(supplied, key) != supplied.end(); }
bool Arguments::has(std::string_view key) const { return lookup(*this, key) != nullptr; }
bool Arguments::flag(std::string_view key) const { return text(key) == "true"; }
std::string Arguments::text(std::string_view key, const std::string& fallback) const {
    const auto* value = lookup(*this, key); return value ? *value : fallback;
}
double Arguments::real(std::string_view key, double fallback) const {
    const auto* value = lookup(*this, key); return value ? real_token(*value, key) : fallback;
}
std::int64_t Arguments::integer(std::string_view key, std::int64_t fallback) const {
    const auto* value = lookup(*this, key);
    if (!value) return fallback;
    const auto spelling=integer_token(*value);
    auto token = std::string_view(spelling);
    if (!token.empty() && token.front() == '+') token.remove_prefix(1);
    std::int64_t result = 0;
    const auto end = std::to_address(token.cend());
    const auto parsed = std::from_chars(token.data(), end, result); // NOLINT(bugprone-suspicious-stringview-data-usage) std::from_chars parses the (data, data+size) pointer range, not a C string
    if (parsed.ec != std::errc{} || parsed.ptr != end) throw ArgumentError(std::string(key) + ": integer out of range");
    return result;
}
std::optional<std::string> Arguments::optional_text(std::string_view key) const {
    const auto* value = lookup(*this, key); return value ? std::optional(*value) : std::nullopt;
}
std::optional<double> Arguments::optional_real(std::string_view key) const {
    return has(key) ? std::optional(real(key)) : std::nullopt;
}
void Arguments::set(std::string key, std::string value, bool explicit_value) {
    if (explicit_value && !present(key)) supplied.push_back(key);
    for (auto& [name, stored] : values) if (name == key) { stored = std::move(value); return; }
    values.emplace_back(std::move(key), std::move(value));
}
std::vector<OptionSpec> option_specs(std::string_view command) {
    const auto contracts = io::parse_json(kArgumentContracts);
    const auto& entry = member(contracts, command);
    std::vector<OptionSpec> result;
    for (const auto& value : io::as_array(member(entry, "options"), "cli.options")) {
        OptionSpec spec;
        spec.dest = scalar(member(value, "dest")); spec.type = scalar(member(value, "type"));
        for (const auto& name : io::as_array(member(value, "names"), "cli.names")) spec.names.push_back(scalar(name));
        if (!io::is_null(member(value, "choices")))
            for (const auto& choice : io::as_array(member(value, "choices"), "cli.choices")) spec.choices.push_back(scalar(choice));
        if (!io::is_null(member(value, "default"))) spec.default_value = scalar(member(value, "default"));
        spec.flag = io::as_bool(member(value, "flag"), "cli.flag");
        spec.required = io::as_bool(member(value, "required"), "cli.required");
        spec.positional = io::as_bool(member(value, "positional"), "cli.positional");
        result.push_back(std::move(spec));
    }
    return result;
}
Arguments parse_arguments(std::string_view command, std::span<const std::string_view> input) {
    const auto specs = option_specs(command);
    Arguments args; args.command = command;
    for (const auto& spec : specs) if (spec.default_value) args.set(spec.dest, *spec.default_value);
    bool options = true;
    for (std::size_t i = 0; i < input.size(); ++i) {
        const auto token = input[i];
        if (options && (token == "--help" || token == "-h")) { args.help = true; return args; }
        if (options && token == "--") { options = false; continue; }
        if (token.find('\0') != std::string_view::npos) throw ArgumentError("argument contains NUL");
        const OptionSpec* chosen = nullptr;
        std::optional<std::string> attached;
        if (options && token.starts_with('-') && token != "-") {
            const auto equals = token.find('='); const auto name = token.substr(0, equals);
            for (const auto& spec : specs) if (!spec.positional)
                for (const auto& flag : spec.names) if (flag == name) chosen = &spec;
            if (!chosen && name.starts_with("--")) {
                for (const auto& spec : specs) if (!spec.positional)
                    for (const auto& flag : spec.names) if (flag.starts_with(name)) {
                        if (chosen && chosen != &spec) throw ArgumentError("ambiguous option: " + std::string(name));
                        chosen = &spec;
                    }
            }
            if (equals != std::string_view::npos) attached = std::string(token.substr(equals + 1));
            if (!chosen && token.size() > 2 && token[1] != '-') {
                for (const auto& spec : specs) if (!spec.flag && !spec.positional)
                    for (const auto& flag : spec.names) if (flag.size() == 2 && token.starts_with(flag)) {
                        chosen = &spec; attached = std::string(token.substr(2));
                    }
            }
            if (!chosen) throw ArgumentError("unrecognized arguments: " + std::string(token));
            if (chosen->flag) {
                if (attached) throw ArgumentError("argument " + std::string(name) + ": ignored explicit argument");
                args.set(chosen->dest, "true", true); continue;
            }
            if (!attached) {
                if (++i >= input.size()) throw ArgumentError("argument " + std::string(name) + ": expected one argument");
                attached = input[i];
                if (attached->starts_with("--")||*attached=="-h") throw ArgumentError("argument " + std::string(name) + ": expected one argument");
                for(const auto& spec:specs)if(!spec.positional)
                    for(const auto& flag:spec.names)if(*attached==flag)
                        throw ArgumentError("argument " + std::string(name) + ": expected one argument");
            }
        } else {
            for (const auto& spec : specs) if (spec.positional && !args.present(spec.dest)) { chosen = &spec; break; }
            if (!chosen) throw ArgumentError("unrecognized arguments: " + std::string(token));
            attached = token;
        }
        if ((chosen->dest == "spatial" || chosen->dest == "terrain3d" || chosen->dest == "threads") && args.present(chosen->dest))
            throw ArgumentError("argument " + chosen->names.front() + ": may only be supplied once");
        if(chosen->type=="int")*attached=integer_token(*attached);
        validate_token(*chosen, *attached); args.set(chosen->dest, std::move(*attached), true);
    }
    for (const auto& spec : specs) if (spec.required && !args.present(spec.dest))
        throw ArgumentError("the following arguments are required: " + spec.names.front());
    return args;
}
std::string help_text(std::string_view command) {
    const auto contracts = io::parse_json(kArgumentContracts);
    auto text = scalar(member(member(contracts, command), "help"));
    if (text.empty()) text = "usage: " + std::string(command) + " [-h]\n\nNative bicycle simulator.\n";
    if (command == "bike-ride" || command == "bike-research" || command == "bike-settle")
        text += "\nSpatial options:\n  --spatial {planar,3d}  override the TOML spatial mode (default: preserve TOML)\n"
                "  --terrain3d PATH      .terrain3d asset or .toml genspec; requires 3D\n"
                "  --threads N           spatial tyre threads, 1..64 (default: 1)\n"
                "  Recording requires --record; --out only selects its destination.\n";
    if (command == "bike-replay")
        text += "\n  --checked             explicitly select checked recalculation (the default)\n"
                "  --view                inspect recorded frames without recalculation\n";
    return text;
}
std::vector<std::string> command_names() {
    return {"bike-sim", "bike-playground", "bike-export", "bike-ride", "bike-settle", "bike-research", "bike-batch", "bike-replay", "bike-recording"};
}
} // namespace bike::cli
