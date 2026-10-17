#include "terrain/trackfile.hpp"
#include "terrain/spec_io.hpp"
#include "configuration/load.hpp"
#include "configuration/wire_decode.hpp"
#include "platform/files.hpp"
#include <toml++/toml.hpp>
#include <algorithm>
#include <cerrno>
#include <fstream>
#include <limits>
#include <system_error>
#include <type_traits>
#include <unistd.h>

namespace bike::terrain {
TrackFileError::~TrackFileError() = default;
namespace {
using bike::io::Value;
Value ordered_node(const toml::node& node);
// NOLINTNEXTLINE(misc-no-recursion) finite TOML document tree, matching T13
Value ordered_table(const toml::table& table) {
    struct Entry {
        std::string key;
        const toml::node* node;
        toml::source_position position;
    };
    std::vector<Entry> entries;
    entries.reserve(table.size());
    for (const auto& [key,node] : table)
        entries.push_back({std::string(key.str()),&node,key.source().begin});
    std::stable_sort(entries.begin(),entries.end(),[](const Entry& a,const Entry& b) {
        if (a.position.line != b.position.line) return a.position.line < b.position.line;
        return a.position.column < b.position.column;
    });
    Value::Object out;
    out.reserve(entries.size());
    for (const auto& entry : entries)
        out.emplace_back(entry.key,ordered_node(*entry.node));
    return {std::move(out)};
}
// NOLINTNEXTLINE(misc-no-recursion) finite TOML document tree, matching T13
Value ordered_node(const toml::node& node) {
    switch (node.type()) {
    case toml::node_type::table: return ordered_table(*node.as_table());
    case toml::node_type::array: {
        Value::Array out;
        out.reserve(node.as_array()->size());
        for (const auto& child : *node.as_array()) out.push_back(ordered_node(child));
        return {std::move(out)};
    }
    case toml::node_type::string: return {node.as_string()->get()};
    case toml::node_type::integer: return detail::integer(node.as_integer()->get());
    case toml::node_type::floating_point: return detail::number(node.as_floating_point()->get());
    case toml::node_type::boolean: return {node.as_boolean()->get()};
    case toml::node_type::date: return {bike::io::Tagged{"toml.date",{}}};
    case toml::node_type::time: return {bike::io::Tagged{"toml.time",{}}};
    case toml::node_type::date_time: return {bike::io::Tagged{"toml.date_time",{}}};
    case toml::node_type::none: break;
    }
    return {};
}
Value ordered_toml(std::string_view source,std::string_view origin) {
    try {
        return ordered_table(toml::parse(source,origin));
    } catch (const toml::parse_error& error) {
        throw TrackFileError(std::string(origin)+": "+std::string(error.description()));
    }
}
bool key_character(char c){return(c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-';}
std::size_t quoted_end(std::string_view text,std::size_t start){
    const char quote=text[start];const bool triple=start+2<text.size()&&text[start+1]==quote&&text[start+2]==quote;
    std::size_t i=start+(triple?3U:1U);
    while(i<text.size()){
        if(quote=='"'&&text[i]=='\\'){i+=std::min<std::size_t>(2,text.size()-i);continue;}
        if(text[i]==quote){
            if(!triple)return i+1;
            std::size_t end=i;while(end<text.size()&&text[end]==quote)++end;
            if(end-i>=3)return end;i=end;
        }else ++i;
    }return text.size(); // The real TOML parser reports an unterminated literal.
}
std::optional<std::string> integer_seed_token(std::string_view token){
    if(token.empty())return std::nullopt;bool negative=false,sign=false;
    if(token.front()=='+'||token.front()=='-'){sign=true;negative=token.front()=='-';token.remove_prefix(1);}
    if(token.empty())return std::nullopt;
    unsigned int radix=10;
    if(token.size()>2&&token.front()=='0'){
        if(token[1]=='x')radix=16;else if(token[1]=='o')radix=8;else if(token[1]=='b')radix=2;
        if(radix!=10){if(sign)return std::nullopt;token.remove_prefix(2);}
    }
    std::string digits;bool previous_digit=false;
    for(char c:token){
        if(c=='_'){if(!previous_digit)return std::nullopt;previous_digit=false;continue;}
        const unsigned int d=c>='0'&&c<='9'?static_cast<unsigned int>(c-'0'):c>='a'&&c<='f'?static_cast<unsigned int>(c-'a')+10U:c>='A'&&c<='F'?static_cast<unsigned int>(c-'A')+10U:radix;
        if(d>=radix)return std::nullopt;digits.push_back(c);previous_digit=true;
    }
    if(!previous_digit||digits.empty()||(radix==10&&digits.size()>1&&digits.front()=='0'))return std::nullopt;
    std::string decimal="0";
    if(radix==10)decimal=digits;
    else for(char c:digits){
        unsigned int carry=c>='0'&&c<='9'?static_cast<unsigned int>(c-'0'):c>='a'&&c<='f'?static_cast<unsigned int>(c-'a')+10U:static_cast<unsigned int>(c-'A')+10U;
        for(std::size_t i=decimal.size();i>0;--i){const unsigned int digit=static_cast<unsigned int>(decimal[i-1]-'0')*radix+carry;decimal[i-1]=static_cast<char>('0'+digit%10U);carry=digit/10U;}
        while(carry!=0){decimal.insert(decimal.begin(),static_cast<char>('0'+carry%10U));carry/=10U;}
    }
    const auto first=decimal.find_first_not_of('0');decimal=first==std::string::npos?"0":decimal.substr(first);
    return negative&&decimal!="0"?"-"+decimal:decimal;
}
std::string read_track_text(const std::filesystem::path& path){
    auto handle=bike::platform::open_regular(path);std::string out;std::array<char,4096> buffer{};
    for(;;){
        const auto n=::read(handle.get(),buffer.data(),buffer.size());
        if(n<0){if(errno==EINTR)continue;throw std::system_error(errno,std::generic_category(),"read track");}
        if(n==0)break;const auto count=static_cast<std::size_t>(n);
        if(count>bike::io::kMaxJsonBytes-out.size())throw TrackFileError("track document exceeds the 512 MiB input limit");
        const auto chunk=std::span(buffer).first(count);out.append(chunk.begin(),chunk.end());
    }handle.close();return out;
}
// NOLINTNEXTLINE(misc-no-recursion) source description=str(value) over the owned tree
std::string scalar_string(const Value& value,bool nested=false){
    if(const auto* s=std::get_if<std::string>(&value.data))return nested?bike::configuration::detail::py_repr_str(*s):*s;
    if(const auto* n=std::get_if<bike::io::Number>(&value.data))return n->lexeme;
    if(const auto* b=std::get_if<bool>(&value.data))return *b?"True":"False";
    if(bike::io::is_null(value))return "None";
    if(const auto* array=std::get_if<Value::Array>(&value.data)){
        std::string out="[";for(std::size_t i=0;i<array->size();++i){if(i!=0)out+=", ";out+=scalar_string((*array)[i],true);}return out+"]";
    }
    if(const auto* object=std::get_if<Value::Object>(&value.data)){
        std::string out="{";for(std::size_t i=0;i<object->size();++i){if(i!=0)out+=", ";out+=bike::configuration::detail::py_repr_str((*object)[i].first)+": "+scalar_string((*object)[i].second,true);}return out+"}";
    }
    return bike::io::canonical_json(value);
}
double float_value(const Value& value,std::string_view path){
    if(const auto* s=std::get_if<std::string>(&value.data)){
        bool okay=false;const double result=bike::io::detail::float_from_lexeme(*s,okay);
        if(!okay||!std::isfinite(result))throw TrackFileError(std::string(path)+": expected finite number");return result;
    }
    if(const auto* b=std::get_if<bool>(&value.data))return *b?1.:0.;
    return bike::io::as_double(value,path);
}
std::string seed_of(const Value& value,std::string_view where){
    const auto& number=bike::io::as_number(value,where);
    if(!bike::io::is_integer_lexeme(number.lexeme))throw TrackFileError(std::string(where)+": seed must be an integer");
    return detail::seed_plus(number.lexeme,0);
}
Value scaled(const Value& value){
    if(const auto* values=std::get_if<Value::Array>(&value.data)){
        Value::Array out;for(const auto& item:*values)out.push_back(detail::number(float_value(item,"millimetre value")*1e-3));return {std::move(out)};
    }return detail::number(float_value(value,"millimetre value")*1e-3);
}
Obstacle resolve(const Value& entry,std::size_t index){
    const auto where="obstacles["+std::to_string(index)+"]";
    const auto* v=detail::member(entry,"type");if(!v||!std::holds_alternative<std::string>(v->data))throw TrackFileError(where+": missing 'type'");
    const auto& type=bike::io::as_string(*v,where);
    if(type=="pothole"){
        const auto* edge=detail::member(entry,"edge");const auto key=edge?bike::io::as_string(*edge,where):"sharp";
        if(key=="sharp")return {Pothole{}};if(key=="sloped")return {SlopedPothole{}};if(key=="bowl")return {BowlPothole{}};
        throw TrackFileError(where+": unknown pothole edge '"+key+"'; allowed: ['sharp', 'sloped', 'bowl']");
    }
    if(type=="bump"){
        const auto* shape=detail::member(entry,"shape");const auto key=shape?bike::io::as_string(*shape,where):"cosine";
        if(key=="cosine")return {Bump{}};if(key=="trapezoid")return {TrapezoidBump{}};
        throw TrackFileError(where+": unknown bump shape '"+key+"'; allowed: ['cosine', 'trapezoid']");
    }
    if(type=="square_edge")return {SquareEdge{}};if(type=="washboard")return {Washboard{}};if(type=="g_out")return {GOut{}};
    if(type=="drop")return {Drop{}};if(type=="kicker")return {Kicker{}};if(type=="roots")return {Roots{}};
    if(type=="rock_garden")return {RockGarden{}};if(type=="roughness")return {RoadRoughness{}};
    throw TrackFileError(where+": unknown type '"+type+"'");
}
std::string_view length_field(const Obstacle& obstacle){
    return std::visit([](const auto& s)->std::string_view{
        using T=std::decay_t<decltype(s)>;
        if constexpr(std::is_same_v<T,Pothole>||std::is_same_v<T,SlopedPothole>||std::is_same_v<T,BowlPothole>)return "hole_length_m";
        else if constexpr(std::is_same_v<T,Bump>)return "bump_length_m";
        else if constexpr(std::is_same_v<T,SquareEdge>)return "ledge_length_m";
        else if constexpr(std::is_same_v<T,GOut>)return "dip_length_m";
        else if constexpr(std::is_same_v<T,Roots>||std::is_same_v<T,RockGarden>||std::is_same_v<T,RoadRoughness>)return "section_length_m";
        else return "";
    },obstacle.shape);
}
Value obstacle_fields(const Obstacle& obstacle,bool identity){
    Value::Object out;
    if(identity)out.emplace_back("class",Value{std::string(obstacle.kind())});
    else{
        const auto index=obstacle.shape.index();
        constexpr std::array<std::string_view,13> tags{"square_edge","pothole","washboard","g_out","drop","kicker","roots","rock_garden","bump","bump","pothole","pothole","roughness"};
        if(index>=tags.size())throw TrackFileError("obstacle type SteppedClimb has no track-file representation");
        out.emplace_back("type",Value{std::string(tags[index])});
        if(index==1||index==10||index==11)out.emplace_back("edge",Value{std::string(index==1?"sharp":index==10?"sloped":"bowl")});
        if(index==8||index==9)out.emplace_back("shape",Value{std::string(index==8?"cosine":"trapezoid")});
    }
    std::visit([&](const auto& s){
        using T=std::decay_t<decltype(s)>;
#define PUT(name) out.emplace_back(#name,detail::field_value(s.name))
        PUT(start_m);
        if constexpr(std::is_same_v<T,SquareEdge>){PUT(height_m);PUT(ledge_length_m);}
        else if constexpr(std::is_same_v<T,Pothole>||std::is_same_v<T,BowlPothole>){PUT(depth_m);PUT(hole_length_m);}
        else if constexpr(std::is_same_v<T,Washboard>){PUT(amplitude_m);PUT(wavelength_m);PUT(n_waves);}
        else if constexpr(std::is_same_v<T,GOut>){PUT(depth_m);PUT(dip_length_m);}
        else if constexpr(std::is_same_v<T,Drop>){PUT(height_m);PUT(ramp_m);}
        else if constexpr(std::is_same_v<T,Kicker>){PUT(height_m);PUT(ramp_m);PUT(gap_m);PUT(landing_angle_deg);PUT(landing_m);}
        else if constexpr(std::is_same_v<T,Roots>){PUT(n_bumps);PUT(height_m);PUT(width_m);PUT(section_length_m);PUT(jitter);out.emplace_back("seed",detail::seed_value(s.seed));}
        else if constexpr(std::is_same_v<T,RockGarden>||std::is_same_v<T,RoadRoughness>){PUT(section_length_m);PUT(amplitude_m);PUT(correlation_length_m);out.emplace_back("seed",detail::seed_value(s.seed));}
        else if constexpr(std::is_same_v<T,Bump>){PUT(height_m);PUT(bump_length_m);}
        else if constexpr(std::is_same_v<T,TrapezoidBump>){PUT(height_m);PUT(ramp_m);PUT(plateau_m);}
        else if constexpr(std::is_same_v<T,SlopedPothole>){PUT(depth_m);PUT(hole_length_m);PUT(edge_m);}
        else if constexpr(std::is_same_v<T,SteppedClimb>){Value::Array steps;for(const auto& row:s.steps)steps.push_back(Value{Value::Array{detail::number(row[0]),detail::number(row[1])}});out.emplace_back("steps",Value{std::move(steps)});PUT(transition_m);}
#undef PUT
    },obstacle.shape);
    if(!identity)for(auto& [key,value]:out)if(std::ranges::find(obstacle.integer_scalar_fields,key)!=obstacle.integer_scalar_fields.end()){
        const double v=bike::io::as_double(value,key);if(std::trunc(v)==v)value={bike::io::Number{detail::gd::py_fixed(v,0)}};
    }
    return {std::move(out)};
}
Value track_value(const TrackSpec& track,bool identity){
    track.validate();Value::Object out{{"name",Value{track.name}},{"length_m",detail::number(track.length_m)}};
    if(!track.description.empty())out.emplace_back("description",Value{track.description});out.emplace_back("surface",Value{track.surface});
    if(track.grade_profile){Value::Array knots;for(const auto& p:track.grade_profile->knots)knots.push_back(Value{Value::Array{detail::number(p[0]),detail::number(p[1])}});out.emplace_back("grade_profile",Value{Value::Object{{"knots",Value{std::move(knots)}}}});}
    if(!track.surface_sections.empty()){
        const auto mapping=track.surface_map();
        Value::Array sections;for(const auto& s:mapping.sections())sections.push_back(Value{Value::Object{{"start_m",detail::number(s.start_m)},{"end_m",detail::number(s.end_m)},{"surface",Value{s.surface}}}});
        out.emplace_back("surface_sections",Value{std::move(sections)});
    }
    Value::Array obstacles;for(const auto& o:track.sorted_obstacles())obstacles.push_back(obstacle_fields(o,identity));out.emplace_back("obstacles",Value{std::move(obstacles)});
    if(identity)out.emplace_back("native_track_identity_schema",detail::integer(1));
    return {std::move(out)};
}
std::string toml_value(const Value& v){
    if(const auto* b=std::get_if<bool>(&v.data))return *b?"true":"false";
    if(const auto* n=std::get_if<bike::io::Number>(&v.data))return n->lexeme;
    if(const auto* s=std::get_if<std::string>(&v.data)){
        std::string out="\"";for(char c:*s){if(c=='\\'||c=='"')out.push_back('\\');out.push_back(c);}return out+'"';
    }
    if(const auto* a=std::get_if<Value::Array>(&v.data)){std::string out="[";for(std::size_t i=0;i<a->size();++i){if(i!=0)out+=", ";out+=toml_value((*a)[i]);}return out+"]";}
    if(const auto* o=std::get_if<Value::Object>(&v.data)){std::string out="{ ";for(std::size_t i=0;i<o->size();++i){if(i!=0)out+=", ";out+=(*o)[i].first+" = "+toml_value((*o)[i].second);}return out+" }";}
    throw TrackFileError("cannot write this value to TOML");
}
}
Value parse_track_toml(std::string_view text,std::string_view origin){
    if(text.size()>bike::io::kMaxJsonBytes)throw TrackFileError("track document exceeds the 512 MiB input limit");
    struct SeedToken{std::string marker,value;};std::vector<SeedToken> seeds;std::string rewritten;rewritten.reserve(text.size());std::size_t copied=0;
    for(std::size_t i=0;i<text.size();){
        if(text[i]=='#'){const auto next=text.find('\n',i);i=next==std::string_view::npos?text.size():next+1;continue;}
        std::string key;std::size_t key_end=i;
        if(text[i]=='\''||text[i]=='"'){
            key_end=quoted_end(text,i);const bool triple=i+2<text.size()&&text[i+1]==text[i]&&text[i+2]==text[i];
            std::size_t equals=key_end;while(equals<text.size()&&(text[equals]==' '||text[equals]=='\t'))++equals;
            if(!triple&&equals<text.size()&&text[equals]=='='){
                const auto decoded=bike::configuration::detail::parse_toml("key = "+std::string(text.substr(i,key_end-i)),origin);
                key=bike::io::as_string(detail::required(decoded,"key"),"track key");
            }
        }else if(key_character(text[i])){key_end=i+1;while(key_end<text.size()&&key_character(text[key_end]))++key_end;key=std::string(text.substr(i,key_end-i));}
        else{++i;continue;}
        i=key_end;if(key!="seed")continue;
        std::size_t value=i;while(value<text.size()&&(text[value]==' '||text[value]=='\t'))++value;
        if(value==text.size()||text[value]!='=')continue;++value;while(value<text.size()&&(text[value]==' '||text[value]=='\t'))++value;
        std::size_t end=value;while(end<text.size()&&text[end]!=' '&&text[end]!='\t'&&text[end]!='\r'&&text[end]!='\n'&&text[end]!=','&&text[end]!=']'&&text[end]!='}'&&text[end]!='#')++end;
        const auto integer=integer_seed_token(text.substr(value,end-value));if(!integer)continue;
        const auto marker=std::to_string(std::numeric_limits<std::int64_t>::min()+1+static_cast<std::int64_t>(seeds.size()));
        rewritten.append(text.substr(copied,value-copied));rewritten+=marker;copied=end;i=end;seeds.push_back({marker,*integer});
    }
    rewritten.append(text.substr(copied));auto tree=ordered_toml(rewritten,origin);
    // Every integer-valued seed was replaced, so original source integer
    // values cannot collide with markers. Strings remain strings. Restore
    // by unique marker identity, independent of TOML table traversal order.
    std::vector<bool> restored(seeds.size(),false);std::vector<Value*> pending{&tree};
    while(!pending.empty()){
        Value* current=pending.back();pending.pop_back();
        if(auto* object=std::get_if<Value::Object>(&current->data))for(auto& [key,child]:*object){
            if(key=="seed")if(auto* n=std::get_if<bike::io::Number>(&child.data))for(std::size_t j=0;j<seeds.size();++j)if(n->lexeme==seeds[j].marker){
                if(restored[j])throw TrackFileError("duplicate internal seed marker");n->lexeme=seeds[j].value;restored[j]=true;break;
            }
            pending.push_back(&child);
        }else if(auto* array=std::get_if<Value::Array>(&current->data))for(auto& child:*array)pending.push_back(&child);
    }
    if(std::ranges::any_of(restored,[](bool value){return !value;}))throw TrackFileError("seed token could not be resolved in the parsed track");
    return tree;
}
Obstacle obstacle_from_dict(const Value& entry,std::size_t index){
    auto result=resolve(entry,index);const auto type=bike::io::as_string(detail::required(entry,"type"),"obstacle.type");
    const auto where="obstacles["+std::to_string(index)+"] ("+type+")";std::vector<std::string> seen;bool has_start=false;
    for(const auto& [key,original]:bike::io::as_object(entry,where)){
        if(key=="type"||(type=="pothole"&&key=="edge")||(type=="bump"&&key=="shape"))continue;
        std::string target=key;Value value=original;
        if(key=="length_m"&&!length_field(result).empty())target=std::string(length_field(result));
        else if(key.ends_with("_mm")){target=key.substr(0,key.size()-3)+"_m";value=scaled(original);}
        if(std::ranges::find(seen,target)!=seen.end())throw TrackFileError(where+": '"+target+"' given twice (metres and millimetres?)");
        seen.push_back(target);bool known=false;
        std::visit([&](auto& s){
#define READ(name) if constexpr(requires{s.name;}){if(target==#name){detail::read_field(s.name,value,where+"."+target);known=true;}}
            READ(start_m) READ(height_m) READ(ledge_length_m) READ(depth_m) READ(hole_length_m)
            READ(amplitude_m) READ(wavelength_m) READ(n_waves) READ(dip_length_m) READ(ramp_m)
            READ(gap_m) READ(landing_angle_deg) READ(landing_m) READ(n_bumps) READ(width_m)
            READ(section_length_m) READ(jitter) READ(correlation_length_m) READ(bump_length_m)
            READ(plateau_m) READ(edge_m)
#undef READ
            if constexpr(requires{s.seed;})if(target=="seed"){s.seed=seed_of(value,where+".seed");known=true;}
        },result.shape);
        if(!known)throw TrackFileError(where+": unknown key '"+key+"'");
        if(target=="start_m")has_start=true;
        if(target!="seed"&&target!="n_bumps"&&target!="n_waves")if(const auto* n=std::get_if<bike::io::Number>(&value.data);n&&bike::io::is_integer_lexeme(n->lexeme))result.integer_scalar_fields.push_back(target);
    }
    if(!has_start)throw TrackFileError(where+": missing 'start_m'");return result;
}
RoadGeneratorSpec generator_from_dict(const Value& entry){
    RoadGeneratorSpec out;std::vector<std::string> seen;
    for(const auto& [key,raw]:bike::io::as_object(entry,"generator")){
        const std::string target=key.ends_with("_mm")?key.substr(0,key.size()-3)+"_m":key;
        const Value v=key.ends_with("_mm")?scaled(raw):raw;
        if(std::ranges::find(seen,target)!=seen.end())throw TrackFileError("generator: '"+target+"' given twice (metres and millimetres?)");seen.push_back(target);
        bool found=false;
#define SCALAR(name) if(target==#name){detail::read_field(out.name,v,"generator."+target);found=true;}
        SCALAR(runup_m) SCALAR(runout_m) SCALAR(min_gap_m) SCALAR(potholes_per_100m) SCALAR(pothole_edge_m)
        SCALAR(bumps_per_100m) SCALAR(bump_ramp_fraction) SCALAR(roughness_m) SCALAR(roughness_correlation_m)
#undef SCALAR
#define RANGE(name) if(target==#name){if(std::holds_alternative<Value::Array>(v.data))detail::read_field(out.name,v,target);else{const double x=float_value(v,target);out.name={x,x};}found=true;}
        RANGE(pothole_depth_m) RANGE(pothole_length_m) RANGE(bump_height_m) RANGE(bump_length_m)
#undef RANGE
        if(target=="seed"){out.seed=seed_of(v,"generator.seed");found=true;}
        if(target=="pothole_edge"||target=="bump_shape"){
            auto& weights=target=="pothole_edge"?out.pothole_edge:out.bump_shape;weights.clear();
            if(const auto* text=std::get_if<std::string>(&v.data))weights.emplace_back(*text,1.);
            else for(const auto& [shape,value]:bike::io::as_object(v,target))weights.emplace_back(shape,bike::io::as_double(value,target));found=true;
        }
        if(!found)throw TrackFileError("generator: unknown key '"+key+"'");
    }
    try{out.validate();}catch(const std::invalid_argument& e){throw TrackFileError("generator: "+std::string(e.what()));}return out;
}
std::vector<Obstacle> carve_roughness(std::vector<Obstacle> obstacles){
    std::vector<Obstacle> defects,roughness;for(const auto& o:obstacles)(std::holds_alternative<RoadRoughness>(o.shape)?roughness:defects).push_back(o);
    const auto order=[](const auto&a,const auto&b){return a.start_m()<b.start_m();};
    if(roughness.empty()||defects.empty()){std::stable_sort(obstacles.begin(),obstacles.end(),order);return obstacles;}
    std::stable_sort(defects.begin(),defects.end(),order);std::vector<Obstacle> carved;
    const auto integer_field=[](const Obstacle& o,std::string_view name){return std::ranges::find(o.integer_scalar_fields,name)!=o.integer_scalar_fields.end();};
    const auto integer_end=[&](const Obstacle& o){
        if(!integer_field(o,"start_m"))return false;
        return std::visit([&](const auto& shape){
            using T=std::decay_t<decltype(shape)>;
            if constexpr(std::is_same_v<T,Drop>)return integer_field(o,"ramp_m");
            else if constexpr(std::is_same_v<T,Kicker>)return integer_field(o,"ramp_m")&&integer_field(o,"gap_m")&&integer_field(o,"landing_m");
            else if constexpr(std::is_same_v<T,Washboard>)return integer_field(o,"wavelength_m");
            else return !length_field(o).empty()&&integer_field(o,length_field(o));
        },o.shape);
    };
    struct Piece{double a,b;std::string seed;bool integer_a,integer_b;};
    for(const auto& base:roughness){const auto& seg=std::get<RoadRoughness>(base.shape);std::vector<Piece> pieces{{base.start_m(),base.end_m(),seg.seed,integer_field(base,"start_m"),integer_end(base)}};
        for(const auto& d:defects){std::vector<Piece> next;for(const auto& p:pieces){
            if(d.end_m()<=p.a||d.start_m()>=p.b){next.push_back(p);continue;}
            if(d.start_m()>p.a)next.push_back({p.a,d.start_m(),p.seed,p.integer_a,integer_field(d,"start_m")});
            if(d.end_m()<p.b)next.push_back({d.end_m(),p.b,detail::seed_plus(p.seed,1000003),integer_end(d),p.integer_b});
        }pieces=std::move(next);}
        for(const auto& p:pieces)if(p.b-p.a>=.5){
            double length=rounded(p.b-p.a,6);bool integer_length=p.integer_a&&p.integer_b;
            while(p.a+length>p.b){length=rounded(length-1e-6,6);integer_length=false;}
            Obstacle part{RoadRoughness{p.a,length,seg.amplitude_m,seg.correlation_length_m,p.seed}};
            if(p.integer_a)part.integer_scalar_fields.push_back("start_m");
            if(integer_length)part.integer_scalar_fields.push_back("section_length_m");
            for(auto name:{"amplitude_m","correlation_length_m"})if(integer_field(base,name))part.integer_scalar_fields.emplace_back(name);
            carved.push_back(std::move(part));
        }
    }
    defects.insert(defects.end(),carved.begin(),carved.end());std::stable_sort(defects.begin(),defects.end(),order);return defects;
}
TrackSpec track_from_dict(const Value& data,std::optional<std::string> seed,std::optional<double> length){
    const auto* name=detail::member(data,"name");if(!name||!std::holds_alternative<std::string>(name->data)||bike::io::as_string(*name,"name").empty())throw TrackFileError("track file needs a non-empty 'name'");
    const auto* length_value=detail::member(data,"length_m");const double size=length?*length:length_value?float_value(*length_value,"length_m"):0.;
    if(!std::isfinite(size)||size<=0.)throw TrackFileError("track file needs a positive 'length_m'");
    std::vector<Obstacle> hand;
    if(const auto* entries=detail::member(data,"obstacles")){const auto& a=bike::io::as_array(*entries,"obstacles");for(std::size_t i=0;i<a.size();++i)hand.push_back(obstacle_from_dict(a[i],i));}
    hand=carve_roughness(std::move(hand));const auto* gen=detail::member(data,"generator");
    if(seed&&!gen)throw TrackFileError("a seed override was given but the track file has no [generator] block");
    if(gen){
        Value resolved{Value::Object(bike::io::as_object(*gen,"generator"))};
        if(seed){
            auto& fields=std::get<Value::Object>(resolved.data);
            const Value value=detail::seed_value(*seed);
            const auto found=std::ranges::find_if(fields,[](const auto& entry){return entry.first=="seed";});
            if(found==fields.end())fields.emplace_back("seed",value);else found->second=value;
        }
        auto spec=generator_from_dict(resolved);std::vector<Interval> reserved;for(const auto& o:hand)reserved.emplace_back(o.start_m(),o.end_m());
        try{auto generated=generate_road(spec,size,reserved);hand.insert(hand.end(),generated.begin(),generated.end());}catch(const std::invalid_argument& e){throw TrackFileError("generator: "+std::string(e.what()));}}
    TrackSpec track;track.name=bike::io::as_string(*name,"name");track.length_m=size;track.obstacles=std::move(hand);
    if(const auto* d=detail::member(data,"description"))track.description=scalar_string(*d);
    if(const auto* s=detail::member(data,"surface"))track.surface=bike::io::as_string(*s,"surface");else track.surface=gen?"asphalt":"hardpack";
    if(const auto* g=detail::member(data,"grade_profile");g&&!bike::io::is_null(*g)){
        const auto& table=bike::io::as_object(*g,"grade_profile");if(table.size()!=1||table.front().first!="knots")throw TrackFileError("grade_profile needs exactly a knots array");
        GradeProfile grade;for(const auto& row:bike::io::as_array(table.front().second,"grade_profile.knots")){const auto& pair=bike::io::as_array(row,"grade knot");if(pair.size()!=2)throw TrackFileError("grade knots must be station/grade pairs");grade.knots.push_back({bike::io::as_double(pair[0],"grade.station"),bike::io::as_double(pair[1],"grade.grade")});}
        grade.validate();track.grade_profile=std::move(grade);
    }
    if(const auto* values=detail::member(data,"surface_sections"))for(const auto& entry:bike::io::as_array(*values,"surface_sections")){
        if(bike::io::as_object(entry,"surface section").size()!=3)throw TrackFileError("surface section needs start_m, end_m and surface");
        SurfaceSection s{bike::io::as_double(detail::required(entry,"start_m"),"surface.start_m"),bike::io::as_double(detail::required(entry,"end_m"),"surface.end_m"),bike::io::as_string(detail::required(entry,"surface"),"surface.surface")};s.validate();track.surface_sections.push_back(std::move(s));
    }
    try{track.validate();}catch(const std::invalid_argument& e){throw TrackFileError(e.what());}return track;
}
TrackSpec load_track(const std::filesystem::path& path,std::optional<std::string> seed,std::optional<double> length){
    return track_from_dict(parse_track_toml(read_track_text(path),path.string()),std::move(seed),length);
}
Value obstacle_to_dict(const Obstacle& obstacle){return obstacle_fields(obstacle,false);}
Value track_to_dict(const TrackSpec& track){return track_value(track,false);}
Value identity_value(const TrackSpec& track){return track_value(track,true);}
std::string dump_track(const TrackSpec& track){
    const auto document=track_to_dict(track);std::string out="# Track file for bike-ride. Heights/depths in the classes' own units (metres);\n# `*_mm` keys are also accepted when editing by hand. Obstacles may not overlap.\n\n";
    for(const auto key:{"name","length_m","description","surface","grade_profile","surface_sections"})if(const auto* v=detail::member(document,key))out+=std::string(key)+" = "+toml_value(*v)+"\n";
    for(const auto& entry:bike::io::as_array(detail::required(document,"obstacles"),"obstacles")){out+="\n[[obstacles]]\n";for(const auto& [key,value]:bike::io::as_object(entry,"obstacle"))out+=key+" = "+toml_value(value)+"\n";}
    return out;
}
std::filesystem::path save_track(const TrackSpec& track,const std::filesystem::path& path){
    const auto text=dump_track(track);std::ofstream file;file.exceptions(std::ios::badbit|std::ios::failbit);file.open(path,std::ios::binary|std::ios::trunc);file.write(text.data(),static_cast<std::streamsize>(text.size()));file.close();return path;
}
} // namespace bike::terrain
