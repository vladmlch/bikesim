#include "hud.hpp"
#include "../io/json.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <iomanip>
#include <locale>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <variant>
#include <vector>

namespace bike::viewer {
namespace {
const runtime::Wire* child(const runtime::WireObject& object, std::string_view name) {
    for (const auto& [key, value] : object) if (key == name) return &value;
    return nullptr;
}
const runtime::Wire* child(const runtime::Wire* object, std::string_view name) {
    if (!object) return nullptr;
    const auto* values = std::get_if<runtime::WireObject>(&object->value);
    return values ? child(*values, name) : nullptr;
}
const runtime::Wire* path(const runtime::Wire* value, std::initializer_list<std::string_view> keys) {
    for (const auto key : keys) value = child(value, key);
    return value;
}
std::optional<double> number(const runtime::Wire* value) {
    if (!value) return {};
    if (const auto* v = std::get_if<double>(&value->value)) return *v;
    if (const auto* v = std::get_if<std::int64_t>(&value->value)) return static_cast<double>(*v);
    return {};
}
std::string text(const runtime::Wire* value, std::string fallback = {}) {
    if (value) if (const auto* v = std::get_if<std::string>(&value->value)) return *v;
    return fallback;
}
bool truth(const runtime::Wire* value, bool fallback = false) {
    if (value) if (const auto* v = std::get_if<bool>(&value->value)) return *v;
    return fallback;
}
std::string f(double value, int width = 0, int precision = 1, bool sign = false) {
    std::ostringstream out; out.imbue(std::locale::classic());
    if (sign) out << std::showpos;
    out << std::fixed << std::setprecision(precision) << std::setw(width) << value;
    return out.str();
}
std::string optional_number(std::optional<double> value, int width, int precision, bool sign = false) {
    return value ? f(*value, width, precision, sign) : std::string(static_cast<std::size_t>(width), '-');
}
std::size_t display_columns(std::string_view text) {
    // HUD symbols are single-column arrows, degree signs and Greek letters.
    // UTF-8 continuation bytes must not widen a terminal legend column.
    std::size_t result = 0;
    for (const char raw : text) if ((static_cast<unsigned char>(raw) & 0xc0U) != 0x80U) ++result;
    return result;
}
std::string pad(std::string value, std::size_t width) { const auto columns=display_columns(value);if(columns<width)value.append(width-columns,' ');return value; }
}
std::vector<HudColumn> physical_columns(const runtime::RenderFrame& frame, std::optional<double> rtf) {
    const auto& state = frame.state;
    const auto& p = frame.presentation;
    const std::string mode = text(child(state.view, "drive_mode"), "physical");
    const auto rate = rtf ? f(*rtf, 0, 2) + "x" : "----x";
    if (!state.latest_sample) return {{.legend="PHYS", .text="[PHYS|" + mode + "]"}, {.legend="", .text=" initial condition RTF=" + rate}};
    // Samples own their channel payload. Formatting never reads model/data.
    const runtime::Wire channels(state.latest_sample->channels);
    const auto* drive = child(&channels, "drive");
    const auto* rider = child(&channels, "rider");
    const auto* tires = child(&channels, "tires");
    const auto n = [&](std::string_view name) { return number(child(drive, name)).value_or(0.); };
    const double front_angle = std::fmod(std::fmod(n("crank_phase_rad") * (180. / std::numbers::pi), 360.) + 360., 360.);
    std::vector<HudColumn> out;
    const auto add = [&](std::string legend, std::string value) { out.push_back({.legend=std::move(legend), .text=out.empty() || out.back().text == " | " ? std::move(value) : " " + value}); };
    const auto sep = [&] { out.push_back({.legend="", .text=" | "}); };
    add("PHYS", "[PHYS|" + mode + "]");
    add("time", "t=" + f(state.latest_sample->time_s, 7, 2) + "s");
    add("rtf", "RTF=" + rate);
    double speed = p.speed_mps;
    if (p.root_x_dofadr >= 0 && static_cast<std::size_t>(p.root_x_dofadr) < state.latest_sample->qvel.size())
        speed = state.latest_sample->qvel[static_cast<std::size_t>(p.root_x_dofadr)];
    add("speed", "v=" + f(speed * 3.6, 6, 2, true) + "km/h");
    add("Fn f/r", "Fn=" + f(number(path(tires,{"front","normal_load_n"})).value_or(0.),4,0) + "/" +
        f(number(path(tires,{"rear","normal_load_n"})).value_or(0.),4,0) + "N");
    sep();
    add("crank f/r", "crk F@" + f(front_angle,3,0) + "deg R@" + f(std::fmod(front_angle + 180.,360.),3,0) + "deg");
    add("pedals f/r", "ped=" + optional_number(number(path(rider,{"front_pedal","vertical_force_on_rider_n"})),4,0,true) + "/" +
        optional_number(number(path(rider,{"rear_pedal","vertical_force_on_rider_n"})),4,0,true) + "N");
    add("cadence", "cad=" + f(n("cadence_rpm"),5,1,true) + "rpm");
    const auto front_teeth = number(child(drive,"gear_front_teeth")), rear_teeth = number(child(drive,"gear_rear_teeth"));
    add("gear", "gear=" + (front_teeth && rear_teeth && *front_teeth != 0. && *rear_teeth != 0. ? f(*front_teeth,2,0) + "/" + f(*rear_teeth,2,0) : "--/--") + "T");
    add("human", "hum=" + f(n("human_sensor_nm"),5,1) + "Nm");
    add("motor", "mot=" + f(n("motor_torque_nm"),5,1) + "Nm");
    sep();
    std::string bar = "bar=";
    constexpr std::array<std::string_view,8> arrows{"→","↘","↓","↙","←","↖","↑","↗"};
    for (const auto side : {"left", "right"}) {
        if (side == std::string_view("right")) bar += ' ';
        bar += side == std::string_view("left") ? 'L' : 'R';
        const auto* force = path(rider,{std::string("grip_") + side,"force_on_bike_n"});
        const auto* components = force ? std::get_if<runtime::WireArray>(&force->value) : nullptr;
        if (!components || components->size() != 3) { bar += "-----N"; continue; }
        const double x = number(&(*components)[0]).value_or(0.), z = number(&(*components)[2]).value_or(0.);
        if (!std::isfinite(x) || !std::isfinite(z)) { bar += "-----N"; continue; }
        const auto sector = static_cast<long>(std::nearbyint(std::atan2(-z,x) * (180. / std::numbers::pi) / 45.));
        bar += arrows[static_cast<std::size_t>((sector % 8 + 8) % 8)];
        bar += f(std::hypot(x,z),3,0) + "N";
    }
    add("bar l/r",bar);
    add("arm", "arm=sh " + optional_number(number(path(&channels,{"rider_joint_torques","rider_shoulder_left"})),4,0,true) + "/el " +
        optional_number(number(path(&channels,{"rider_joint_torques","rider_elbow_left"})),4,0,true) + " Nm");
    add("saddle", "saddle=" + optional_number(number(path(rider,{"saddle","normal_load_n"})),4,0) + "N");
    const auto lean_limit=number(child(&channels,"rider_lean_limit_rad"));
    add("lean", "lean=" + f(p.torso_pitch_deg - p.pitch_rad*(180./std::numbers::pi),5,1,true) + "deg/lim" +
        (lean_limit ? f(lean_limit.value()*(180./std::numbers::pi),0,0) : "--") + "°");
    const auto* balance = child(&channels,"rider_balance");
    std::string status = "---";
    if (balance && std::holds_alternative<runtime::WireObject>(balance->value)) {
        if (truth(child(balance,"balance_lost"))) { const auto position=number(child(balance,"balance_lost_at_m")); status=position?"LOST@x="+f(*position,0,0)+" m":"LOST"; }
        else { const double low=number(child(balance,"low_speed_s")).value_or(0.); status=low>0.?"low "+f(low,0,1)+"s":"ok"; }
    }
    add("BAL","BAL="+pad(status,14)); sep();
    add("slip f/r","κ="+optional_number(number(path(tires,{"front","slip_ratio"})),5,2,true)+"/"+
        optional_number(number(path(tires,{"rear","slip_ratio"})),5,2,true));
    if (p.spatial) {
        const auto& spatial = *p.spatial;
        sep();
        add("roll", "roll=" + (spatial.attitude_valid && spatial.roll_rad ? f(*spatial.roll_rad * (180. / std::numbers::pi),0,2,true) + "deg" : "N/A"));
        add("steer", "steer=" + f(spatial.steer_angle_rad * (180. / std::numbers::pi),0,2,true) + "deg/" + f(spatial.steer_torque_nm,0,2,true) + "Nm");
        add("lateral slip F/R", "lat=" + (spatial.contact_valid[0] ? f(spatial.lateral_slip_mps[0],0,3,true) : "N/A") + "/" +
            (spatial.contact_valid[1] ? f(spatial.lateral_slip_mps[1],0,3,true) : "N/A") + "m/s");
        add("contact F/R", std::string("contact=") + (spatial.contact_valid[0] ? "valid" : "—") + "/" + (spatial.contact_valid[1] ? "valid" : "—"));
        if (spatial.hold_active) add("hold", "HOLD");
    }
    auto coast=text(child(drive,"coasting_reason")); if(coast.empty()&&!truth(child(drive,"freehub_engaged"),true))coast="freehub";
    add("coast","coast="+pad(coast.empty()?"-":coast,12));sep();add("shaft W","motor="+f(n("motor_shaft_power_w"),5,0,true)+"W");
    return out;
}
std::string hud_legend(const std::vector<HudColumn>& columns) {
    std::string out;
    for (const auto& column : columns) {
        const auto leading=column.text.find_first_not_of(' ');
        const auto spaces=leading==std::string::npos?column.text.size():leading;
        out.append(spaces,' '); out+=pad(column.legend,display_columns(column.text)-spaces);
    }
    return out;
}
std::string hud_line(const runtime::RenderFrame& frame,const InputState& input,std::optional<double> achieved,Mode mode) {
    std::string out;
    if(mode==Mode::physical||mode==Mode::research||mode==Mode::checked_replay)
        for(const auto& column:physical_columns(frame,achieved))out+=column.text;
    else {const auto& p=frame.presentation;out=std::string(mode==Mode::inspection?"[PLAYBACK] t=":"[RIDE] t=")+f(frame.state.time_s,0,2)+"s x="+f(p.position_m,0,2)+"m v="+f(p.speed_mps*3.6,0,2)+
        "km/h | Fork:"+f(p.fork_travel_mm)+"mm Shock:"+f(p.shock_stroke_mm)+"mm | Pitch:"+f(p.pitch_rad*(180./std::numbers::pi),0,2,true)+
        "deg "+(p.pitch_rad>0.?"nose-dn":p.pitch_rad<0.?"nose-up":"level");}
    out+=" | Brake:"+std::string(input.braking?"ON ":"off ")+f(input.brake_strength*100.,0,0)+"%";
    if(input.paused)out+=" | PAUSED";
    if(frame.state.outcome)out+=" | "+*frame.state.outcome;
    if(frame.state.first_failure)out+=" | INVALID: first model failure retained";
    return out;
}
std::string overlay_text(const runtime::RenderFrame& frame,const InputState& input,std::optional<double> achieved,Mode mode) {
    const auto& p=frame.presentation;
    std::string spatial_line;
    if (p.spatial) {
        const auto& s = *p.spatial;
        spatial_line = "roll " + (s.attitude_valid && s.roll_rad ? f(*s.roll_rad * (180. / std::numbers::pi),0,2,true) + " deg" : "N/A") +
            "  steer " + f(s.steer_angle_rad * (180. / std::numbers::pi),0,2,true) + " deg / " + f(s.steer_torque_nm,0,2,true) + " Nm\n" +
            "lateral slip F/R " + (s.contact_valid[0] ? f(s.lateral_slip_mps[0],0,3,true) : "N/A") + "/" +
            (s.contact_valid[1] ? f(s.lateral_slip_mps[1],0,3,true) : "N/A") + " m/s  contact " +
            (s.contact_valid[0] ? "valid" : "—") + "/" + (s.contact_valid[1] ? "valid" : "—") + (s.hold_active ? "  HOLD\n" : "\n");
    }
    return "t="+f(frame.state.time_s,0,2)+" s  x="+f(p.position_m,0,1)+" m  speed="+f(p.speed_mps*3.6,0,1)+" km/h\n"+spatial_line+
        "Fork "+f(p.fork_travel_mm)+" mm  Shock "+f(p.shock_stroke_mm)+" mm  Load F/R "+f(p.front_load_n,0,0)+"/"+f(p.rear_load_n,0,0)+" N\n"+
        "playback="+std::to_string(input.scale)+"x  achieved="+(achieved?f(*achieved,0,2):"--")+"x"+
        (input.paused?"  PAUSED":"")+(input.braking?"  BRAKING":"")+"\n"+
        (frame.state.first_failure?"INVALID: model evidence failed\n":"")+
        (frame.state.outcome?*frame.state.outcome+" (R reset / Q close)\n":"")+
        help_text(mode).substr(0,help_text(mode).find('\n'));
}
std::string playground_hud(const io::Value& telemetry,bool sweep) {
    const auto& values=io::as_object(telemetry,"stand telemetry");
    const auto get=[&](std::string_view name)->const io::Value&{const auto* value=io::find_member(values,name);if(!value)throw std::invalid_argument("stand HUD is missing "+std::string(name));return *value;};
    const auto n=[&](std::string_view name){return io::as_double(get(name),name);};
    const auto b=[&](std::string_view name){return io::as_bool(get(name),name);};
    const auto i=[&](std::string_view name){return std::to_string(io::as_int64(get(name),name));};
    return std::string("[STAND")+(sweep?" [AUTO-SWEEP ON]":"")+"|Rider:"+(b("rider_active")?"ON":"OFF")+"|"+std::string(io::as_string(get("damper_preset"),"damper_preset"))+"] "+
        "Fork:"+f(n("fork_travel_mm"),5)+"mm (Air:"+f(n("fork_air_psi"),4)+"PSI, "+i("fork_tokens")+"tok, Fd:"+f(n("fork_damper_n"),4,0,true)+"N) | "+
        "Shock:"+f(n("shock_stroke_mm"),4)+"/65mm ("+f(n("shock_compression_pct"),4)+"%, Fd:"+f(n("shock_damper_n"),4,0,true)+"N) | "+
        "ZEB:[H"+i("fork_hsc")+" L"+i("fork_lsc")+" R"+i("fork_reb")+"] Shk:[H"+i("shock_hsc")+" L"+i("shock_lsc")+" R"+i("shock_reb")+" B"+i("shock_hbo")+"|"+(b("shock_lockout")?"FIRM":"OPEN")+"]";
}
} // namespace bike::viewer
