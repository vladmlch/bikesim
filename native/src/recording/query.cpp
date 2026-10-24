#include "reader.hpp"
#include "analysis_detail.hpp"
#include "spatial_analysis.hpp"
#include <array>
#include <map>
#include <cmath>
#include <limits>
#include <iterator>

namespace bike::recording {
io::Value Recording::query(std::string_view sql, std::uint64_t maximum, std::span<const io::Value> parameters) const {
    detail::limit(maximum);
    auto cursor = stream(sql, parameters);
    detail::Array columns;
    for (const auto& name : cursor.columns()) columns.push_back(detail::text(name));
    std::vector<io::Value::Object> rows;
    while (rows.size() <= maximum) {
        const auto wanted = std::min<std::uint64_t>(4096, maximum + 1 - rows.size());
        auto batch = cursor.next_batch(static_cast<std::size_t>(wanted));
        if (batch.empty()) break;
        rows.insert(rows.end(), std::make_move_iterator(batch.begin()), std::make_move_iterator(batch.end()));
    }
    auto out = detail::bounded(std::move(rows), maximum);
    detail::set(out, "columns", io::Value{std::move(columns)});
    return out;
}
io::Value query(const Recording& recording, std::string_view sql, std::uint64_t maximum) { return recording.query(sql, maximum); }

viz::Channels spatial_report_channels(const Recording& recording, std::int64_t episode) {
    using namespace detail;
    if (recording.schema_version() == 1) throw std::invalid_argument("recording schema 1 has no spatial channels");
    const auto episode_row = recording.episode(episode);
    const std::array parameters{integer(episode)};
    std::map<std::string, std::vector<double>> columns;
    const auto number = [](const Value& value) {
        if (io::is_null(value) || std::holds_alternative<io::Tagged>(value.data)) return std::numeric_limits<double>::quiet_NaN();
        if (const auto* boolean = std::get_if<bool>(&value.data)) return *boolean ? 1. : 0.;
        return io::as_double(value, "spatial report channel");
    };
    const auto flatten = [&](std::string prefix, const Value& value) {
        if (const auto* values = std::get_if<Array>(&value.data)) {
            for (std::size_t i=0; i<values->size(); ++i) columns[prefix + '.' + std::to_string(i)].push_back(number((*values)[i]));
        } else columns[std::move(prefix)].push_back(number(value));
    };
    const auto rows = recording.rows("SELECT p.time_s,p.interval_end_s,s.* FROM channel_spatial s JOIN physics p USING(episode_id,interval_id) WHERE s.episode_id=? ORDER BY s.interval_id",parameters);
    constexpr std::array scalar_names{"attitude_valid","roll_rad","pitch_rad","heading_rad","steer_angle_rad","steer_rate_rad_s",
        "steer_torque_nm","steer_work_j","lateral_displacement_m","front_contact_valid","rear_contact_valid",
        "front_lateral_slip_mps","rear_lateral_slip_mps","front_normal_load_n","rear_normal_load_n"};
    for (const auto& row:rows) {
        columns["time_s"].push_back(number(field(row,"time_s")));
        columns["spatial.energy_time_s"].push_back(number(field(row,"interval_end_s")));
        for (const auto name:scalar_names) flatten(std::string("spatial.")+name,field(row,name));
        for (const auto name:{"root_position_world_m","root_quat_world_from_body_wxyz"})
            flatten(std::string("spatial.")+name,field(row,name));
    }
    for (const auto& row:recording.rows("SELECT * FROM observations WHERE episode_id=? ORDER BY observation_id",parameters)) {
        for (const auto name:{"time_s","source_time_s","valid","valid_mask","specific_force_sensor_mps2","angular_rate_sensor_rad_s"})
            flatten(std::string("observations.")+name,field(row,name));
    }
    (void)episode_row;
    const auto work=recording.rows("SELECT hold_work_j,tyre_loss_j FROM channel_spatial_energy WHERE episode_id=? ORDER BY interval_id",parameters);
    const auto geometry=recording.rows("SELECT front_wheel_position_world_m,rear_wheel_position_world_m FROM channel_spatial_geometry WHERE episode_id=? ORDER BY interval_id",parameters);
    if(work.size()!=rows.size()||geometry.size()!=rows.size())throw RecordingError("spatial auxiliary channel extents differ from accepted intervals");
    for(const auto& row:work)for(const auto name:{"hold_work_j","tyre_loss_j"})flatten(std::string("spatial.")+name,field(row,name));
    for(const auto& row:geometry)for(const auto side:{"front","rear"}) {
        const auto& position=io::as_array(field(row,std::string(side)+"_wheel_position_world_m"),"recorded wheel position");
        if(position.size()!=3)throw RecordingError("recorded wheel position must have three world components");
        constexpr std::array axes{"x","y","z"};
        for(std::size_t axis=0;axis<3;++axis)columns[std::string("spatial.")+side+"_wheel_"+axes[axis]+"_m"].push_back(number(position[axis]));
    }
    viz::Channels result;
    for(auto& [name,values]:columns)result.emplace_back(std::move(name),std::move(values));
    return result;
}
} // namespace bike::recording
