#include "plot_internal.hpp"
#include "../io/json.hpp"
#include "../platform/files.hpp"
#include <algorithm>
#include <charconv>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <fstream>
#include <limits>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <system_error>
#include <sys/stat.h>
#include <unistd.h>

namespace bike::viz {
namespace {
struct FileCloser {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) gsl::owner is unavailable; the deleter is the ownership boundary
    void operator()(std::FILE *file) const noexcept { (void)std::fclose(file); }
};
} // namespace

const std::vector<double>* numeric(const Channels& channels,std::string_view name) {
    for(const auto& [key,column]:channels)if(key==name)return std::get_if<std::vector<double>>(&column);
    return nullptr;
}
const std::vector<std::string>* strings(const Channels& channels,std::string_view name) {
    for(const auto& [key,column]:channels)if(key==name)return std::get_if<std::vector<std::string>>(&column);
    return nullptr;
}
void validate(const Plot& plot) {
    const auto label=[](std::string_view text){if(text.size()>16384||text.find('\0')!=std::string_view::npos)throw std::invalid_argument("plot label exceeds bounds or contains NUL");
        (void)io::canonical_json(io::Value{std::string(text)});};
    label(plot.title);label(plot.x_label);label(plot.y_label);label(plot.secondary_y_label);
    if(plot.series.size()>256||plot.markers.size()>100000||plot.notes.size()>1024)throw std::invalid_argument("plot has too many series/markers/notes");
    std::size_t points=0;
    for(const auto& series:plot.series){label(series.name);if(series.x.size()!=series.y.size())throw std::invalid_argument("plot series '"+series.name+"': incompatible x/y lengths");
        if(series.x.size()>2000000-points)throw std::invalid_argument("plot exceeds two million points");
        points+=series.x.size();
        if(!series.color.empty())(void)detail::rgb(series.color);
        }
    for(const auto& marker:plot.markers){label(marker.label);if(!std::isfinite(marker.first)||!std::isfinite(marker.second))throw std::invalid_argument("plot marker must be finite");(void)detail::rgb(marker.color);}
    for(const auto& note:plot.notes)label(note);
    for(const auto& range:{plot.x_range,plot.y_range})if(range&&(!std::isfinite(range->first)||!std::isfinite(range->second)||range->first>=range->second))throw std::invalid_argument("plot axis range must increase and be finite");
    if(plot.log_y&&plot.y_range&&plot.y_range->first<=0.)throw std::invalid_argument("logarithmic plot range must be positive");
}
void validate(const Report& report) {
    if(report.panels.empty()||report.panels.size()>128||report.columns==0||report.columns>8)throw std::invalid_argument("report panel layout is invalid");
    if(report.width==0||report.height==0||report.width>16384||report.height>16384||static_cast<std::uint64_t>(report.width)*report.height>64000000ULL)
        throw std::invalid_argument("report image dimensions are outside the 64-million-pixel limit");
    if(report.title.size()>16384||report.title.find('\0')!=std::string::npos)throw std::invalid_argument("report title exceeds bounds or contains NUL");
    (void)io::canonical_json(io::Value{report.title});
    std::size_t total=0;
    for(const auto& plot:report.panels){validate(plot);for(const auto& series:plot.series){
        if(series.x.size()>8000000-total)throw std::invalid_argument("report exceeds eight million total points");
        total+=series.x.size();
        }
        }
}
Channels load_ride_csv(const std::filesystem::path& path) {
    // RFC 4180 fields, including embedded commas/newlines and escaped quotes.
    // Keep numeric-vs-text columns consistent with csv.DictReader + float.
    auto handle=platform::open_regular(path);
    std::unique_ptr<std::FILE,FileCloser> in(::fdopen(handle.get(),"rb"));
    if(!in)throw std::system_error(errno,std::generic_category(),"cannot read ride CSV: "+path.string());
    (void)handle.release();
    std::vector<std::vector<std::string>> rows;std::vector<std::string> row;std::string field;
    bool quoted=false,closed_quote=false,at_start=true;std::size_t bytes=0;
    const auto finish_field=[&]{row.push_back(std::move(field));field.clear();at_start=true;closed_quote=false;};
    const auto finish_row=[&]{finish_field();if(!(row.size()==1&&row[0].empty()))rows.push_back(std::move(row));row.clear();};
    // NOLINTNEXTLINE(clang-analyzer-unix.Stream) fgetc returning EOF is the parser's termination signal
    const auto get=[&](){const int c=std::fgetc(in.get());if(c!=EOF&&++bytes>512ULL*1024ULL*1024ULL)throw std::invalid_argument("ride CSV exceeds 512 MiB");return c;};
    // NOLINTNEXTLINE(clang-analyzer-unix.Stream,cert-err33-c) ungetc is only issued after a successful fgetc; EOF pushback never happens
    const auto peek=[&](){const int c=std::fgetc(in.get());if(c!=EOF)std::ungetc(c,in.get());return c;};
    int next=0;
    while((next=get())!=EOF){
        const char c=static_cast<char>(next);
        if(quoted){if(c=='"'){if(peek()=='"'){(void)get();field.push_back('"');}else{quoted=false;closed_quote=true;}}else field.push_back(c);continue;}
        if(c=='"'&&at_start){quoted=true;at_start=false;continue;}
        if(c==','){finish_field();continue;}
        if(c=='\r'||c=='\n'){if(c=='\r'&&peek()=='\n')(void)get();finish_row();continue;}
        if(closed_quote)throw std::invalid_argument("ride CSV has text after a closing quote");
        field.push_back(c);at_start=false;
    }
    if(std::ferror(in.get()))throw std::runtime_error("ride CSV read failed");
    if(quoted)throw std::invalid_argument("ride CSV contains an unterminated quoted field");
    if(!field.empty()||!row.empty()||closed_quote)finish_row();
    if(rows.empty())return {};
    std::unordered_set<std::string> names;
    Channels channels;channels.reserve(rows.front().size());
    for(std::size_t column=0;column<rows.front().size();++column){
        const auto& name=rows.front()[column];if(name.empty()||!names.insert(name).second)throw std::invalid_argument("ride CSV has empty/duplicate header");
        std::vector<std::string> text;std::vector<double> numbers;bool numeric_column=true;
        for(std::size_t line=1;line<rows.size();++line){
            if(rows[line].size()!=rows.front().size())throw std::invalid_argument("ride CSV has inconsistent row width at row "+std::to_string(line+1));
            text.push_back(rows[line][column]);const auto& value=text.back();double parsed=std::numeric_limits<double>::quiet_NaN();
            if(!value.empty()){
                std::string_view token=value;const auto space=[](char c){return c==' '||c=='\t'||c=='\r'||c=='\n'||c=='\f'||c=='\v';};
                while(!token.empty()&&space(token.front()))token.remove_prefix(1);
                while(!token.empty()&&space(token.back()))token.remove_suffix(1);
                if(!token.empty()&&token.front()=='+')token.remove_prefix(1);
                if(token.empty())numeric_column=false;
                else{const auto* end=std::to_address(token.end());
                    // NOLINTNEXTLINE(bugprone-suspicious-stringview-data-usage) from_chars takes begin/end pointers, not NUL-terminated input
                    const auto result=std::from_chars(token.data(),end,parsed);if(result.ec!=std::errc{}||result.ptr!=end)numeric_column=false;}}
            numbers.push_back(parsed);
        }
        if(rows.size()>1)channels.emplace_back(name,numeric_column?Column{std::move(numbers)}:Column{std::move(text)});
    }
    return channels;
}
void write_artifact(const Report& report,const std::filesystem::path& path) {
    const auto extension=path.extension().string();
    if(extension==".png")write_png(report,path);
    else if(extension==".svg")detail::write_text(path,to_svg(report));
    else if(extension==".html")detail::write_text(path,to_html(report));
    else throw std::invalid_argument("plot destination must end in .png, .svg or .html");
}
namespace detail {
std::vector<double> required(const Channels& channels,std::string_view name) {
    const auto* values=numeric(channels,name);if(!values)throw std::invalid_argument("channels lack numeric "+std::string(name));return *values;
}
void add_series(Plot& plot,const Channels& channels,const std::vector<double>& x,std::string_view key,std::string_view selected,bool secondary,SeriesStyle style) {
    if(const auto* y=numeric(channels,key))plot.series.push_back({.name=std::string(key),.x=x,.y=*y,.color=std::string(selected),.style=style,.secondary_y=secondary});
}
void track_markers(Plot& plot,const terrain::TrackSpec& track) {for(const auto& [x,label]:track.markers())plot.markers.push_back({.kind=MarkerKind::vertical,.first=x,.second=0.,.label=label});}
std::string number(double value) {
    if(!std::isfinite(value))return "null";
    std::array<char,64> data{};const auto result=std::to_chars(data.data(),std::to_address(data.end()),value);if(result.ec!=std::errc{})throw std::runtime_error("plot numeric formatting failed");return {data.data(),result.ptr};
}
std::string json_string(std::string_view value) {
    auto out=io::canonical_json(io::Value{std::string(value)});
    std::string safe;safe.reserve(out.size());
    // Escaping < defeats </script>, even inside JSON string values. U+2028/9
    // are escaped for older JavaScript parsers. DOM labels use textContent.
    for(std::size_t i=0;i<out.size();++i){const unsigned char c=static_cast<unsigned char>(out[i]);
        if(c=='<')safe+="\\u003c";else if(c=='>')safe+="\\u003e";else if(c=='&')safe+="\\u0026";
        else if(i+2<out.size()&&c==0xe2&&static_cast<unsigned char>(out[i+1])==0x80&&(static_cast<unsigned char>(out[i+2])==0xa8||static_cast<unsigned char>(out[i+2])==0xa9)){
            safe+=static_cast<unsigned char>(out[i+2])==0xa8?"\\u2028":"\\u2029";i+=2;
        }else safe+=out[i];}
    return safe;
}
std::string escape_xml(std::string_view value) {
    std::string out;out.reserve(value.size());for(const char c:value){switch(c){case '&':out+="&amp;";break;case '<':out+="&lt;";break;case '>':out+="&gt;";break;case '"':out+="&quot;";break;case '\'':out+="&apos;";break;default:if(static_cast<unsigned char>(c)>=32||c=='\n'||c=='\t')out+=c;}}
    return out;
}
std::array<unsigned char,3> rgb(std::string_view value) {
    if(value.size()!=7||value.front()!='#')throw std::invalid_argument("plot colors must use #RRGGBB");
    std::array<unsigned char,3> out{};
    for(std::size_t i=0;i<3;++i){unsigned component=0;const auto token=value.substr(1+2*i,2);const auto* end=std::to_address(token.end());
        // NOLINTNEXTLINE(bugprone-suspicious-stringview-data-usage) from_chars takes begin/end pointers, not NUL-terminated input
        const auto result=std::from_chars(token.data(),end,component,16);
        if(result.ec!=std::errc{}||result.ptr!=end)throw std::invalid_argument("invalid plot hex color");
        out[i]=static_cast<unsigned char>(component);
        }
    return out;
}
std::string color(const Series& series,std::size_t index) {return series.color.empty()?std::string(palette[index%palette.size()]):series.color;}
Limits limits(const Plot& plot,bool secondary) {
    Limits limits{.x0=std::numeric_limits<double>::infinity(),.x1=-std::numeric_limits<double>::infinity(),.y0=std::numeric_limits<double>::infinity(),.y1=-std::numeric_limits<double>::infinity()};
    for(const auto& series:plot.series){for(std::size_t i=0;i<series.x.size();++i){
        if(!std::isfinite(series.x[i])||!std::isfinite(series.y[i]))continue;
        limits.x0=std::min(limits.x0,series.x[i]);limits.x1=std::max(limits.x1,series.x[i]);
        if(series.secondary_y!=secondary||(plot.log_y&&series.y[i]<=0.))continue;
        limits.y0=std::min(limits.y0,series.y[i]);limits.y1=std::max(limits.y1,series.y[i]);}}
    if(!std::isfinite(limits.x0)){limits.x0=0.;limits.x1=1.;}
    if(!std::isfinite(limits.y0)){limits.y0=plot.log_y?.01:0.;limits.y1=1.;}
    if(limits.x0==limits.x1){const double margin=std::max(1.,std::abs(limits.x0)*.05);limits.x0-=margin;limits.x1+=margin;}
    if(limits.y0==limits.y1){const double margin=std::max(1.,std::abs(limits.y0)*.05);limits.y0-=margin;limits.y1+=margin;if(plot.log_y)limits.y0=limits.y1*.01;}
    if(plot.x_range){limits.x0=plot.x_range->first;limits.x1=plot.x_range->second;}
    if(plot.y_range&&!secondary){limits.y0=plot.y_range->first;limits.y1=plot.y_range->second;}
    else if(!plot.log_y){const double margin=(limits.y1-limits.y0)*.06;limits.y0-=margin;limits.y1+=margin;}
    if(plot.log_y){limits.y0=std::log10(limits.y0);limits.y1=std::log10(limits.y1);}
    if(!std::isfinite(limits.x0)||!std::isfinite(limits.x1)||!std::isfinite(limits.y0)||!std::isfinite(limits.y1)||
        !std::isfinite(limits.x1-limits.x0)||!std::isfinite(limits.y1-limits.y0)||limits.x1<=limits.x0||limits.y1<=limits.y0)
        throw std::invalid_argument("plot axis extent is not representable");
    return limits;
}
void write_text(const std::filesystem::path& path,std::string_view value) {
    if(path.empty()||path.filename().empty()||path.native().find('\0')!=std::string::npos)throw std::invalid_argument("plot destination is invalid");
    if(!path.parent_path().empty())std::filesystem::create_directories(path.parent_path());
    // Plot exports intentionally replace an ordinary destination. Open first,
    // inspect that exact descriptor, and only then truncate: no leaf-symlink
    // race and no accidental writes to a device or FIFO.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg) narrow controlled use
    platform::FileHandle out(::open(path.c_str(),O_WRONLY|O_CREAT|O_NOFOLLOW|O_CLOEXEC|O_NONBLOCK,0666));
    if(!out.valid())throw std::system_error(errno,std::generic_category(),"cannot open plot destination: "+path.string());
    struct stat status{};
    if(::fstat(out.get(),&status)!=0)throw std::system_error(errno,std::generic_category(),"cannot inspect plot destination");
    if(!S_ISREG(status.st_mode))throw std::invalid_argument("plot destination is not a regular file");
    if(::ftruncate(out.get(),0)!=0)throw std::system_error(errno,std::generic_category(),"cannot truncate plot destination");
    platform::write_all(out,std::as_bytes(std::span(value)));out.close();
}
Drawing draw(const Report& report) {
    validate(report);Drawing out{.width=report.width,.height=report.height,.primitives={}};
    const auto rectangle=[&](double x,double y,double w,double h,std::string_view color){out.primitives.push_back({.kind=Primitive::Kind::rectangle,.points={{.x=x,.y=y},{.x=w,.y=h}},.color=std::string(color),.text={},.thickness=1.,.font_size=14.,.fill=true,.dashed=false});};
    const auto text=[&](double x,double y,std::string label,double size=14.,std::string_view color=foreground){out.primitives.push_back({.kind=Primitive::Kind::text,.points={{.x=x,.y=y}},.color=std::string(color),.text=std::move(label),.thickness=1.,.font_size=size,.fill=false,.dashed=false});};
    const auto line=[&](Point a,Point b,std::string_view color,double width=1.,bool dash=false){out.primitives.push_back({.kind=Primitive::Kind::line,.points={a,b},.color=std::string(color),.text={},.thickness=width,.font_size=14.,.fill=false,.dashed=dash});};
    rectangle(0,0,report.width,report.height,background);text(22,30,report.title,22.);
    const auto rows=(report.panels.size()+report.columns-1)/report.columns;
    const double panel_width=static_cast<double>(report.width)/report.columns,panel_height=(static_cast<double>(report.height)-52.)/static_cast<double>(rows);
    for(std::size_t index=0;index<report.panels.size();++index){const auto& plot=report.panels[index];
        // NOLINTNEXTLINE(bugprone-integer-division) deliberate: panel row index is integer division
        const double outer_x=static_cast<double>(index%report.columns)*panel_width,outer_y=52.+static_cast<double>(index/report.columns)*panel_height;
        const auto legend_rows=(plot.series.size()+2)/3;
        const double left=outer_x+76.,right=outer_x+panel_width-65.,top=outer_y+44.,
            bottom=outer_y+panel_height-67.-static_cast<double>(legend_rows)*15.;
        if(right<=left||bottom<=top)throw std::invalid_argument("report dimensions too small for the panel count");
        rectangle(left,top,right-left,bottom-top,panel);text(left,outer_y+22.,plot.title,16.);
        auto primary=limits(plot),secondary=limits(plot,true);
        if(plot.equal_axes){const double x_scale=(primary.x1-primary.x0)/(right-left),y_scale=(primary.y1-primary.y0)/(bottom-top);
            if(x_scale<y_scale){const double center=(primary.x0+primary.x1)/2.,half=y_scale*(right-left)/2.;primary.x0=center-half;primary.x1=center+half;}
            else{const double center=(primary.y0+primary.y1)/2.,half=x_scale*(bottom-top)/2.;primary.y0=center-half;primary.y1=center+half;}}
        const auto point=[&](double x,double y,bool second=false){const auto& axis=second?secondary:primary;return Point{.x=left+(x-primary.x0)/(primary.x1-primary.x0)*(right-left),.y=bottom-((plot.log_y?std::log10(y):y)-axis.y0)/(axis.y1-axis.y0)*(bottom-top)};};
        for(int tick=0;tick<=5;++tick){const double fraction=static_cast<double>(tick)/5.;const double x=left+fraction*(right-left),y=bottom-fraction*(bottom-top);
            line({.x=x,.y=top},{.x=x,.y=bottom},grid, .6,true);line({.x=left,.y=y},{.x=right,.y=y},grid,.6,true);
            const auto compact=[](double value){std::ostringstream o;o.imbue(std::locale::classic());o.precision(4);o<<value;return o.str();};
            text(x-13,bottom+20,compact(primary.x0+fraction*(primary.x1-primary.x0)),11.,muted);
            const double value=primary.y0+fraction*(primary.y1-primary.y0);text(outer_x+5,y+4,compact(plot.log_y?std::pow(10.,value):value),11.,muted);
            if(!plot.secondary_y_label.empty()){const double second=secondary.y0+fraction*(secondary.y1-secondary.y0);text(right+6,y+4,compact(plot.log_y?std::pow(10.,second):second),11.,muted);}
        }
        text(left,bottom+42,plot.x_label,12.,muted);text(left,top-8,plot.y_label,12.,muted);
        if(!plot.secondary_y_label.empty())text(right-55,top-8,plot.secondary_y_label,12.,muted);
        for(const auto& marker:plot.markers){
            if(marker.kind==MarkerKind::horizontal){const auto pt=point(primary.x0,marker.first);if(pt.y>=top&&pt.y<=bottom){line({.x=left,.y=pt.y},{.x=right,.y=pt.y},marker.color,1.,true);if(!marker.label.empty())text(left+5,pt.y-4,marker.label,10.,marker.color);}}
            else if(marker.kind==MarkerKind::vertical){const auto pt=point(marker.first,plot.log_y?std::pow(10.,primary.y0):primary.y0);if(pt.x>=left&&pt.x<=right){line({.x=pt.x,.y=top},{.x=pt.x,.y=bottom},marker.color,1.,true);if(!marker.label.empty())text(pt.x+3,top+14,marker.label,10.,marker.color);}}
            else{const auto a=point(marker.first,plot.log_y?std::pow(10.,primary.y0):primary.y0),b=point(marker.second,plot.log_y?std::pow(10.,primary.y0):primary.y0);
                const double x0=std::clamp(a.x,left,right),x1=std::clamp(b.x,left,right);if(x1>x0)rectangle(x0,top,x1-x0,bottom-top,"#3a2c50");}
        }
        for(std::size_t series_index=0;series_index<plot.series.size();++series_index){const auto& series=plot.series[series_index];const auto selected=color(series,series_index);
            Primitive path;path.kind=Primitive::Kind::line;path.color=selected;path.thickness=1.4;path.dashed=series.dashed;
            const auto flush=[&]{if(!path.points.empty()){out.primitives.push_back(std::move(path));path=Primitive{};path.color=selected;path.thickness=1.4;path.dashed=series.dashed;}};
            std::optional<Point> previous;
            const auto inside=[&](Point p){return p.x>=left&&p.x<=right&&p.y>=top&&p.y<=bottom;};
            const auto append_segment=[&](Point a,Point b){
                // Liang-Barsky clips the segment, not its samples. Clamping
                // samples would invent a plateau at every explicit axis limit.
                const double dx=b.x-a.x,dy=b.y-a.y;double low=0.,high=1.;
                const std::array<double,4> p{-dx,dx,-dy,dy},q{a.x-left,right-a.x,a.y-top,bottom-a.y};
                for(std::size_t side=0;side<4;++side){if(p[side]==0.){if(q[side]<0.){flush();return;}continue;}
                    const double ratio=q[side]/p[side];if(p[side]<0.)low=std::max(low,ratio);else high=std::min(high,ratio);}
                if(low>high){flush();return;}const Point start{.x=a.x+low*dx,.y=a.y+low*dy},end{.x=a.x+high*dx,.y=a.y+high*dy};
                if(!path.points.empty()&&(path.points.back().x!=start.x||path.points.back().y!=start.y))flush();
                if(path.points.empty())path.points.push_back(start);
                path.points.push_back(end);
                if(high<1.)flush();
            };
            for(std::size_t i=0;i<series.x.size();++i){const double x=series.x[i],y=series.y[i];
                if(!std::isfinite(x)||!std::isfinite(y)||(plot.log_y&&y<=0.)){flush();previous.reset();continue;}
                const auto pt=point(x,y,series.secondary_y);
                if(!std::isfinite(pt.x)||!std::isfinite(pt.y))throw std::invalid_argument("plot point cannot be represented in the selected axis range");
                if(series.style==SeriesStyle::bars){const double width=std::max(1.,(right-left)/static_cast<double>(std::max<std::size_t>(series.x.size(),1))*.8);
                    const double base=plot.log_y?bottom:std::clamp(point(x,0.,series.secondary_y).y,top,bottom),py=std::clamp(pt.y,top,bottom);
                    const double x0=std::clamp(pt.x-width/2.,left,right),x1=std::clamp(pt.x+width/2.,left,right);
                    if(x1>x0)rectangle(x0,std::min(py,base),x1-x0,std::max(1.,std::abs(base-py)),selected);}
                else if(series.style==SeriesStyle::points){if(inside(pt))rectangle(pt.x-2.,pt.y-2.,4.,4.,selected);}
                else{if(previous){if(series.style==SeriesStyle::step){const Point corner{.x=pt.x,.y=previous->y};append_segment(*previous,corner);append_segment(corner,pt);}else append_segment(*previous,pt);}
                    else if(inside(pt))path.points.push_back(pt);
                    previous=pt;
                    }
            }flush();
            const double lx=left+static_cast<double>(series_index%3)*((right-left)/3.);
            // NOLINTNEXTLINE(bugprone-integer-division) deliberate: legend column index is integer division
            const double ly=bottom+60.+static_cast<double>(series_index/3)*15.;
            line({.x=lx,.y=ly-4},{.x=lx+12,.y=ly-4},selected,2.,series.dashed);text(lx+17,ly,series.name,10.,foreground);
        }
        for(std::size_t note=0;note<plot.notes.size();++note)text(left+8,top+30.+static_cast<double>(note)*15.,plot.notes[note],11.,muted);
    }
    return out;
}
} // namespace detail
} // namespace bike::viz
