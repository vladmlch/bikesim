#include "plot_internal.hpp"
namespace bike::viz {
namespace detail {
std::string drawing_svg(const Drawing& drawing) {
    std::string out=R"(<svg xmlns="http://www.w3.org/2000/svg" role="img" width=")"+std::to_string(drawing.width)+"\" height=\""+
        std::to_string(drawing.height)+"\" viewBox=\"0 0 "+std::to_string(drawing.width)+" "+std::to_string(drawing.height)+"\">";
    for(const auto& p:drawing.primitives){
        if(p.kind==Primitive::Kind::rectangle){const auto a=p.points.at(0),b=p.points.at(1);out+="<rect x=\""+number(a.x)+"\" y=\""+number(a.y)+"\" width=\""+number(b.x)+"\" height=\""+number(b.y)+"\" fill=\""+p.color+"\"/>";}
        else if(p.kind==Primitive::Kind::text){const auto point=p.points.at(0);out+="<text x=\""+number(point.x)+"\" y=\""+number(point.y)+"\" fill=\""+p.color+R"(" font-family="DejaVu Sans, sans-serif" font-size=")"+number(p.font_size)+"\">"+escape_xml(p.text)+"</text>";}
        else if(p.points.size()==1){const auto point=p.points.front();out+="<circle cx=\""+number(point.x)+"\" cy=\""+number(point.y)+R"(" r="2" fill=")"+p.color+"\"/>";}
        else if(!p.points.empty()){out+=R"(<polyline fill="none" stroke=")"+p.color+"\" stroke-width=\""+number(p.thickness)+"\""+(p.dashed?" stroke-dasharray=\"4 4\"":"")+" points=\"";
            for(const auto point:p.points)out+=number(point.x)+","+number(point.y)+" ";
            out+="\"/>";
            }
    }
    return out+"</svg>";
}
}
std::string to_svg(const Report& report){return detail::drawing_svg(detail::draw(report));}
std::string to_svg(const Plot& plot){return to_svg(Report{.title=plot.title,.panels={plot},.columns=1,.width=1280,.height=720});}
} // namespace bike::viz
