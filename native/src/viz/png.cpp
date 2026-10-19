#include "plot_internal.hpp"
#include <exception>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <utility>
#if defined(BIKE_HAS_CAIRO) && BIKE_HAS_CAIRO
#include <cairo.h>
#endif

namespace bike::viz {
namespace {
#if !defined(BIKE_HAS_CAIRO) || !BIKE_HAS_CAIRO
// A bounded first-party RGB raster/PNG encoder keeps headless PNG export
// independent of a display or an unqualified system Cairo. The optional
// pinned Cairo SDK provides antialiased font rendering using the same layout.
// Stored DEFLATE blocks are standard PNG/zlib; no executable is launched.
void u32(std::string& out,std::uint32_t value) {for(std::uint32_t const shift:{24U,16U,8U,0U})out.push_back(static_cast<char>((value>>shift)&255U));}
std::uint32_t crc32(std::string_view text) {
    std::uint32_t crc=0xffffffffU;
    for(const char raw:text){const auto byte=static_cast<unsigned char>(raw);crc^=byte;for(unsigned bit=0;bit<8;++bit)crc=(crc>>1U)^(0xedb88320U&(0U-(crc&1U)));}
    return ~crc;
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
void chunk(std::string& output,std::string_view type,std::string_view bytes) {
    if(bytes.size()>std::numeric_limits<std::uint32_t>::max())throw std::overflow_error("PNG chunk too large");
    u32(output,static_cast<std::uint32_t>(bytes.size()));std::string content(type);content+=bytes;output+=content;u32(output,crc32(content));
}
std::array<unsigned char,7> glyph(char raw) {
    const char c=raw>='a'&&raw<='z'?static_cast<char>(raw-'a'+'A'):raw;
    // Original compact 5x7 glyphs. Non-ASCII code points use a visible box;
    // exact UTF-8 labels also remain in the PNG iTXt semantic payload.
    switch(c){
        case 'A':return {14,17,17,31,17,17,17};case 'B':return {30,17,17,30,17,17,30};case 'C':return {14,17,16,16,16,17,14};
        case 'D':return {30,17,17,17,17,17,30};case 'E':return {31,16,16,30,16,16,31};case 'F':return {31,16,16,30,16,16,16};
        case 'G':return {14,17,16,23,17,17,14};case 'H':return {17,17,17,31,17,17,17};case 'I':return {14,4,4,4,4,4,14};
        case 'J':return {7,2,2,2,2,18,12};case 'K':return {17,18,20,24,20,18,17};case 'L':return {16,16,16,16,16,16,31};
        case 'M':return {17,27,21,21,17,17,17};case 'N':return {17,25,21,19,17,17,17};case 'O':return {14,17,17,17,17,17,14};
        case 'P':return {30,17,17,30,16,16,16};case 'Q':return {14,17,17,17,21,18,13};case 'R':return {30,17,17,30,20,18,17};
        case 'S':return {15,16,16,14,1,1,30};case 'T':return {31,4,4,4,4,4,4};case 'U':return {17,17,17,17,17,17,14};
        case 'V':return {17,17,17,17,17,10,4};case 'W':return {17,17,17,21,21,21,10};case 'X':return {17,17,10,4,10,17,17};
        case 'Y':return {17,17,10,4,4,4,4};case 'Z':return {31,1,2,4,8,16,31};
        case '0':return {14,17,19,21,25,17,14};case '1':return {4,12,4,4,4,4,14};case '2':return {14,17,1,2,4,8,31};
        case '3':return {30,1,1,14,1,1,30};case '4':return {2,6,10,18,31,2,2};case '5':return {31,16,16,30,1,1,30};
        case '6':return {14,16,16,30,17,17,14};case '7':return {31,1,2,4,8,8,8};case '8':return {14,17,17,14,17,17,14};case '9':return {14,17,17,15,1,1,14};
        case '.':return {0,0,0,0,0,6,6};case ',':return {0,0,0,0,6,6,4};case ':':return {0,6,6,0,6,6,0};case ';':return {0,6,6,0,6,6,4};
        case '-':return {0,0,0,31,0,0,0};case '+':return {0,4,4,31,4,4,0};case '=':return {0,0,31,0,31,0,0};case '/':return {1,1,2,4,8,16,16};
        case '\\':return {16,16,8,4,2,1,1};case '(':return {2,4,8,8,8,4,2};case ')':return {8,4,2,2,2,4,8};
        case '[':return {14,8,8,8,8,8,14};case ']':return {14,2,2,2,2,2,14};case '%':return {25,25,2,4,8,19,19};
        case '_':return {0,0,0,0,0,0,31};case '|':return {4,4,4,4,4,4,4};case '>':return {16,8,4,2,4,8,16};case '<':return {1,2,4,8,4,2,1};
        case '!':return {4,4,4,4,4,0,4};case '?':return {14,17,1,2,4,0,4};case '@':return {14,17,23,21,23,16,14};
        case ' ':return {};default:return {31,17,17,17,17,17,31};
    }
}
struct Raster {
    unsigned width,height;std::vector<unsigned char> pixels;
    explicit Raster(unsigned w,unsigned h):width(w),height(h),pixels(static_cast<std::size_t>(w)*h*3U,0){}
    void dot(int x,int y,const std::array<unsigned char,3>& color){if(x<0||y<0||std::cmp_greater_equal(x,width)||std::cmp_greater_equal(y,height))return;
        const auto offset=(static_cast<std::size_t>(y)*width+static_cast<unsigned>(x))*3U;for(std::size_t i=0;i<3;++i)pixels[offset+i]=color[i];}
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    void rectangle(int x,int y,int w,int h,const std::array<unsigned char,3>& color){const int right=std::min(x+w,static_cast<int>(width)),bottom=std::min(y+h,static_cast<int>(height));
        for(int row=std::max(0,y);row<bottom;++row)for(int col=std::max(0,x);col<right;++col)dot(col,row,color);}
    void line(detail::Point a,detail::Point b,const std::array<unsigned char,3>& color,double thickness,bool dashed){
        int x=static_cast<int>(std::lround(a.x)),y=static_cast<int>(std::lround(a.y));const int x1=static_cast<int>(std::lround(b.x)),y1=static_cast<int>(std::lround(b.y));
        const int dx=std::abs(x1-x),sx=x<x1?1:-1,dy=-std::abs(y1-y),sy=y<y1?1:-1;int error=dx+dy,index=0;
        const int size=std::max(1,static_cast<int>(std::ceil(thickness))),half=size/2;
        while(true){if(!dashed||(index/4)%2==0)rectangle(x-half,y-half,size,size,color);if(x==x1&&y==y1)break;const int e=2*error;if(e>=dy){error+=dy;x+=sx;}if(e<=dx){error+=dx;y+=sy;}++index;}
    }
    void text(detail::Point at,std::string_view text,double font_size,const std::array<unsigned char,3>& color){
        const int scale=std::max(1,static_cast<int>(std::lround(font_size/9.)));int x=static_cast<int>(std::lround(at.x)),y=static_cast<int>(std::lround(at.y))-7*scale;
        for(const char i : text){const auto c=static_cast<unsigned char>(i);if((c&0xc0U)==0x80U)continue;const auto pattern=glyph(c<128?static_cast<char>(c):'?');
            for(int row=0;row<7;++row)for(int col=0;col<5;++col)if((pattern[static_cast<std::size_t>(row)]&(1U<<static_cast<unsigned>(4-col)))!=0)rectangle(x+col*scale,y+row*scale,scale,scale,color);
            x+=6*scale;if(std::cmp_greater_equal(x,width))break;
        }
    }
};
std::string native_png(const detail::Drawing& drawing,const Report& report) {
    Raster image(drawing.width,drawing.height);
    for(const auto& p:drawing.primitives){const auto color=detail::rgb(p.color);
        if(p.kind==detail::Primitive::Kind::rectangle){const auto a=p.points.at(0),b=p.points.at(1);image.rectangle(static_cast<int>(std::floor(a.x)),static_cast<int>(std::floor(a.y)),static_cast<int>(std::ceil(b.x)),static_cast<int>(std::ceil(b.y)),color);}
        else if(p.kind==detail::Primitive::Kind::text)image.text(p.points.at(0),p.text,p.font_size,color);
        else if(p.points.size()==1)image.rectangle(static_cast<int>(std::lround(p.points[0].x))-1,static_cast<int>(std::lround(p.points[0].y))-1,3,3,color);
        else for(std::size_t i=1;i<p.points.size();++i)image.line(p.points[i-1],p.points[i],color,p.thickness,p.dashed);
    }
    const std::size_t stride=static_cast<std::size_t>(image.width)*3U;
    std::string scanlines;scanlines.reserve((stride+1)*image.height);
    for(unsigned row=0;row<image.height;++row){scanlines.push_back('\0');
        for(const auto byte:std::span(image.pixels).subspan(static_cast<std::size_t>(row)*stride,stride))scanlines.push_back(static_cast<char>(byte));}
    std::string compressed;compressed.reserve(scanlines.size()+scanlines.size()/65535*5+16);compressed.push_back('\x78');compressed.push_back('\x01');
    for(std::size_t offset=0;offset<scanlines.size();){const auto count=std::min<std::size_t>(65535,scanlines.size()-offset);compressed.push_back(offset+count==scanlines.size()?'\1':'\0');
        const auto length=static_cast<std::uint16_t>(count),inverse=static_cast<std::uint16_t>(~length);compressed.push_back(static_cast<char>(length&255U));compressed.push_back(static_cast<char>(length>>8U));
        compressed.push_back(static_cast<char>(inverse&255U));compressed.push_back(static_cast<char>(inverse>>8U));compressed.append(scanlines,offset,count);offset+=count;}
    std::uint32_t a=1,b=0;for(const char raw:scanlines){const auto c=static_cast<unsigned char>(raw);a=(a+c)%65521U;b=(b+a)%65521U;}u32(compressed,(b<<16U)|a);
    std::string output("\x89PNG\r\n\x1a\n",8),header;u32(header,drawing.width);u32(header,drawing.height);header.append("\x08\x02\x00\x00\x00",5);chunk(output,"IHDR",header);
    std::string semantic="bike-plot";semantic.append(5,'\0');semantic+=detail::report_json(report);chunk(output,"iTXt",semantic);chunk(output,"IDAT",compressed);chunk(output,"IEND",{});return output;
}
#endif
#if defined(BIKE_HAS_CAIRO) && BIKE_HAS_CAIRO
struct SurfaceDelete {void operator()(cairo_surface_t* p)const noexcept{if(p)cairo_surface_destroy(p);}};
struct ContextDelete {void operator()(cairo_t* p)const noexcept{if(p)cairo_destroy(p);}};
std::string cairo_png(const detail::Drawing& drawing) {
    std::unique_ptr<cairo_surface_t,SurfaceDelete> surface(cairo_image_surface_create(CAIRO_FORMAT_RGB24,static_cast<int>(drawing.width),static_cast<int>(drawing.height)));
    if(cairo_surface_status(surface.get())!=CAIRO_STATUS_SUCCESS)throw std::runtime_error("Cairo PNG surface allocation failed");
    std::unique_ptr<cairo_t,ContextDelete> context(cairo_create(surface.get()));auto* cr=context.get();
    cairo_select_font_face(cr,"DejaVu Sans",CAIRO_FONT_SLANT_NORMAL,CAIRO_FONT_WEIGHT_NORMAL);
    for(const auto& p:drawing.primitives){const auto rgb=detail::rgb(p.color);cairo_set_source_rgb(cr,rgb[0]/255.,rgb[1]/255.,rgb[2]/255.);
        if(p.kind==detail::Primitive::Kind::rectangle){const auto a=p.points.at(0),b=p.points.at(1);cairo_rectangle(cr,a.x,a.y,b.x,b.y);cairo_fill(cr);}
        else if(p.kind==detail::Primitive::Kind::text){cairo_set_font_size(cr,p.font_size);cairo_move_to(cr,p.points.at(0).x,p.points.at(0).y);cairo_show_text(cr,p.text.c_str());}
        else if(!p.points.empty()){cairo_set_line_width(cr,p.thickness);const std::array<double,2> dash{4.,4.};cairo_set_dash(cr,p.dashed?dash.data():nullptr,p.dashed?2:0,0.);
            cairo_move_to(cr,p.points.front().x,p.points.front().y);for(std::size_t i=1;i<p.points.size();++i)cairo_line_to(cr,p.points[i].x,p.points[i].y);if(p.points.size()==1)cairo_line_to(cr,p.points.front().x+.5,p.points.front().y);cairo_stroke(cr);}
    }
    if(cairo_status(cr)!=CAIRO_STATUS_SUCCESS)throw std::runtime_error("Cairo PNG rendering failed");
    struct Destination {std::string bytes;std::exception_ptr failure;};Destination destination;
    const auto status=cairo_surface_write_to_png_stream(surface.get(),[](void* opaque,const unsigned char* data,unsigned length) noexcept {
        auto* out=static_cast<Destination*>(opaque);try{
            // Cairo owns precisely `length` readable bytes during this C
            // callback; copy the bounded range before returning to the SDK.
            for(const auto byte:std::views::counted(data,static_cast<std::ptrdiff_t>(length)))out->bytes.push_back(static_cast<char>(byte));
            return CAIRO_STATUS_SUCCESS;
        }catch(...){out->failure=std::current_exception();return CAIRO_STATUS_WRITE_ERROR;}
    },&destination);
    if(destination.failure)std::rethrow_exception(destination.failure);if(status!=CAIRO_STATUS_SUCCESS)throw std::runtime_error("Cairo PNG encoding failed");return destination.bytes;
}
#endif
}
bool png_available() noexcept {return true;}
void write_png(const Report& report,const std::filesystem::path& path) {
    const auto drawing=detail::draw(report);
#if defined(BIKE_HAS_CAIRO) && BIKE_HAS_CAIRO
    detail::write_text(path,cairo_png(drawing));
#else
    detail::write_text(path,native_png(drawing,report));
#endif
}
void write_png(const Plot& plot,const std::filesystem::path& path,unsigned width,unsigned height) {write_png(Report{.title=plot.title,.panels={plot},.columns=1,.width=width,.height=height},path);}
} // namespace bike::viz
