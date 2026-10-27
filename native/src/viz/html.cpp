#include "plot_internal.hpp"
#include "../platform/provenance.hpp"
#include "bike_viz_paths.hpp"
#include <fstream>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace bike::viz {
namespace {
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) process-local table mirrors the reference module structure
std::mutex resource_mutex;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) process-local table mirrors the reference module structure
std::optional<std::filesystem::path> plotly_override;
std::optional<std::string> plotly_script() {
    std::optional<std::filesystem::path> configured;
    {std::scoped_lock<std::mutex> const lock(resource_mutex);configured=plotly_override;}
    std::vector<std::filesystem::path> candidates;
    if(configured)candidates.push_back(*configured);
    else{
        candidates.push_back(platform::install_prefix()/"share/bike_sim/plotly/plotly.min.js");
#ifdef BIKE_SOURCE_ROOT
        candidates.push_back(std::filesystem::path(BIKE_SOURCE_ROOT)/"native/resources/plotly/plotly.min.js");
#endif
    }
    for(const auto& candidate:candidates){
        std::error_code error;const auto size=std::filesystem::file_size(candidate,error);
        if(error){if(configured)throw std::runtime_error("configured Plotly resource unavailable: "+candidate.string());continue;}
        if(size==0||size>16000000)throw std::invalid_argument("Plotly resource exceeds 16 MB or is empty");
        std::ifstream in(candidate,std::ios::binary);std::string value(static_cast<std::size_t>(size),'\0');
        in.read(value.data(),static_cast<std::streamsize>(value.size()));if(!in)throw std::runtime_error("could not read Plotly resource");
        // Never accept a truncated HTML download as a JavaScript bundle.
        if(value.find("plotly.js")==std::string::npos||value.find("Plotly")==std::string::npos)
            throw std::invalid_argument("configured resource does not identify Plotly.js");
        return value;
    }
    return {};
}
constexpr std::string_view script=R"BIKEJS(
(()=>{'use strict';
const reports=JSON.parse(document.getElementById('bike-plot-data').textContent);
const colors=['#38bdf8','#fbbf24','#34d399','#f97316','#c084fc','#ef4444','#8b949e','#a855f7'];
const ns='http://www.w3.org/2000/svg';
const elem=(tag,attrs={},text)=>{const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,String(v));if(text!==undefined)e.textContent=text;return e;};
const finite=v=>typeof v==='number'&&Number.isFinite(v);
const fmt=v=>finite(v)?Number(v.toPrecision(6)).toString():'—';
const save=(text,type,name)=>{const link=document.createElement('a');const url=URL.createObjectURL(new Blob([text],{type}));link.href=url;link.download=name;link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
const panelDomain=(p,secondary=false)=>{let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;
for(const s of p.series)for(let i=0;i<s.x.length;i++){const x=s.x[i],y=s.y[i];if(!finite(x)||!finite(y))continue;xmin=Math.min(xmin,x);xmax=Math.max(xmax,x);if(s.secondary!==secondary||(p.log&&y<=0))continue;ymin=Math.min(ymin,y);ymax=Math.max(ymax,y);}
if(!finite(xmin)){xmin=0;xmax=1;}if(xmin===xmax){xmin-=1;xmax+=1;}if(!finite(ymin)){ymin=p.log ? .01 : 0;ymax=1;}if(ymin===ymax){const m=Math.max(1,Math.abs(ymin)*.05);ymin-=m;ymax+=m;if(p.log)ymin=ymax*.01;}
if(p.xrange)[xmin,xmax]=p.xrange;if(p.yrange&&!secondary)[ymin,ymax]=p.yrange;
if(p.log){ymin=Math.log10(ymin);ymax=Math.log10(ymax);}else if(!p.yrange){const m=(ymax-ymin)*.06;ymin-=m;ymax+=m;}return[xmin,xmax,ymin,ymax];};
let uid=0;
for(const report of reports){const section=document.createElement('section');const h=document.createElement('h2');h.textContent=report.title;section.append(h);document.getElementById('reports').append(section);
const toolbar=document.createElement('div');toolbar.className='toolbar';section.append(toolbar);
const note=document.createElement('span');note.textContent='Wheel: zoom · drag: pan · double-click: reset · legend: toggle · hover: values';toolbar.append(note);
const dataButton=document.createElement('button');dataButton.textContent='Export data';dataButton.onclick=()=>save(JSON.stringify(report,null,2),'application/json','plot-data.json');toolbar.append(dataButton);
const grid=document.createElement('div');grid.className='panels';grid.style.gridTemplateColumns=`repeat(${report.columns},minmax(0,1fr))`;section.append(grid);
for(const p of report.panels){const box=document.createElement('article');grid.append(box);const title=document.createElement('h3');title.textContent=p.title;box.append(title);
for(const text of p.notes){const note=document.createElement('p');note.className='note';note.textContent=text;box.append(note);}
if(window.Plotly){const chart=document.createElement('div');chart.className='plotly-chart';box.append(chart);const traces=p.series.map((s,i)=>({x:s.x,y:s.y,name:s.name,type:s.style==='bars'?'bar':'scatter',mode:s.style==='points'?'markers':'lines',yaxis:s.secondary?'y2':'y',line:{color:s.color||colors[i%colors.length],width:1.4,dash:s.dashed?'dash':'solid',shape:s.style==='step'?'hv':'linear'},marker:{color:s.color||colors[i%colors.length]}}));
const shapes=[],annotations=[];for(const m of p.markers){const common={line:{color:m.color,dash:'dot',width:1},xref:'x',yref:'paper'};if(m.kind==='horizontal')shapes.push({type:'line',xref:'paper',yref:'y',x0:0,x1:1,y0:m.first,y1:m.first,line:common.line});else if(m.kind==='band')shapes.push({...common,type:'rect',x0:m.first,x1:m.second,y0:0,y1:1,line:{width:0},fillcolor:m.color,opacity:.18,layer:'below'});else{shapes.push({...common,type:'line',x0:m.first,x1:m.first,y0:0,y1:1});if(m.label)annotations.push({xref:'x',yref:'paper',x:m.first,y:1,text:m.label,showarrow:false,textangle:-90,font:{color:m.color,size:9},xanchor:'left',yanchor:'top'});}}
window.Plotly.newPlot(chart,traces,{paper_bgcolor:'#1a1e24',plot_bgcolor:'#22272e',font:{color:'#e6edf3'},hovermode:'x unified',height:360,margin:{l:70,r:70,t:24,b:65},xaxis:{title:{text:p.xlabel},range:p.xrange,gridcolor:'#484f58'},yaxis:{title:{text:p.ylabel},type:p.log?'log':'linear',range:p.yrange? (p.log?p.yrange.map(Math.log10):p.yrange):undefined,gridcolor:'#484f58',scaleanchor:p.equal_axes?'x':undefined,scaleratio:p.equal_axes?1:undefined},yaxis2:{title:{text:p.secondary_label},overlaying:'y',side:'right'},shapes,annotations},{responsive:true,displaylogo:false});continue;}
const svg=elem('svg',{viewBox:'0 0 1000 350',role:'img','aria-label':p.title,tabindex:0});box.append(svg);const legend=document.createElement('div');legend.className='legend';box.append(legend);const tooltip=document.createElement('pre');tooltip.className='tooltip';box.append(tooltip);
const base=panelDomain(p),right=panelDomain(p,true);if(p.equal_axes){const xs=(base[1]-base[0])/850,ys=(base[3]-base[2])/250;if(xs<ys){const c=(base[0]+base[1])/2,h=ys*850/2;base[0]=c-h;base[1]=c+h;}else{const c=(base[2]+base[3])/2,h=xs*250/2;base[2]=c-h;base[3]=c+h;}}let range=base.slice(),drag=null,hidden=new Set();const clip='clip-'+(++uid);
const sx=x=>70+(x-range[0])/(range[1]-range[0])*850;
const sy=(y,secondary=false)=>{const r=secondary?right:range;return 290-((p.log?Math.log10(y):y)-r[2])/(r[3]-r[2])*250;};
const draw=()=>{svg.replaceChildren();const defs=elem('defs'),cp=elem('clipPath',{id:clip});cp.append(elem('rect',{x:70,y:40,width:850,height:250}));defs.append(cp);svg.append(defs);svg.append(elem('rect',{x:70,y:40,width:850,height:250,fill:'#22272e'}));
for(let k=0;k<=5;k++){const q=k/5,x=70+850*q,y=290-250*q;svg.append(elem('path',{d:`M${x} 40V290 M70 ${y}H920`,stroke:'#484f58','stroke-width':.6,fill:'none'}));svg.append(elem('text',{x,y:309,fill:'#8b949e','font-size':11,'text-anchor':'middle'},fmt(range[0]+q*(range[1]-range[0]))));const value=range[2]+q*(range[3]-range[2]);svg.append(elem('text',{x:64,y:y+4,fill:'#8b949e','font-size':11,'text-anchor':'end'},fmt(p.log?10**value:value)));if(p.secondary_label)svg.append(elem('text',{x:928,y:y+4,fill:'#8b949e','font-size':11},fmt(right[2]+q*(right[3]-right[2]))));}
svg.append(elem('text',{x:70,y:22,fill:'#8b949e','font-size':12},p.ylabel));svg.append(elem('text',{x:920,y:22,fill:'#8b949e','font-size':12,'text-anchor':'end'},p.secondary_label||''));svg.append(elem('text',{x:495,y:340,fill:'#8b949e','font-size':12,'text-anchor':'middle'},p.xlabel));
const content=elem('g',{'clip-path':`url(#${clip})`});svg.append(content);for(const m of p.markers){if(m.kind==='band')content.append(elem('rect',{x:sx(m.first),y:40,width:Math.max(0,sx(m.second)-sx(m.first)),height:250,fill:m.color,opacity:.18}));else if(m.kind==='horizontal')content.append(elem('path',{d:`M70 ${sy(m.first)}H920`,stroke:m.color,'stroke-dasharray':'4 4'}));else{const x=sx(m.first);content.append(elem('path',{d:`M${x} 40V290`,stroke:m.color,'stroke-dasharray':'3 4'}));if(m.label)content.append(elem('text',{x:x+4,y:52,fill:m.color,'font-size':10,transform:`rotate(90 ${x+4} 52)`},m.label));}}
p.series.forEach((s,idx)=>{if(hidden.has(idx))return;const color=s.color||colors[idx%colors.length];let path='',active=false,previousY=0;for(let i=0;i<s.x.length;i++){const x=s.x[i],y=s.y[i];if(!finite(x)||!finite(y)||(p.log&&y<=0)){active=false;continue;}const px=sx(x),py=sy(y,s.secondary);if(s.style==='bars'){const w=Math.max(1,850/Math.max(s.x.length,1)*.8),y0=p.log?290:sy(0,s.secondary);content.append(elem('rect',{x:px-w/2,y:Math.min(py,y0),width:w,height:Math.max(1,Math.abs(py-y0)),fill:color}));}else if(s.style==='points')content.append(elem('circle',{cx:px,cy:py,r:2.5,fill:color}));else{if(active&&s.style==='step')path+=`L${px},${previousY}`;path+=`${active?'L':'M'}${px},${py}`;active=true;previousY=py;}}if(path)content.append(elem('path',{d:path,fill:'none',stroke:color,'stroke-width':1.4,'stroke-dasharray':s.dashed?'6 4':'none','vector-effect':'non-scaling-stroke'}));});};
p.series.forEach((s,i)=>{const b=document.createElement('button');b.textContent=s.name;b.style.color=s.color||colors[i%colors.length];b.onclick=()=>{hidden.has(i)?hidden.delete(i):hidden.add(i);b.style.opacity=hidden.has(i)?'.35':'1';draw();};legend.append(b);});
const exportButton=document.createElement('button');exportButton.textContent='Export SVG';exportButton.onclick=()=>save(new XMLSerializer().serializeToString(svg),'image/svg+xml','plot.svg');legend.append(exportButton);
const local=e=>{const r=svg.getBoundingClientRect();return[(e.clientX-r.left)/r.width*1000,(e.clientY-r.top)/r.height*350];};
svg.addEventListener('wheel',e=>{e.preventDefault();const [px]=local(e),q=Math.max(0,Math.min(1,(px-70)/850)),center=range[0]+q*(range[1]-range[0]),factor=Math.exp(Math.max(-1,Math.min(1,e.deltaY*.001)));range[0]=center+(range[0]-center)*factor;range[1]=center+(range[1]-center)*factor;draw();},{passive:false});
svg.addEventListener('pointerdown',e=>{drag={x:local(e)[0],range:range.slice()};svg.setPointerCapture(e.pointerId);});svg.addEventListener('pointerup',()=>drag=null);svg.addEventListener('pointercancel',()=>drag=null);svg.addEventListener('dblclick',()=>{range=base.slice();draw();});
svg.addEventListener('pointermove',e=>{const [px]=local(e);if(drag){const shift=(px-drag.x)/850*(drag.range[1]-drag.range[0]);range[0]=drag.range[0]-shift;range[1]=drag.range[1]-shift;draw();}const x=range[0]+(px-70)/850*(range[1]-range[0]);const lines=[p.xlabel+': '+fmt(x)];p.series.forEach((s,i)=>{if(hidden.has(i))return;let best=-1,d=Infinity;for(let k=0;k<s.x.length;k++)if(finite(s.x[k])&&Math.abs(s.x[k]-x)<d){d=Math.abs(s.x[k]-x);best=k;}lines.push(s.name+': '+(best<0?'—':fmt(s.y[best])));});tooltip.textContent=lines.join('\n');});
draw();
}}
})();
)BIKEJS";
}
void set_plotly_resource(const std::filesystem::path& path) {
    if(path.empty())throw std::invalid_argument("Plotly resource path is empty");
    std::scoped_lock<std::mutex> const lock(resource_mutex);plotly_override=path;
}
namespace detail {
std::string report_json(const Report& report) {
    validate(report);std::string out="{\"title\":"+json_string(report.title)+",\"columns\":"+std::to_string(report.columns)+",\"panels\":[";
    const auto values=[](const std::vector<double>& array){std::string encoded="[";bool first=true;for(double const x:array){if(!first)encoded+=',';first=false;encoded+=number(x);}return encoded+"]";};
    for(std::size_t panel_index=0;panel_index<report.panels.size();++panel_index){if(panel_index)out+=',';const auto& p=report.panels[panel_index];out+="{\"title\":"+json_string(p.title)+",\"xlabel\":"+json_string(p.x_label)+",\"ylabel\":"+json_string(p.y_label)+
        ",\"secondary_label\":"+json_string(p.secondary_y_label)+",\"log\":"+(p.log_y?"true":"false")+",\"equal_axes\":"+(p.equal_axes?"true":"false")+",\"series\":[";
        for(std::size_t i=0;i<p.series.size();++i){if(i)out+=',';const auto& s=p.series[i];const auto style=s.style==SeriesStyle::bars?"bars":s.style==SeriesStyle::step?"step":s.style==SeriesStyle::points?"points":"line";
            out+="{\"name\":"+json_string(s.name)+",\"x\":"+values(s.x)+",\"y\":"+values(s.y)+",\"color\":"+json_string(color(s,i))+",\"secondary\":"+(s.secondary_y?"true":"false")+",\"dashed\":"+(s.dashed?"true":"false")+",\"style\":"+json_string(style)+"}";}
        out+="],\"markers\":[";for(std::size_t i=0;i<p.markers.size();++i){if(i)out+=',';const auto& m=p.markers[i];out+="{\"kind\":"+json_string(m.kind==MarkerKind::vertical?"vertical":m.kind==MarkerKind::horizontal?"horizontal":"band")+
            ",\"first\":"+number(m.first)+",\"second\":"+number(m.second)+",\"label\":"+json_string(m.label)+",\"color\":"+json_string(m.color)+",\"full_height\":"+(m.full_height?"true":"false")+"}";}
        out+="],\"notes\":[";for(std::size_t i=0;i<p.notes.size();++i){if(i)out+=',';out+=json_string(p.notes[i]);}out+=']';
        if(p.x_range)out+=",\"xrange\":["+number(p.x_range->first)+","+number(p.x_range->second)+"]";
        if(p.y_range)out+=",\"yrange\":["+number(p.y_range->first)+","+number(p.y_range->second)+"]";
        out+='}';}
    return out+"]}";
}
}
std::string to_html(std::span<const Report> reports) {
    if(reports.empty())throw std::invalid_argument("HTML report requires at least one figure");
    const auto plotly=plotly_script();
    std::string data="[";for(std::size_t i=0;i<reports.size();++i){if(i)data+=',';data+=detail::report_json(reports[i]);}data+=']';
    std::string out=R"(<!DOCTYPE html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="bike-plot-backend" content=")"+
        std::string(plotly?"plotly-local":"native-html-v1")+"\"><title>"+detail::escape_xml(reports.front().title)+"</title>"
        "<style>body{margin:0;background:#1a1e24;color:#e6edf3;font:14px system-ui,sans-serif}main{max-width:1800px;margin:auto;padding:20px}h2{margin-top:30px}h3{margin:12px 0}article{min-width:0;background:#1a1e24;padding:12px;border:1px solid #444c56;border-radius:8px}svg{width:100%;height:auto;display:block;touch-action:none;user-select:none}.panels{display:grid;gap:14px}.toolbar,.legend{display:flex;flex-wrap:wrap;align-items:center;gap:8px;margin:10px 0}button{background:#22272e;color:#e6edf3;border:1px solid #484f58;padding:6px 10px;border-radius:5px;cursor:pointer}.tooltip{min-height:2em;max-height:140px;overflow:auto;color:#8b949e;font-size:12px}.note{color:#8b949e}.plotly-chart{min-height:360px}@media(max-width:800px){.panels{grid-template-columns:1fr!important}main{padding:8px}}</style></head><body><main><div id=\"reports\"></div><noscript>";
    for(const auto& report:reports)out+=to_svg(report);
    out+=R"(</noscript></main><script id="bike-plot-data" type="application/json">)"+data+"</script>";
    if(plotly)out+="<script>"+*plotly+"</script>";
    out+="<script>";out+=script;out+="</script></body></html>";return out;
}
std::string to_html(const Report& report){const std::array reports{report};return to_html(reports);}
std::string to_html(const Plot& plot){return to_html(Report{.title=plot.title,.panels={plot},.columns=1,.width=1280,.height=720});}
} // namespace bike::viz
