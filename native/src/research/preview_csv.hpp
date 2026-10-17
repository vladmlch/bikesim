#pragma once
#include "../runtime/samples.hpp"
#include <filesystem>
#include <fstream>
#include <iosfwd>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace bike::research {
// Optional live preview. It consumes accounted scalar rows and owns no
// physical or research history. Database publication has a different owner.
class PhysicalLiveCsv {
public:
    explicit PhysicalLiveCsv(const std::filesystem::path&, double interval_s = .01, int flush_every = 25);
    // Injection seam for a caller-owned stream; close() flushes but does not
    // destroy or close that stream. Its lifetime must exceed this preview.
    explicit PhysicalLiveCsv(std::ostream&, double interval_s = .01, int flush_every = 25);
    ~PhysicalLiveCsv();
    PhysicalLiveCsv(const PhysicalLiveCsv&) = delete;
    PhysicalLiveCsv& operator=(const PhysicalLiveCsv&) = delete;
    [[nodiscard]] bool due(double time_s, std::int64_t generation) const;
    void write(double time_s, std::int64_t generation, const runtime::WireObject& scalar_row);
    void write_marker(std::string marker);
    void close();
    [[nodiscard]] const std::vector<std::string>& cleanup_notes() const noexcept { return cleanup_notes_; }
private:
    void emit(const runtime::WireObject&, bool flush);
    void require_open() const;
    std::unique_ptr<std::ofstream> file_;
    std::ostream* stream_ = nullptr;
    double interval_s_, next_time_s_ = 0.;
    int flush_every_, rows_since_flush_ = 0;
    std::optional<std::int64_t> generation_;
    std::vector<std::string> columns_, cleanup_notes_;
    bool closed_ = false, failed_ = false;
};
using PhysicalPreviewCsv = PhysicalLiveCsv;
} // namespace bike::research
