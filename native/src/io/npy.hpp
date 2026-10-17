#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bike::io {
inline constexpr std::size_t kMaxNumpyBytes=128U*1024U*1024U;
inline constexpr std::size_t kMaxNumpyMembers=4096;
struct NpyArray {
    std::string dtype;
    std::vector<std::size_t> shape;
    bool fortran_order=false;
    std::vector<std::byte> bytes;
    [[nodiscard]] std::size_t elements() const;
    [[nodiscard]] std::size_t item_size() const;
    void validate() const;
    [[nodiscard]] std::vector<double> doubles(bool allow_nonfinite=false) const;
    [[nodiscard]] std::vector<std::int64_t> integers() const;
    [[nodiscard]] std::vector<bool> booleans() const;
    [[nodiscard]] std::vector<std::string> strings() const;
};
using NpzArchive=std::vector<std::pair<std::string,NpyArray>>;
[[nodiscard]] NpyArray parse_npy(std::span<const std::byte>);
[[nodiscard]] std::vector<std::byte> encode_npy(const NpyArray&);
[[nodiscard]] NpzArchive parse_npz(std::span<const std::byte>);
[[nodiscard]] std::vector<std::byte> read_numpy_bytes(const std::filesystem::path&);
[[nodiscard]] NpyArray double_array(std::span<const double>,std::vector<std::size_t> shape);
[[nodiscard]] NpyArray integer_array(std::span<const std::int64_t>,std::vector<std::size_t> shape);
[[nodiscard]] NpyArray bool_array(const std::vector<bool>&,std::vector<std::size_t> shape);
[[nodiscard]] NpyArray string_array(std::span<const std::string>,std::vector<std::size_t> shape);
} // namespace bike::io
