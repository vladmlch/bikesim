// files.hpp — bounded filesystem primitives (task T06) behind the
// exclusive-publication contract of src/bike_sim/recording/storage.py:
// exclusive-create staging/destination objects, validation of the OPENED
// object (never a pre-open stat), no-follow regular-file checks, hashing
// through a separately opened read stream after close, and directory
// synchronization. Existing destinations — including empty ones — are
// never adopted or silently replaced; callers that retry compose a fresh
// exclusive name exactly like the source's attempt loop.
//
// Error classes: input-domain violations (empty path, embedded NUL byte,
// invalid handle) throw std::invalid_argument; OS-level failures throw
// std::system_error carrying the errno category; a verified-opened-object
// type mismatch throws std::runtime_error. POSIX only (macOS/Linux).
#pragma once

#include <cstddef>
#include <filesystem>
#include <span>
#include <string>

namespace bike::platform {

// RAII owner for one OS file handle. Move-only; the destructor never throws
// and silently closes, while close() is the explicit path where a close
// failure must surface as an error (publication writes use it).
class FileHandle {
  public:
    FileHandle() noexcept = default;
    explicit FileHandle(int descriptor) noexcept;
    ~FileHandle();
    FileHandle(FileHandle &&other) noexcept;
    FileHandle &operator=(FileHandle &&other) noexcept;
    FileHandle(const FileHandle &) = delete;
    FileHandle &operator=(const FileHandle &) = delete;

    [[nodiscard]] int get() const noexcept;
    [[nodiscard]] bool valid() const noexcept;
    // Releases ownership without closing; the caller now owns the descriptor.
    [[nodiscard]] int release() noexcept;
    // Explicit close: a descriptor-level close failure throws
    // std::system_error. No-op on an invalid handle.
    void close();

  private:
    int fd_ = -1;
};

// Exclusive creation: O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW — an existing name
// of ANY kind (file, empty file, symlink, directory) fails instead of being
// adopted. The opened object is fstat-validated as a regular file.
[[nodiscard]] FileHandle open_exclusive(const std::filesystem::path &path);

// No-follow read: O_RDONLY|O_NOFOLLOW — a leaf symlink fails with ELOOP and
// the opened object must be a regular file (character devices, fifos and
// directories are rejected after open, not by a pre-open stat).
[[nodiscard]] FileHandle open_regular(const std::filesystem::path &path);

// Full write: loops over partial writes and EINTR until data is committed;
// a zero-progress write or an error throws.
void write_all(const FileHandle &handle, std::span<const std::byte> data);

// fsync the opened object (the storage.py _sync_file payload step).
void sync(const FileHandle &handle);

// fsync a directory so the names created inside it are durable.
void sync_directory(const std::filesystem::path &path);

// The produced exclusive-write boundary: exclusive create + full write +
// fsync + explicit close. A pre-existing destination always throws — the
// source publication never adopts it.
void write_exclusive(const std::filesystem::path &path,
                     std::span<const std::byte> data);

// Exclusive directory creation (the source's destination.mkdir()): creates
// exactly the named directory — no parent creation, EEXIST on any existing
// name including an empty directory.
void create_directory(const std::filesystem::path &path);

// Exclusive hard link (the source's os.link publication step): fails with
// EEXIST when destination already names anything.
void link_new(const std::filesystem::path &source,
              const std::filesystem::path &destination);

// SHA-256 of a file's bytes through a fresh no-follow, regular-file-checked
// read stream — the hash-after-checkpoint/close step. Bounded chunks; the
// whole file is never resident.
[[nodiscard]] std::string file_sha256(const std::filesystem::path &path);

} // namespace bike::platform
