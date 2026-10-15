// files.cpp — POSIX implementation of the bounded filesystem primitives.
// Every descriptor is owned by FileHandle immediately; path checks validate
// the opened object (fstat), never a pre-open existence/type probe.
#include "platform/files.hpp"

#include "io/hash.hpp"

#include <array>
#include <cerrno>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

[[noreturn]] void fail_errno(std::string_view what, const int error) {
    throw std::system_error(std::error_code(error, std::generic_category()),
                            std::string(what));
}

void require_path(const std::filesystem::path &path, std::string_view what) {
    const std::string &native = path.native();
    if (native.empty())
        throw std::invalid_argument(std::string(what) + ": path is empty");
    // A NUL byte silently truncates the C path open() sees — reject the
    // input rather than let it name a different object than requested.
    if (native.find('\0') != std::string::npos)
        throw std::invalid_argument(std::string(what) +
                                    ": path contains a NUL byte");
}

void require_regular(const bike::platform::FileHandle &handle,
                     std::string_view what,
                     const std::filesystem::path &path) {
    // The opened object is validated — not a pre-open stat — so a race or a
    // changed name cannot substitute another kind of filesystem object.
    struct stat info {};
    if (::fstat(handle.get(), &info) != 0) {
        const int error = errno;
        fail_errno(std::string(what) + ": fstat " + path.native(), error);
    }
    if (!S_ISREG(info.st_mode))
        throw std::runtime_error(std::string(what) +
                                 ": not a regular file: " + path.native());
}

std::string hex_digest(const std::array<std::uint8_t, 32> &digest) {
    constexpr std::string_view digits = "0123456789abcdef";
    std::string out;
    out.reserve(64);
    for (const std::uint8_t byte : digest) {
        out += digits[byte >> 4U];
        out += digits[byte & 0x0fU];
    }
    return out;
}

} // namespace

namespace bike::platform {

FileHandle::FileHandle(const int descriptor) noexcept : fd_(descriptor) {}

FileHandle::~FileHandle() {
    if (fd_ >= 0) (void) ::close(fd_);
}

FileHandle::FileHandle(FileHandle &&other) noexcept : fd_(other.fd_) {
    other.fd_ = -1;
}

FileHandle &FileHandle::operator=(FileHandle &&other) noexcept {
    if (this != &other) {
        if (fd_ >= 0) (void) ::close(fd_);
        fd_ = other.fd_;
        other.fd_ = -1;
    }
    return *this;
}

int FileHandle::get() const noexcept { return fd_; }

bool FileHandle::valid() const noexcept { return fd_ >= 0; }

int FileHandle::release() noexcept {
    const int fd = fd_;
    fd_ = -1;
    return fd;
}

void FileHandle::close() {
    const int fd = release();
    if (fd < 0) return;
    if (::close(fd) != 0) {
        const int error = errno;
        fail_errno("platform::FileHandle::close", error);
    }
}

FileHandle open_exclusive(const std::filesystem::path &path) {
    require_path(path, "platform::open_exclusive");
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg) POSIX open is a vararg interface; the mode argument is required under O_CREAT
    FileHandle handle{::open(path.c_str(),
                             O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC,
                             0666)};
    if (!handle.valid()) {
        const int error = errno;
        fail_errno("platform::open_exclusive: " + path.native(), error);
    }
    require_regular(handle, "platform::open_exclusive", path);
    return handle;
}

FileHandle open_regular(const std::filesystem::path &path) {
    require_path(path, "platform::open_regular");
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg) POSIX open is declared vararg even without a mode argument
    FileHandle handle{::open(path.c_str(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC)};
    if (!handle.valid()) {
        const int error = errno;
        fail_errno("platform::open_regular: " + path.native(), error);
    }
    require_regular(handle, "platform::open_regular", path);
    return handle;
}

void write_all(const FileHandle &handle, const std::span<const std::byte> data) {
    if (!handle.valid())
        throw std::invalid_argument("platform::write_all: invalid handle");
    std::size_t offset = 0;
    while (offset < data.size()) {
        const ssize_t written =
            ::write(handle.get(), data.subspan(offset).data(),
                    data.size() - offset);
        if (written < 0) {
            const int error = errno;
            if (error == EINTR) continue;
            fail_errno("platform::write_all", error);
        }
        if (written == 0)
            throw std::runtime_error(
                "platform::write_all: zero-progress write");
        offset += static_cast<std::size_t>(written);
    }
}

void sync(const FileHandle &handle) {
    if (!handle.valid())
        throw std::invalid_argument("platform::sync: invalid handle");
    if (::fsync(handle.get()) != 0) {
        const int error = errno;
        fail_errno("platform::sync", error);
    }
}

void sync_directory(const std::filesystem::path &path) {
    require_path(path, "platform::sync_directory");
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg) POSIX open is declared vararg even without a mode argument
    const FileHandle handle{::open(path.c_str(), O_RDONLY | O_CLOEXEC)};
    if (!handle.valid()) {
        const int error = errno;
        fail_errno("platform::sync_directory: " + path.native(), error);
    }
    if (::fsync(handle.get()) != 0) {
        const int error = errno;
        fail_errno("platform::sync_directory: " + path.native(), error);
    }
}

void write_exclusive(const std::filesystem::path &path,
                     const std::span<const std::byte> data) {
    FileHandle handle = open_exclusive(path);
    write_all(handle, data);
    sync(handle);
    // Explicit close: a delayed write error can surface at close() — it is a
    // write failure here, not a swallowed teardown detail.
    handle.close();
}

void create_directory(const std::filesystem::path &path) {
    require_path(path, "platform::create_directory");
    if (::mkdir(path.c_str(), 0777) != 0) {
        const int error = errno;
        fail_errno("platform::create_directory: " + path.native(), error);
    }
}

void link_new(const std::filesystem::path &source,
              const std::filesystem::path &destination) {
    require_path(source, "platform::link_new source");
    require_path(destination, "platform::link_new destination");
    if (::link(source.c_str(), destination.c_str()) != 0) {
        const int error = errno;
        fail_errno("platform::link_new: " + source.native() + " -> " +
                       destination.native(),
                   error);
    }
}

std::string file_sha256(const std::filesystem::path &path) {
    const FileHandle handle = open_regular(path);
    io::Sha256 hash;
    std::array<char, 4096> chunk {};
    while (true) {
        const ssize_t got = ::read(handle.get(), chunk.data(), chunk.size());
        if (got < 0) {
            const int error = errno;
            if (error == EINTR) continue;
            fail_errno("platform::file_sha256: " + path.native(), error);
        }
        if (got == 0) break;
        hash.update(std::as_bytes(
            std::span(chunk).first(static_cast<std::size_t>(got))));
    }
    return hex_digest(hash.finish());
}

} // namespace bike::platform
