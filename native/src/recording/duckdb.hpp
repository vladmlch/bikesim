#pragma once

#include "../io/value.hpp"
#include "../platform/files.hpp"
#include <duckdb.h>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace bike::recording {

// Generic cold-path database ownership. No engine, model, policy or graphics
// headers belong here. Cursor keeps a streaming result and one DuckDB chunk;
// its owning Connection must outlive it. Both are single-thread confined.
class Cursor {
public:
    ~Cursor();
    Cursor(Cursor&&) noexcept;
    Cursor& operator=(Cursor&&) noexcept;
    Cursor(const Cursor&) = delete;
    Cursor& operator=(const Cursor&) = delete;
    [[nodiscard]] const std::vector<std::string>& columns() const;
    [[nodiscard]] std::vector<io::Value::Object> next_batch(std::size_t maximum = 4096);
private:
    struct Impl;
    explicit Cursor(std::unique_ptr<Impl>);
    std::unique_ptr<Impl> impl_;
    friend class Connection;
};

class Connection {
public:
    ~Connection();
    Connection(const Connection&) = delete;
    Connection& operator=(const Connection&) = delete;
    [[nodiscard]] duckdb_connection handle() const noexcept { return connection_; }
    void execute(std::string_view sql, std::span<const io::Value> parameters = {}) const;
    [[nodiscard]] Cursor stream(std::string_view sql, std::span<const io::Value> parameters = {},
                                bool select_only = false) const;
    [[nodiscard]] std::vector<io::Value::Object> query(
        std::string_view sql, std::span<const io::Value> parameters = {}) const;
    void require_select(std::string_view sql) const;
private:
    explicit Connection(duckdb_database);
    duckdb_connection connection_ = nullptr;
    friend class Database;
};

class Database {
public:
    explicit Database(const std::filesystem::path&, bool read_only = false);
    ~Database();
    Database(const Database&) = delete;
    Database& operator=(const Database&) = delete;
    [[nodiscard]] Connection& connection() const noexcept { return *connection_; }
    // Hash the validated read-only inode used by DuckDB, without reopening
    // its original pathname. Writer connections have no input descriptor.
    [[nodiscard]] std::string opened_sha256() const;
private:
    platform::FileHandle opened_;
    duckdb_database database_ = nullptr;
    std::unique_ptr<Connection> connection_;
};

} // namespace bike::recording
