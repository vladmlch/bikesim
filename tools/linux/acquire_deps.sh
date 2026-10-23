#!/usr/bin/env bash
# Explicit pinned image/dependency acquisition. Never called by CMake, CTest,
# configure, build or install. Archives are inert input: no fetched code runs.
# Usage: acquire_deps.sh LOCK DESTINATION {linux-aarch64|darwin-arm64}
set -euo pipefail

die() { printf 'dependency acquisition: %s\n' "$*" >&2; exit 1; }
[[ $# -eq 3 ]] || die 'usage: acquire_deps.sh LOCK DESTINATION {linux-aarch64|darwin-arm64}'
lock_file=$(cd -- "$(dirname -- "$1")" && printf '%s/%s\n' "$PWD" "$(basename -- "$1")")
mkdir -p -- "$2"
dependency_destination=$(cd -- "$2" && pwd -P)
platform_key=$3
[[ $platform_key == linux-aarch64 || $platform_key == darwin-arm64 ]] || die "unsupported acquisition platform: $platform_key"
[[ -f $lock_file ]] || die "missing lock: $lock_file"
for tool in jq curl unzip tar awk; do
    command -v "$tool" >/dev/null || die "required acquisition tool is missing: $tool"
done
if command -v sha256sum >/dev/null; then
    sha_tool=sha256sum
    sha_args=(--check --strict -)
else
    command -v shasum >/dev/null || die 'required SHA-256 tool is missing'
    sha_tool=shasum
    sha_args=(-a 256 --check -)
fi
jq -e '.schema_version == 2 and (.dependencies | type == "object" and length > 0)' \
    "$lock_file" >/dev/null || die 'expected a nonempty schema-2 dependency lock'
mkdir -p -- "$dependency_destination/_archives"
acquisition_work=$(mktemp -d "$dependency_destination/.acquire.XXXXXXXX")
trap 'rm -rf -- "$acquisition_work"' EXIT
jq -er '.dependencies | keys[]' "$lock_file" > "$acquisition_work/dependencies.list"

check_relative() {
    [[ $1 != /* && $1 != *\\* && $1 != *$'\n'* && $1 != *$'\r'* && $1 != *$'\t'* ]] \
        || die "unsafe relative lock path: $1"
    case /$1/ in */../*|*/./*|*//*) die "unsafe relative lock path: $1" ;; esac
}

verify_sha() {
    local expected=$1 path=$2
    [[ $expected =~ ^[0-9a-f]{64}$ ]] || die "invalid SHA-256 for $path"
    printf '%s  %s\n' "$expected" "$path" | "$sha_tool" "${sha_args[@]}"
}

while IFS= read -r dependency_name; do
    # Only a literal null skips a dependency. Missing platform keys are errors.
    selected_entry=$(jq -c --arg name "$dependency_name" --arg key "$platform_key" '
        .dependencies[$name] as $entry |
        if ($entry | has("platforms")) then
            if ($entry.platforms | type) != "object" then error("platforms must be an object")
            elif ($entry.platforms | has($key) | not) then error("dependency " + $name + " has no lock entry for platform " + $key)
            elif $entry.platforms[$key] == null then null
            elif ($entry.platforms[$key] | type) != "object" then error("platform entry must be an object or null")
            else ($entry | del(.platforms)) +
                 ($entry.platforms[$key] | {acquisition, required_files}) end
        else $entry end' "$lock_file")
    [[ $selected_entry != null ]] || continue
    printf '%s\n' "$selected_entry" > "$acquisition_work/entry.json"
    entry_file=$acquisition_work/entry.json
    jq -e '(.required_files | type == "object" and length > 0) and
           (.license_files | type == "array" and length > 0)' \
        "$entry_file" >/dev/null || die "missing required files or licenses for $dependency_name"
    # Check jq's status before entering a loop. A process substitution would
    # hide a failed parser/enumeration behind the reader loop's exit status.
    jq -er '.required_files | to_entries[] | [.key, .value] | @tsv' \
        "$entry_file" > "$acquisition_work/required-files.tsv"
    jq -er '.license_files[]' "$entry_file" > "$acquisition_work/licenses.list"
    # Assert a nonempty .version without keeping the value (SC2034).
    jq -er '.version | select(type == "string" and length > 0)' "$entry_file" >/dev/null
    dependency_root=$(jq -er '.extracted_root | select(type == "string" and length > 0)' "$entry_file")
    archive_relative=$(jq -er '.acquisition.archive | select(type == "string" and length > 0)' "$entry_file")
    archive_url=$(jq -er '.acquisition.url | select(type == "string" and startswith("https://"))' "$entry_file")
    archive_sha=$(jq -er '.acquisition.archive_sha256' "$entry_file")
    check_relative "$dependency_root"
    check_relative "$archive_relative"
    [[ $dependency_root != */* && $archive_relative == _archives/* ]] \
        || die "unsupported lock layout for $dependency_name"
    [[ ! -e $dependency_destination/$dependency_root && ! -L $dependency_destination/$dependency_root ]] \
        || die "destination already contains $dependency_root; use a fresh destination"
    archive_path=$dependency_destination/$archive_relative
    mkdir -p -- "$(dirname -- "$archive_path")"
    if [[ ! -f $archive_path ]]; then
        curl --proto '=https' --proto-redir '=https' --tlsv1.2 -fLsS --retry 3 \
            "$archive_url" -o "$acquisition_work/download"
        verify_sha "$archive_sha" "$acquisition_work/download"
        mv -- "$acquisition_work/download" "$archive_path"
    else
        verify_sha "$archive_sha" "$archive_path"
    fi

done < "$acquisition_work/dependencies.list"

# Both image acquisition and offline source-package rebuild use this exact
# extraction path. The offline helper cannot download missing archives.
bash "$(dirname -- "${BASH_SOURCE[0]}")/extract_deps.sh" \
    "$lock_file" "$dependency_destination" "$platform_key"
