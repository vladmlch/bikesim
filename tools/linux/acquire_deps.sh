#!/usr/bin/env bash
# Explicit Linux image/dependency acquisition. Never called by CMake, CTest,
# configure, build or install. Archives are inert input: no fetched code runs.
# Usage: acquire_deps.sh LOCK DESTINATION linux-aarch64
set -euo pipefail

die() { printf 'dependency acquisition: %s\n' "$*" >&2; exit 1; }
[[ $# -eq 3 ]] || die 'usage: acquire_deps.sh LOCK DESTINATION linux-aarch64'
lock_file=$(realpath -- "$1")
dependency_destination=$(realpath -m -- "$2")
platform_key=$3
[[ $platform_key == linux-aarch64 ]] || die "unsupported acquisition platform: $platform_key"
[[ -f $lock_file ]] || die "missing lock: $lock_file"
for tool in jq curl sha256sum unzip tar awk; do
    command -v "$tool" >/dev/null || die "required acquisition tool is missing: $tool"
done
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
    printf '%s  %s\n' "$expected" "$path" | sha256sum --check --strict -
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
    version=$(jq -er '.version | select(type == "string" and length > 0)' "$entry_file")
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

    # Extraction starts only after the archive passes its pinned SHA-256.
    # Each archive is handled by its audited upstream layout, never by a command
    # read from the lock. No wheel is installed or imported.
    unpacked=$acquisition_work/unpacked
    staged_root=$acquisition_work/staged
    mkdir -p -- "$unpacked" "$staged_root"
    case $dependency_name in
        mujoco)
            unzip -q "$archive_path" 'mujoco/include/*' \
                "mujoco/libmujoco.so.$version" \
                "mujoco-$version.dist-info/licenses/*" -d "$unpacked"
            mkdir -p -- "$staged_root/lib"
            cp -a -- "$unpacked/mujoco/include" "$staged_root/include"
            cp -- "$unpacked/mujoco/libmujoco.so.$version" "$staged_root/lib/"
            cp -- "$unpacked/mujoco-$version.dist-info/licenses/LICENSE" \
                "$unpacked/mujoco-$version.dist-info/licenses/LICENSES_THIRD_PARTY.md" "$staged_root/"
            ;;
        duckdb)
            unzip -q "$archive_path" duckdb.h duckdb.hpp libduckdb.so -d "$unpacked"
            mkdir -p -- "$staged_root/include" "$staged_root/lib"
            cp -- "$unpacked/duckdb.h" "$unpacked/duckdb.hpp" "$staged_root/include/"
            cp -- "$unpacked/libduckdb.so" "$staged_root/lib/"
            # The release zip carries its complete MIT notice as the initial
            # block in the hash-pinned amalgamated header, not a LICENSE member.
            awk 'NR == 1 { if ($0 != "/*") exit 1; next }
                 /^\*\/$/ { complete = 1; exit }
                 { print }
                 END { if (!complete) exit 1 }' "$unpacked/duckdb.hpp" > "$staged_root/LICENSE"
            ;;
        numpy_openblas)
            jq -er '.required_files | keys[] | select(startswith("lib/"))' \
                "$entry_file" > "$acquisition_work/numpy-libraries.list"
            unzip -q "$archive_path" 'numpy.libs/*' \
                "numpy-$version.dist-info/licenses/LICENSE.txt" -d "$unpacked"
            mkdir -p -- "$staged_root/lib"
            while IFS= read -r library_member; do
                check_relative "$library_member"
                [[ $library_member == lib/* && ${library_member#lib/} != */* ]] \
                    || die "unsupported NumPy provider member: $library_member"
                cp -- "$unpacked/numpy.libs/${library_member#lib/}" "$staged_root/$library_member"
            done < "$acquisition_work/numpy-libraries.list"
            cp -- "$unpacked/numpy-$version.dist-info/licenses/LICENSE.txt" "$staged_root/LICENSE.txt"
            ;;
        glfw)
            unzip -q "$archive_path" -d "$unpacked"
            # Preserve all CMake inputs and platform source files, not headers
            # alone. Wayland remains unselected by the Linux CMake policy.
            cp -a -- "$unpacked/glfw-$version/." "$staged_root/"
            ;;
        tomlplusplus|rapidjson|cairo|eigen)
            tar --extract --file "$archive_path" --directory "$staged_root" --strip-components=1
            if [[ $dependency_name == rapidjson ]]; then
                # This pinned upstream commit calls it license.txt; preserve
                # that file and map a copy to the lock's common LICENSE path.
                cp -- "$staged_root/license.txt" "$staged_root/LICENSE"
            fi
            ;;
        *) die "no reviewed extraction layout for dependency: $dependency_name" ;;
    esac
    printf '%s\n' "$version" > "$staged_root/BIKE_VERSION"
    while IFS=$'\t' read -r member expected; do
        check_relative "$member"
        [[ -f $staged_root/$member ]] || die "required file missing: $dependency_name/$member"
        verify_sha "$expected" "$staged_root/$member"
    done < "$acquisition_work/required-files.tsv"
    while IFS= read -r license_member; do
        check_relative "$license_member"
        [[ $license_member == "$dependency_root/"* ]] || die "license outside dependency root: $license_member"
        [[ -f $staged_root/${license_member#"$dependency_root/"} ]] || die "missing license: $license_member"
    done < "$acquisition_work/licenses.list"
    mv -- "$staged_root" "$dependency_destination/$dependency_root"
    rm -rf -- "$unpacked"
    printf 'dependency %s %s acquired and sha256 verified\n' "$dependency_name" "$version"
done < "$acquisition_work/dependencies.list"
