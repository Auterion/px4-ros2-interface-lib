#!/usr/bin/env bash
set -euo pipefail

# Regenerate a package CHANGELOG.rst in the catkin_pkg/bloom RST format from the
# git tag history. Versions track the PX4 Autopilot release line.
#
# Usage:
#   scripts/generate_changelog.sh <package-dir> [--in-place|--check]
#
#   <package-dir>  px4_ros2_cpp or px4_ros2_py
#   (no mode)      print the changelog to stdout
#   --in-place     overwrite <package-dir>/CHANGELOG.rst
#   --check        fail if <package-dir>/CHANGELOG.rst is stale
#
# Only v<major>.<minor>.<patch> tags are considered, so the historical
# pre-release-cycle tags (which used a different scheme) are ignored and the
# changelog starts cleanly on the PX4 release line.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${SCRIPT_DIR}/.."

PKG_DIR="${1:?usage: generate_changelog.sh <package-dir> [--in-place|--check]}"
MODE="${2:-stdout}"
PKG_NAME="$(basename "${PKG_DIR}")"
CHANGELOG="${PKG_DIR}/CHANGELOG.rst"

underline() { printf '%s\n%s\n' "$1" "$(printf '%*s' "${#1}" '' | tr ' ' "$2")"; }

previous_tag() {
  git tag --list 'v[0-9]*.[0-9]*.[0-9]*' --sort=v:refname \
    | awk -v cur="$1" '$0==cur{print prev; exit} {prev=$0}'
}

contributors() {
  local range="$1"
  [ -z "${range}" ] && { echo "Nuno Marques"; return; }
  local names
  names="$(git log "${range}" --format='%an' | sort -u | sed '/^$/d' | paste -sd ', ' -)"
  [ -z "${names}" ] && names="Nuno Marques"
  echo "${names}"
}

generate() {
  underline "Changelog for package ${PKG_NAME}" '^'
  echo
  echo "Versions track the PX4 Autopilot release line. Entries are produced from"
  echo "the tagged git history by scripts/generate_changelog.sh."
  echo

  local tags
  tags="$(git tag --list 'v[0-9]*.[0-9]*.[0-9]*' --sort=-v:refname)"
  if [ -z "${tags}" ]; then
    underline "Forthcoming" '-'
    echo
    echo "* (none)"
    echo
    return
  fi

  while IFS= read -r tag; do
    [ -z "${tag}" ] && continue
    local version date prev range
    version="${tag#v}"
    date="$(git log -1 --format=%ad --date=short "${tag}")"
    prev="$(previous_tag "${tag}")"
    underline "${version} (${date})" '-'
    echo
    if [ -z "${prev}" ]; then
      echo "* Initial tracked release on the PX4 Autopilot ${version} line."
      range=""
    else
      range="${prev}..${tag}"
      local subject_lines
      subject_lines="$(git log "${range}" --no-merges --format='%s' -- "${PKG_DIR}" \
        | sed -E 's/^[a-z]+(\([^)]*\))?(!)?: /* /' | sort -u || true)"
      if [ -n "${subject_lines}" ]; then
        echo "${subject_lines}"
      else
        echo "* Synchronized with PX4 Autopilot ${version}."
      fi
    fi
    echo "* Contributors: $(contributors "${range}")"
    echo
  done <<< "${tags}"
}

emit() {
  local out
  out="$(generate)"
  printf '%s\n' "${out}"
}

case "${MODE}" in
  stdout)
    emit
    ;;
  --in-place)
    emit > "${CHANGELOG}"
    echo "Wrote ${CHANGELOG}" >&2
    ;;
  --check)
    tmp="$(mktemp)"
    trap 'rm -f "${tmp}"' EXIT
    emit > "${tmp}"
    if ! diff -q "${tmp}" "${CHANGELOG}" >/dev/null 2>&1; then
      echo "${CHANGELOG} is out of date. Run: scripts/generate_changelog.sh ${PKG_DIR} --in-place" >&2
      diff -u "${CHANGELOG}" "${tmp}" || true
      exit 1
    fi
    echo "${CHANGELOG} is up to date." >&2
    ;;
  *)
    echo "Unknown mode: ${MODE}" >&2
    exit 2
    ;;
esac
