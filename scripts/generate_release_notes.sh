#!/usr/bin/env bash
set -euo pipefail

# Generate Markdown release notes for a px4-ros2-interface-lib version, to stdout.
#
# Usage:
#   scripts/generate_release_notes.sh [<tag>] [<base-ref>]
#
# <tag>      defaults to v<version> read from px4_ros2_cpp/package.xml.
# <base-ref> defaults to the previous v<major.minor.patch> tag.
#
# Notes are derived from the Conventional-Commit history between the two refs,
# grouped by type, with a header stating the PX4 / px4_msgs compatibility line
# (versions track the PX4 Autopilot release line).

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${SCRIPT_DIR}/.."

version_from_manifest() {
  sed -n 's:.*<version>\(.*\)</version>.*:\1:p' px4_ros2_cpp/package.xml | head -n1
}

previous_tag() {
  # The v-tag immediately preceding $1 in version order (empty if none).
  git tag --list 'v[0-9]*.[0-9]*.[0-9]*' --sort=v:refname \
    | awk -v cur="$1" '$0==cur{print prev; exit} {prev=$0}'
}

TAG="${1:-v$(version_from_manifest)}"
PXVER="${TAG#v}"
BASE="${2:-$(previous_tag "${TAG}")}"

echo "Message compatibility: this release tracks **PX4 Autopilot ${PXVER}** and the matching **px4_msgs ${PXVER}** message set."
echo

if [ -z "${BASE}" ] || ! git rev-parse "${BASE}" >/dev/null 2>&1; then
  echo "Initial tracked release of the PX4 ROS 2 Interface Library on the ${PXVER} line."
  exit 0
fi

RANGE="${BASE}..${TAG}"
if ! git rev-parse "${TAG}" >/dev/null 2>&1; then
  # The tag may not exist yet at notes-generation time; use HEAD instead.
  RANGE="${BASE}..HEAD"
fi

echo "Changes since [\`${BASE}\`](../../releases/tag/${BASE}):"
echo

emit_section() {
  local title="$1" pattern="$2"
  local lines
  lines="$(git log "${RANGE}" --no-merges --format='%s' | grep -E "${pattern}" || true)"
  if [ -n "${lines}" ]; then
    echo "### ${title}"
    echo "${lines}" | sed -E 's/^[a-z]+(\([^)]*\))?(!)?: /- /' | sort -u
    echo
  fi
}

emit_section "Features" '^feat(\(|!|:)'
emit_section "Fixes" '^fix(\(|!|:)'
emit_section "Documentation" '^docs(\(|!|:)'
emit_section "Build, CI and packaging" '^(build|ci|chore)(\(|!|:)'

OTHER="$(git log "${RANGE}" --no-merges --format='%s' \
  | grep -Ev '^(feat|fix|docs|build|ci|chore)(\(|!|:)' || true)"
if [ -n "${OTHER}" ]; then
  echo "### Other"
  echo "${OTHER}" | sed -E 's/^/- /' | sort -u
  echo
fi

echo "### Contributors"
git log "${RANGE}" --format='%an' | sort -u | sed '/^$/d' | paste -sd ', ' -
