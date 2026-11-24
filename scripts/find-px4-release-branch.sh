#! /bin/bash
# Pick the PX4 release branch this repository should create next.
#
# With a branch name, check that it has the release/X.Y form and use it.
#
# Without one, compare the release/X.Y branches of PX4-Autopilot with the ones
# here and pick the oldest PX4 branch that is newer than every release branch
# this repository already has. Older PX4 branches that never existed here are
# left alone, and so is anything that is not exactly release/X.Y.
#
# The result is printed and, when running in GitHub Actions, written to
# $GITHUB_OUTPUT as `branch=<name>`. The name is empty when there is nothing
# to create.
#
# Usage: scripts/find-px4-release-branch.sh [release/X.Y]
#
# Environment:
#   PX4_URL     PX4-Autopilot remote to read the branches from
#   LIB_REMOTE  remote of this repository to compare against (default: origin)
set -euo pipefail

PX4_URL="${PX4_URL:-https://github.com/PX4/PX4-Autopilot.git}"
LIB_REMOTE="${LIB_REMOTE:-origin}"

# Print a message and hand the branch (possibly empty) to the workflow.
report() {
  echo "$2"
  if [ -n "${GITHUB_OUTPUT:-}" ]; then
    echo "branch=$1" >> "$GITHUB_OUTPUT"
  fi
  if [ -z "$1" ] && [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    echo "$2" >> "$GITHUB_STEP_SUMMARY"
  fi
}

# List the release/X.Y branches of a remote, oldest version first.
release_branches() {
  git ls-remote --heads "$1" 'refs/heads/release/*' |
    sed -n 's|^.*refs/heads/\(release/[0-9][0-9]*\.[0-9][0-9]*\)$|\1|p' |
    sort -V
}

requested="${1:-}"
if [ -n "$requested" ]; then
  if [[ ! "$requested" =~ ^release/[0-9]+\.[0-9]+$ ]]; then
    echo "::error::Branch name must match 'release/X.Y' format, got: $requested"
    exit 1
  fi
  report "$requested" "Using the requested branch $requested"
  exit 0
fi

ours="$(release_branches "$LIB_REMOTE")"
theirs="$(release_branches "$PX4_URL")"

newest="$(tail -n 1 <<< "$ours")"
if [ -z "$newest" ]; then
  report "" "No release/X.Y branch here to compare against, nothing to do"
  exit 0
fi

next=""
while read -r candidate; do
  [ -n "$candidate" ] || continue
  [ "$candidate" != "$newest" ] || continue
  if [ "$(printf '%s\n' "$newest" "$candidate" | sort -V | tail -n 1)" = "$candidate" ]; then
    next="$candidate"
    break
  fi
done <<< "$theirs"

if [ -z "$next" ]; then
  report "" "No PX4 release branch newer than $newest"
else
  report "$next" "PX4 has $next, which is newer than $newest"
fi
