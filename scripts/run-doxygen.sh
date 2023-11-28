#! /bin/bash
set -e

THIS_DIR="$(dirname "$(readlink -f "$0")")"
ROOT_DIR="$(dirname "$THIS_DIR")"

cd "$ROOT_DIR"

# Get doxygen-awesome-css
[ ! -d doxygen-awesome-css ] && git clone https://github.com/jothepro/doxygen-awesome-css.git
pushd doxygen-awesome-css
git checkout v2.2.1
popd

doxygen