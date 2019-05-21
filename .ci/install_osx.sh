#!/usr/bin/env bash
set -ex

brew update > /dev/null
brew bundle

pip3 install -U pytest

if [ "$BUILD_DARTPY" = "ON" ]; then
  brew install pybind11
fi
