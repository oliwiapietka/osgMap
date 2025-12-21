#!/usr/bin/env bash
set -euo pipefail

MAIN_BRANCH=${1:-origin/main}
CLANG_BIN=${2:-clang-format-14}
GIT_CLANG_FORMAT=git-clang-format-14

if ! command -v "$CLANG_BIN" >/dev/null 2>&1; then
  echo "clang-format not found: $CLANG_BIN"
  exit 1
fi

if ! command -v "$GIT_CLANG_FORMAT" >/dev/null 2>&1; then
  echo "git-clang-format not found: $GIT_CLANG_FORMAT"
  exit 1
fi

git fetch origin main >/dev/null 2>&1 || true

OUTPUT=$("$GIT_CLANG_FORMAT" \
  --diff "$MAIN_BRANCH" \
  --extensions c,cpp,h,hpp \
  --binary "$CLANG_BIN")

if echo "$OUTPUT" | grep -qi "no modified files to format"; then
  exit 0
fi

if echo "$OUTPUT" | grep -qi "did not modify any files"; then
  exit 0
fi

echo
echo "/// code format check failed!"
echo
echo "$OUTPUT"
exit 1
