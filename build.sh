#!/usr/bin/env bash
# Packages the module into a source tarball for linux/any upload.
set -euo pipefail

cd "$(dirname "$0")"

tar -czf module.tar.gz \
    meta.json \
    requirements.txt \
    setup.sh \
    run.sh \
    src \
    README.md

echo "Wrote module.tar.gz"
