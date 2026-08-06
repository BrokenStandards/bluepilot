#!/usr/bin/env bash
# Builds the BluePilot mapd fork (third_party/mapd_bp) as static binaries for
# the comma device (arm64) and PC (x86_64), running the Go test suite first.
set -euo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../third_party/mapd_bp/src" && pwd)"

cd "$SRC_DIR"

echo "== go test =="
go test ./...

echo "== building arm64 =="
CGO_ENABLED=0 GOOS=linux GOARCH=arm64 go build -ldflags "-s -w" -o ../mapd .

echo "== building x86_64 =="
CGO_ENABLED=0 GOOS=linux GOARCH=amd64 go build -ldflags "-s -w" -o ../mapd-x86_64 .

echo "== done =="
ls -la ../mapd ../mapd-x86_64
