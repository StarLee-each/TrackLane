#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR/protobuf
$SCRIPT_DIR/nanopb/generator/nanopb_generator.py lane.proto
$SCRIPT_DIR/nanopb/generator/nanopb_generator.py ble.proto
# $SCRIPT_DIR/nanopb/generator/nanopb_generator.py track_option.proto
