#!/bin/bash
SCRIPT_DIR=$(dirname $(readlink -f $0))
echo ros | sudo -S HOME=$SCRIPT_DIR $SCRIPT_DIR/URControl &>$SCRIPT_DIR/URControl.log &
