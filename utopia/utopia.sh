#!/bin/bash
#
# utopia.sh
#
#       Set up QEMU and launch it.
#
# Copyright (C) 2013-2024, Computing Systems Lab., ECE, NTUA.

set -e  # Fail on all errors.

UTOPIA_CONFIG="./utopia.config"

VERBOSE=1
PF=\*\*\*

if [ -n "$VERBOSE" ]; then
    echo " "
    echo " $PF Reading configuration"
    echo " "
fi

# Check for the configuration file
if [ -n "$VERBOSE" ]; then
    echo "UTOPIA_CONFIG="$UTOPIA_CONFIG
fi

if [ ! -e "$UTOPIA_CONFIG" ]; then
    echo "$0: cannot find '$UTOPIA_CONFIG'." 1>&2
    echo "Please provide a configuration file and rerun." 1>&2
    exit 1
fi

# Source the configuration file
# FIXME: some parsing here might be better.

# Default values
QEMU_BUILD_DIR=$HOME/opt/qemu
UTOPIA_MEMORY_MB=1024
UTOPIA_SSH_PORT=22223
UTOPIA_SSH_INTERFACE=127.0.0.1
QCOW2_BACKING_FILE=./cslab_rootfs_2024.raw
QCOW2_PRIVATE_FILE=./private.qcow2
SHARED_FS_DIR=./shared


. $UTOPIA_CONFIG

if [ -n "$VERBOSE" ]; then
    echo " "
    echo " $PF Checking configuration"
    echo " "
fi

# Check for qemu installation
if [ -z "$QEMU_BUILD_DIR" ]; then
    echo "$0: Configuration variable QEMU_BUILD_DIR is empty." 1>&2
    echo "Please specify the correct location of the QEMU build directory." 1>&2
    exit 1
fi

if [ ! -e "$QEMU_BUILD_DIR" ] || [ ! -d "$QEMU_BUILD_DIR" ]; then
    echo "$0: '$QEMU_BUILD_DIR' does not exist or is not a directory." 1>&2
    echo "Please specify the correct location of the QEMU build directory." 1>&2
    exit 1
fi

QEMU=$QEMU_BUILD_DIR/bin/qemu-system-x86_64
QEMU_IMG=$QEMU_BUILD_DIR/bin/qemu-img
if [ ! -e "$QEMU" ] || [ ! -e "$QEMU_IMG" ]; then
    echo "$0: Could not find QEMU binaries in '$QEMU_BUILD_DIR'." 1>&2
    echo "Make sure you have successfully installed QEMU in the specified directory." 1>&2
    exit 1
fi


# Check for a private qcow2 image.
# If not found, create one based on a read-only backing file.
if [ -n "$VERBOSE" ]; then
    echo "QCOW2_PRIVATE_FILE="$QCOW2_PRIVATE_FILE
    echo "QCOW2_BACKING_FILE="$QCOW2_BACKING_FILE
fi

if [ -z "$QCOW2_PRIVATE_FILE" ]; then
    echo "$0: Configuration variable QCOW2_PRIVATE_FILE is empty." 1>&2
    echo "Please specify a private qcow2 filename." 1>&2
    exit 1
fi

if [ ! -f "$QCOW2_PRIVATE_FILE" ]; then
    echo "$0: Private file '$QCOW2_PRIVATE_FILE' not found." 1>&2

    if [ -z "$QCOW2_BACKING_FILE" ]; then
        echo "$0: Configuration variable QCOW2_BACKING_FILE is empty." 1>&2
        echo "Please specify a backing qcow2 filename." 1>&2
        exit 1
    fi
    
    if [ ! -e "$QCOW2_BACKING_FILE" ]; then
        echo "$0: '$QCOW2_BACKING_FILE' does not exist." 1>&2
        echo "Please specify an existing backing qcow2 filename." 1>&2
        exit 1
    fi
    echo "Creating file '$QCOW2_PRIVATE_FILE', backed by '$QCOW2_BACKING_FILE'" 1>&2
    $QEMU_IMG create -b "$QCOW2_BACKING_FILE" -f qcow2 -F raw "$QCOW2_PRIVATE_FILE"
fi

# Ensure the shared directory exists
if [ -n "$VERBOSE" ]; then
    echo "SHARED_FS_DIR="$SHARED_FS_DIR
fi

echo "Ensuring shared directory '$SHARED_FS_DIR' exists" 1>&2
mkdir -p "$SHARED_FS_DIR"


echo " "
echo " $PF Starting your Virtual Machine ..."
echo " "

#port=$[($RANDOM % 20002000) + 2222]
echo "To connect with X2Go: See below for SSH settings" 1>&2
echo "To connect with SSH: ssh -p $UTOPIA_SSH_PORT root@localhost" 1>&2
echo "To connect with vncviewer: vncviewer localhost:0" 1>&2

# This will connect the first serial port of the VM
# over TCP to lunix.cslab.ece.ntua.gr:49152.
exec $QEMU -enable-kvm -M pc -m $UTOPIA_MEMORY_MB \
    -smp 2 -drive file=$QCOW2_PRIVATE_FILE,if=virtio \
    -net nic -net user,hostfwd=tcp:$UTOPIA_SSH_INTERFACE:$UTOPIA_SSH_PORT-:22 \
    -vnc 127.0.0.1:0 \
    -nographic -monitor /dev/null  \
    -chardev socket,id=sensors0,host=lunix.cslab.ece.ntua.gr,port=49152,ipv4=on \
    -device isa-serial,chardev=sensors0,id=serial1,index=1 \
    -fsdev local,id=fsdev0,path="$SHARED_FS_DIR",security_model=none \
    -device virtio-9p-pci,fsdev=fsdev0,mount_tag=shared \
    $*
    #-fw_cfg name=etc/sercon-port,string=0 \
    #-fsdev local,id=exp1,path=$HOME,security_model=passthrough \
    #-device virtio-9p-pci,fsdev=exp1,mount_tag=v_tmp \
    #-chardev pty,id=charserial0 \
    #-device isa-serial,chardev=charserial0,id=serial0 \
