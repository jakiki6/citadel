#!/bin/sh

export PATH=$PATH:$(guix build -f toolchain.scm)/bin
