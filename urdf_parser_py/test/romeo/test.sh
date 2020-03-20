#!/bin/bash

dir=$(dirname $BASH_SOURCE)
scripts="$dir/../../scripts"

name=romeo
orig="$(rospack find romeo_description)/urdf/$name.urdf"
gen="/tmp/$name.urdf"
patch="/tmp/$name.patch"

$scripts/display_urdf "$orig" -o "$gen"
diff -u "$orig" "$gen" > "$patch"
