#!/bin/bash
if [ $# -gt 0 ]; then
    # Loop until all parameters are used up
    ((i = 0))
    while [ "$1" != "" ]; do
        echo "Binding $1 to /dev/rfcomm$i"
        sudo rfcomm bind $i $1
        ((i = i+1))
        # Shift all the parameters down by one
        shift
    done
else
    echo "Your command line contains no arguments"
fi