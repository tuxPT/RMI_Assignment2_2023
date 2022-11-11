#!/bin/bash

challenge="1"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
        # how to call agent for challenge 1
        python3 mainRob.py -c 1 -h "$host" -p "$pos" -r "$robname"
        ;;
    2)
        # how to call agent for challenge 2
        python3 mainRob.py -c 2 -h "$host" -p "$pos" -r "$robname" -f "$outfile.map"
        ;;
    3)
        # how to call agent for challenge 3
        python3 mainRob.py -c 3 -h "$host" -p "$pos" -r "$robname" -f "$outfile.path"
        ;;
    4)
        # how to call agent for challenge 4
        python3 mainRob.py -c 4 -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        ;;
esac

