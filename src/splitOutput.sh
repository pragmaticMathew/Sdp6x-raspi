#!/bin/bash 

if [ -z "$1" ]; then
  echo "Error: missing input file parameter"
  exit 1
fi

FILE=$1     
if [ ! -f $FILE ]; then
   echo "File $FILE does not exists. Quit now"
   exit 1
fi

echo "this script takes $FILE and splits it up to $FILE.output and $FILE.metadata" 

if [  -f $FILE.output ]; then
   rm $FILE.output
fi

if [  -f $FILE.output.metadata ]; then
   rm $FILE.output.metadata
fi

cat $FILE | grep --invert-match  "*" >$FILE.output
cat $FILE | grep "*" >$FILE.output.metadata


