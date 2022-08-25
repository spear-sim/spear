#!/bin/bash

for f in *; 
do 
    subs=`ls $f`

    for files in $subs;
    do 
       mv "$f/$files/data/images" "$f/$files/images"
       mv "$f/$files/data/sensor_data" "$f/$files/sensor_data"
       rm -r "$f/$files/data"
    done

done


