#!/bin/bash
for file in *.bin
do 
	kitti2pcd --infile $file --outfile `basename $file .bin`.pcd
	mkdir `basename $file .bin`;
done
