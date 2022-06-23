import os
#import shutil

# This program was written to assemble all gathered data which is stored
# in seperate folders for each product to one dataset kept in the same folder.
# Additionally opearations are performed on the lables to fit the needed neural
# network input format.
# For Yolo the standard format is: label xcenter ycenter xwidth xheight
# All coordinates and sizes should be in normalized values from 0-1 based
# on image resolution, image resolution used for D435 was 848x480 

rootdir="/home/lab/Documents/Eidur/Data"

# Folder to keep the combined dataset in
folder = input("Destination folder name\n")
destdir = os.path.join(rootdir,folder)
print(destdir)

os.mkdir(destdir)
print("Directory " , destdir ,  " Created ") 

input("-\n")

# Dataset folder
folder = input("Source folder name\n") 

sourcedir = os.path.join(rootdir,folder)

input("-\n")

counter=0

for subdir, dirs, files in os.walk(sourcedir):
    for file in files:
        print(os.path.join(subdir, file))
        print(file)
        print(os.path.splitext(file))
        filename = str(counter).zfill(4)

        counter+=1