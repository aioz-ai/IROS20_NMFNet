import random 
import os, glob
from PIL import Image
from shutil import copyfile

import os
#path to the folder which contains all of your models. Please remember to backup your old models first before running this!
path = '/home/aioz-kim/gazebo_simulation/extra_models'

files = []

for r, d, f in os.walk(path):
	for file in f:
		if 'model-1_4.sdf' in file:
			files.append(os.path.join(r, file))
			

for f in files:
	print(f)
	fname = str(f).split('/')[-1:][0]
	new_url = f.replace(fname,'model.sdf')
        print(new_url)
        copyfile(new_url,f)
	
