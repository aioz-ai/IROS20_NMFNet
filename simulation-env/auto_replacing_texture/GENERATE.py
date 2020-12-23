import random 
import os, glob
from PIL import Image
import operator

from PIL import ImageDraw

import os
#path to the folder which contains all of your models. Please remember to backup your old models first before running this!
path = '/home/aioz-kim/.gazebo/models'
texture_path = '/home/aioz-kim/gazebo_simulation/texture_collection'

files = []
texture_files = []
shift = (0, 0)

def blend(img1, img2):
	# compute the size of the panorama
	nw, nh = map(max, map(operator.add, img2.size, shift), img1.size)

	# paste img1 on top of img2
	newimg1 = Image.new('RGBA', size=(nw, nh), color=(0, 0, 0, 0))
	newimg1.paste(img2, shift)
	newimg1.paste(img1, (0, 0))

	# paste img2 on top of img1
	newimg2 = Image.new('RGBA', size=(nw, nh), color=(0, 0, 0, 0))
	newimg2.paste(img1, (0, 0))
	newimg2.paste(img2, shift)

# blend with alpha=0.5
	result = Image.blend(newimg1, newimg2, alpha=0.5)
	return result

def generate_image(path, image_name):
	img1 = Image.new('RGB', (700, 500), (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
	return img1
#scan through original textures in models folders
for r, d, f in os.walk(path):
	for file in f:
		if '.png' in file:
			files.append(os.path.join(r, file))
		if '.jpg' in file:
			files.append(os.path.join(r, file))
		if '.jpeg' in file:
				files.append(os.path.join(r, file))
		if '.tga' in file:
				files.append(os.path.join(r, file))	

#scan through random textures for blending  
for r, d, f in os.walk(texture_path):
	for file in f:
		if '.png' in file:
			texture_files.append(os.path.join(r, file))
		if '.jpg' in file:
			texture_files.append(os.path.join(r, file))
		if '.jpeg' in file:
				texture_files.append(os.path.join(r, file))
		if '.tga' in file:
				files.append(os.path.join(r, file))		
count = 0
for f in files:
	#print(f)
	fname = str(f).split('/')[-1:][0]
	#fname = str(count)
	new_url = f.replace(fname,'')
	img1 = generate_image(new_url,fname)
	img2 = Image.open(random.choice(texture_files))
	img2 = img2.resize((700,500))
	result_img = blend(img1, img2)
	os.remove(f)
	save_path = new_url + str(fname)
	print(save_path)
	result_img.save(save_path)
	count = count + 1
	

	
