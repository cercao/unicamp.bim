import numpy as np
import matplotlib.image as mplimg
import matplotlib.pyplot as plt
from skimage.color import rgb2gray
from argparse import ArgumentParser
import glob
import sys,os
from skimage.morphology import disk
from skimage.morphology import closing
from scipy.ndimage.interpolation import rotate
from skimage.filters import threshold_isodata

ia898path = os.path.abspath('toolbox')
if ia898path not in sys.path:
    sys.path.append(ia898path)
import ia898.src as ia

def scale(X,_range):
	_min,_max = _range
	X_std = (X - X.min()) / (X.max() - X.min())
	X_scaled = X_std * (_max - _min) + _min
	return X_scaled

def generate(file_dir, img):
	file = open(file_dir,'r')
	cloud = file.readlines()

	x,y  = [],[]

	r,c = img.shape

	for i in range(11,len(cloud)):
		line = cloud[i].split(' ')
		x.append(float(line[1]))
		y.append(float(line[2]))

	x_a = np.array(x); y_a = np.array(y)

	xa = np.rint(scale(x_a,(0,c-1))).astype(int)
	ya = np.rint(scale(y_a,(0,r-1))).astype(int)

	img = np.zeros((r,c)).astype(int)
	img[ya,xa] = 1
	img2 = closing(img,disk(5))

	return rotate(img2,180,cval=0)

from humanfriendly.tables import format_pretty_table

def compare(true_img,pred_img,out=False):
	true_img = true_img > threshold_isodata(true_img)
	pred_img = pred_img > threshold_isodata(pred_img)
	TP = np.sum(np.logical_and(true_img==1,pred_img==1))
	TN = np.sum(np.logical_and(true_img==0,pred_img==0))
	FP = np.sum(np.logical_and(true_img==0,pred_img==1))
	FN = np.sum(np.logical_and(true_img==1,pred_img==0))

	precision = TP/(TP+FP)
	recall = TP/(TP+FN)
	accu = (TP+TN)/(TP+TN+FP+FN)

	if out:
		return [precision, recall, accu]
	else:
		return [TP,TN,FP,FN]

parse  = ArgumentParser()
parse.add_argument("path",type=str)
parse.add_argument("image",type=str)

args = parse.parse_args()

img1 = rgb2gray(mplimg.imread(args.image))
img2 = generate(args.path, img1)

header1 = ['True Positive','True Negative','False Positive','False Negative']
header2 = ['Precision', 'Recall','Accuracy']

condition = compare(img1,img2)
evaluation = compare(img1,img2,True)

print(format_pretty_table([list(map(int,condition))],header1))
print(format_pretty_table([list(map(float,evaluation))],header2))

plt.subplot(121)
plt.imshow(img1,cmap='gray')
plt.subplot(122)
plt.imshow(img2,cmap='gray')
plt.show()



# all_clouds = glob.glob('/home/avell/Downloads/Learning_1/*.pcd')

# for cloud in all_clouds:
# 	name = cloud.split('/')[-1][:-4]
# 	fig = plt.figure()
# 	plt.imshow(generate(cloud),cmap='gray')
# 	plt.savefig(name+'png',dpi=fig.dpi)
