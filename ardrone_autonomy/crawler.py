import os
import sys

searchphrase = sys.argv[1]

for dirname, dirnames, filenames in os.walk('.'):
    for filename in filenames:
    	fn = os.path.join(dirname, filename)
#	print fn
    	with open(fn, "r") as f:
    		searchlines = f.readlines()
    		for i, line in enumerate(searchlines):
    			if searchphrase in line: 
    				print "%s [%d] \n" % (fn, i)
