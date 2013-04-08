import os

searchphrase = ["UDP", "udp", "5556"]

for dirname, dirnames, filenames in os.walk('.'):

    for filename in filenames:
    	fn = os.path.join(dirname, filename)
    	with open(fn, "r") as f:
    		searchlines = f.readlines()
    		for i, line in enumerate(searchlines):
    			if searchphrase[2] in line: 
    				print "%s [%d] \n" % (fn, i)