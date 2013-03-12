import os

searchphrase = ["UDP", "udp", "5556"]

for dirname, dirnames, filenames in os.walk('.'):
    # print path to all subdirectories first.
    for subdirname in dirnames:
        os.path.join(dirname, subdirname)

    # print path to all filenames.
    for filename in filenames:
    	fn = os.path.join(dirname, filename)
		with open(fn, "r") as f:
		    searchlines = f.readlines()
		for i, line in enumerate(searchlines):
		    if searchphrase[0] or searchphrase[1] or searchphrase[2] in line: 
		        print "%s [%d] \n" % (fn, i)