import os

for dirname, dirnames, filenames in os.walk('.'):
    # print path to all subdirectories first.
    for subdirname in dirnames:
        os.path.join(dirname, subdirname)

    # print path to all filenames.
    for filename in filenames:
        print os.path.join(dirname, filename)

# with open("file.txt", "r") as f:
#     searchlines = f.readlines()
# for i, line in enumerate(searchlines):
#     if "searchphrase" in line: 
#         for l in searchlines[i:i+3]: print l,
#         print