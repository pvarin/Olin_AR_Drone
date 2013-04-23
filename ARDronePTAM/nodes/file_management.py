#!/usr/bin/env python
import roslib
roslib.load_manifest('ARDronePTAM')
import fileinput, sys
import rospy
import os, os.path
from sensor_msgs.msg import PointCloud2


class FileManager:

	def __init__(self):
		self.imgSubscriber = rospy.Subscriber( "vslam/pc2", PointCloud2, self.cloud_cb )
		self.rootdir='/home/eric/groovy_workspace/Olin_AR_Drone/ARDronePTAM/data'
		self.cb_count = 0

	def find_oldest(self, files):
		filenames = []
		for filename in files:
			if filename == 'master.pcd':
				filenames.append(999999999999)
			else:
				filenames.append(int(filename[:10]))

		return files[filenames.index(min(filenames))]


	def contains_line(self, readfile, thisline):
		for line in readfile:
			if line == thisline:
				return True
		return False


	def cloud_cb(self, data):
		if not data.data:
			print "No points in the point cloud"
			return
		
		if self.cb_count == 0:
			pass
		elif self.cb_count == 1:
			for dirpath, dirnames, files in os.walk(self.rootdir):
				old = self.find_oldest(files)

			print old
			os.rename( os.path.join( self.rootdir, old ), os.path.join( self.rootdir, "master.pcd" ) )
		# for subdir, dirs, files in os.walk(self.rootdir):
		#     print "ALL FILES:"
		#     for file in files:
		#     	print file
		    	#if file != files[len(files)-1]:
			        # f=open(file, 'r')
			        # lines=f.readlines()
			        # f.close()
			        # print "I opened a file!!!!!!!!!!!!"
			        #f=open(file, 'w')
			        #for line in lines:
			            #newline=doWhatYouWant(line)
			            #f.write(newline)
			        #f.close()
		else:
			for dirpath, dirnames, files in os.walk(self.rootdir):
				old = self.find_oldest(files)

			with open(self.rootdir + "/" + old) as infile:
				with open(self.rootdir + "/master.pcd", "r") as readfile:
					line_buffer = ""

					for i, line in enumerate(infile):
						if not self.contains_line(readfile, line):
							line_buffer = line_buffer + line

			with open(self.rootdir + "/master.pcd", "a") as outfile:
				outfile.write(line_buffer)

			os.remove(self.rootdir + "/" + old)

		print self.cb_count
		self.cb_count += 1


def main(args):
  rospy.init_node('file_management')
  fm = FileManager()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)