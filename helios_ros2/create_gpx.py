#!/usr/bin/env python3
import gpxpy.gpx
import os
import sys

def main(argv):	
	path="logs/"
	try:
		name=sys.argv[1]
	except:
		name="logs_test"
	traj_input = open(path+name+"_traj.txt","r")
	path_input = open(path+name+"_path.txt","r")
	traj_output = open(path+name+"_traj.gpx","w")
	path_output = open(path+name+"_path.gpx","w")

	#ecriture path
	
	lon_list = []
	lat_list = []


	gpx = gpxpy.gpx.GPX()

	gpx_track = gpxpy.gpx.GPXTrack()
	gpx.tracks.append(gpx_track)

	gpx_segment = gpxpy.gpx.GPXTrackSegment()
	gpx_track.segments.append(gpx_segment)
	for line in path_input.readlines():
		inter=line.split(",")
		lon_list.append(float(inter[1]))
		lat_list.append(float(inter[0]))
	
	for i in range(len(lon_list)):
		gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lon_list[i], lat_list[i]))
	
	path_output.write(gpx.to_xml())
	path_output.write("\n")
	path_output.close()
	path_input.close()
	
	#ecriture traj
	gpx = gpxpy.gpx.GPX()

	gpx_track = gpxpy.gpx.GPXTrack()
	gpx.tracks.append(gpx_track)

	gpx_segment = gpxpy.gpx.GPXTrackSegment()
	gpx_track.segments.append(gpx_segment)

	lon_list = []
	lat_list = []
	for line in traj_input.readlines():
		inter=line.split(",")
		lon_list.append(float(inter[1]))
		lat_list.append(float(inter[0]))
	
	for i in range(len(lon_list)):
		gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lon_list[i], lat_list[i]))
	
	traj_output.write(gpx.to_xml())
	traj_output.write("\n")
	traj_output.close()
	traj_input.close()




if __name__ == "__main__":
	main(sys.argv[1:])	
