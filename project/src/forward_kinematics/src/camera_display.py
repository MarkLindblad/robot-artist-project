#!/usr/bin/env python
import argparse
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import intera_interface
import dots
import matplotlib.pyplot as plt

def show_image_callback(img_data, (edge_detection, window_name)):

	"""The callback function to show image by using CvBridge and cv

	"""
	bridge = CvBridge()

	try:
		cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")

	except CvBridgeError, err:
		rospy.logerr(err)
		return

	# if edge_detection == True:
	# 	gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	# 	blurred = cv2.GaussianBlur(gray, (3, 3), 0)
	# 	# customize the second and the third argument, minVal and maxVal
	# 	# in function cv2.Canny if needed
	# 	get_edge = cv2.Canny(blurred, 10, 100)
	# 	cv_image = np.hstack([get_edge])
	# edge_str = "(Edge Detection)" if edge_detection else ''

	# gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

	# dtype, n_channels = bridge.encoding_as_cvtype2('8UC3')
	# im = np.ndarray(shape=(480, 640, n_channels), dtype=dtype)
	# cv2.imwrite("this_was_a_message_briefly.png", im)
	filename = 'savedImage.jpg'
	cv2.imwrite(filename, cv_image)
	img = np.array(cv2.imread(filename, 0))


	print(np.min(img), np.max(img))

	plt.imshow(img, cmap = 'gray')
	plt.show()

	dots_x, dots_y, size = dots.grey_dither(img, angle = 30, size = 100)
	plt.plot(dots_x, dots_y, 'ko')
	plt.show()
	cv_win_name = ' '.join([window_name, edge_str])
	cv2.namedWindow(cv_win_name, 0)
	# refresh the image on the screen
	cv2.imshow(cv_win_name, cv_image)
	cv2.waitKey(10)

def main():
	"""Camera Display Example

	"""
	rp = intera_interface.RobotParams()
	valid_cameras = rp.get_camera_names()
	print(valid_cameras)

	if not valid_cameras:
		rp.log_message(("Cannot detect any camera_config" " parameters on this robot. Exiting."), "ERROR")
		return
	arg_fmt = argparse.RawDescriptionHelpFormatter

	parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
	parser.add_argument('-c', '--camera', type=str, default="head_camera", choices=valid_cameras, help='Setup Camera Name for Camera Display')
	parser.add_argument('-r', '--raw', action='store_true', help='Specify use of the raw image (unrectified) topic')
	parser.add_argument('-e', '--edge', action='store_true', help='Streaming the Canny edge detection image')

	args = parser.parse_args()
	print("Initializing node... ")

	rospy.init_node('camera_display', anonymous=True)
	camera = intera_interface.Cameras()

	if not camera.verify_camera_exists(args.camera):
		rospy.logerr("Invalid camera name, exiting the example.")
		return

	camera.start_streaming(args.camera)
	rectify_image = not args.raw
	use_canny_edge = args.edge
	camera.set_callback(args.camera, show_image_callback, rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera))

	def clean_shutdown():
		print("Shutting down camera_display node.")
		cv2.destroyAllWindows()
	rospy.on_shutdown(clean_shutdown)
	rospy.loginfo("Camera_display node running. Ctrl-c to quit")
	rospy.spin()



if __name__ == '__main__':
	main()
