import moonranger as m

if __name__ == '__main__':
	ekf = m.OrientationEKF()
	print((ekf.is_ekf_initialized()))