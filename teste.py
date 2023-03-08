import pykitti
import pcl
import numpy as np
import struct

def hv_in_range(m, n, fov, fov_type='h'):
	""" extract filtered in-range velodyne coordinates based on azimuth & elevation angle limit 
		horizontal limit = azimuth angle limit
		vertical limit = elevation angle limit
	"""
	if fov_type == 'h':
		return np.logical_and(np.arctan2(n, m) > (-fov[1] * np.pi / 180), \
							  np.arctan2(n, m) < (-fov[0] * np.pi / 180))
	elif fov_type == 'v':
		return np.logical_and(np.arctan2(n, m) < (fov[1] * np.pi / 180), \
							  np.arctan2(n, m) > (fov[0] * np.pi / 180))
	else:
		raise NameError("fov type must be set between 'h' and 'v' ")

def points_filter(points):
	"""
	filter points based on h,v FOV and x,y,z distance range.
	x,y,z direction is based on velodyne coordinates
	1. azimuth & elevation angle limit check
	2. x,y,z distance limit
	"""
	h_min, h_max = -180, 180
	v_min, v_max = -24.9, 2.0
	v_res, h_res = 0.42, 0.35	
	v_fov, h_fov = (-24.9, 2.0), (-90, 90)	
	if h_fov[0] < -50:
		h_fov = (-50,) + h_fov[1:]
	if h_fov[1] > 50:
		h_fov = h_fov[:1] + (50,)		
	x, y, z = points[:, 0], points[:, 1], points[:, 2]
	d = np.sqrt(x ** 2 + y ** 2 + z ** 2)
	
	h_points = hv_in_range(x, y, h_fov, fov_type='h')
	v_points = hv_in_range(d, z, v_fov, fov_type='v')
	ids = np.logical_and(h_points, v_points)
	return ids

def prepare_velo_points(pts3d_raw):
    '''Replaces the reflectance value by 1, and tranposes the array, so
       points can be directly multiplied by the camera projection matrix'''

    pts3d = pts3d_raw
    # Reflectance > 0
    pts3d = pts3d[pts3d[:, 3] > 0 ,:]
    pts3d[:,3] = 1
    return pts3d.transpose()

def project_velo_points_in_img_0(pts3d):
    '''Project 3D points into 2D image. Expects pts3d as a 4xN
       numpy array. Returns the 2D projection of the points that
       are in front of the camera only an the corresponding 3D points.'''
    # 3D points in camera reference frame.
    xyz_v = pts3d
    R = np.array([0.007967514, -0.9999679, -0.0008462264, \
                  -0.002771053, 0.0008241710, -0.9999958, \
                  0.9999644, 0.007969825, -0.002764397])                  
    t = np.array([-0.01377769, -0.05542117, -0.2918589])
    
    R = R.reshape(3, 3)
    t = t.reshape(3, 1)

    RT = np.concatenate((R, t), axis=1)
    T_cam0unrect_velo = np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))

    P2 = np.array([718.8560, 0.000000, 607.1928, 45.38225, \
                  0.000000, 718.8560, 185.2157, -0.1130887, \
                  0.000000, 0.000000, 1.000000, 0.003779761])
    P2 = np.reshape(P2, (3, 4))
    P_rect_20 = P2[:3, :3]

    R0 = np.array([0.9999454, 0.007259129, -0.007519551, \
                  -0.007292213, 0.9999638, -0.004381729, \
                  0.007487471, 0.004436324, 0.9999621])
    R_rect_00 = np.eye(4)
    R_rect_00[0:3, 0:3] = np.reshape(R0, (3, 3))

    R2 = np.array([0.9999191, 0.01228161, -0.003316013, \
                  -0.01228209, 0.9999246, -0.0001245511, \
                  0.003314233, 0.0001652686, 0.9999945])
    R_rect_20 = np.eye(4)
    R_rect_20[0:3, 0:3] = np.reshape(R2, (3, 3))

    T2 = np.eye(4)
    T2[0, 3] = P2[0, 3] / P2[0, 0]

    # convert velodyne coordinates(X_v, Y_v, Z_v) to camera coordinates(X_c, Y_c, Z_c)
    for i in range(xyz_v.shape[1]):
      xyz_v[:3, i] = np.matmul(RT, xyz_v[:, i]) 

    xyz_c = np.delete(xyz_v, 3, axis=0)

    # convert camera coordinates(X_c, Y_c, Z_c) image(pixel) coordinates(x,y)
    for i in range(xyz_c.shape[1]):
      xyz_c[:, i] = np.matmul(P_rect_20, xyz_c[:, i])

    xy_i = xyz_c[::] / xyz_c[::][2]
    ans = np.delete(xy_i, 2, axis=0)

    return ans
    
def project_velo_points_in_img_567(pts3d):
    '''Project 3D points into 2D image. Expects pts3d as a 4xN
       numpy array. Returns the 2D projection of the points that
       are in front of the camera only an the corresponding 3D points.'''
    # 3D points in camera reference frame.
    xyz_v = pts3d
    R = np.array([0.007027555, -0.9999753, 0.00002599616, \
                  -0.002254837, -0.00004184312, -0.9999975, \
                  0.9999728, 0.007027479, -0.002255075])
    t = np.array([-0.007137748, -0.07482656, -0.3336324])
    
    R = R.reshape(3, 3)
    t = t.reshape(3, 1)

    RT = np.concatenate((R, t), axis=1)
    T_cam0unrect_velo = np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))

    P2 = np.array([707.0912, 0.000000, 601.8873, 46.88783, \
                  0.000000, 707.0912, 183.1104, 0.1178601, \
                  0.000000, 0.000000, 1.000000, 0.006203223])
    P2 = np.reshape(P2, (3, 4))
    P_rect_20 = P2[:3, :3]

    R0 = np.array([0.9999280, 0.008085985, -0.008866797, \
                  -0.008123205, 0.9999583, -0.004169750, \
                  0.008832711, 0.004241477, 0.9999520])
    R_rect_00 = np.eye(4)
    R_rect_00[0:3, 0:3] = np.reshape(R0, (3, 3))
    
    R2 = np.array([0.9999019, 0.01307921, -0.005015634, \
                  -0.01307809, 0.9999144, 0.0002561203, \
                  0.005018555, -0.0001905003, 0.9999874])
    R_rect_20 = np.eye(4)
    R_rect_20[0:3, 0:3] = np.reshape(R2, (3, 3))

    T2 = np.eye(4)
    T2[0, 3] = P2[0, 3] / P2[0, 0]

    # convert velodyne coordinates(X_v, Y_v, Z_v) to camera coordinates(X_c, Y_c, Z_c)
    for i in range(xyz_v.shape[1]):
      xyz_v[:3, i] = np.matmul(RT, xyz_v[:, i]) 

    xyz_c = np.delete(xyz_v, 3, axis=0)

    # convert camera coordinates(X_c, Y_c, Z_c) image(pixel) coordinates(x,y)
    for i in range(xyz_c.shape[1]):
      xyz_c[:, i] = np.matmul(P_rect_20, xyz_c[:, i])

    xy_i = xyz_c[::] / xyz_c[::][2]
    ans = np.delete(xy_i, 2, axis=0)

    return ans

#basepath = '/home/phi/CLionProjects/kitti/'
basepath = '/media/leo/16E8689CE8687BBD1/CLionProjects/kitti/'
sequence = '00'
print("22222")

data = pykitti.odometry(basepath, sequence)
#4541, 2761, 1101, 1101
#for i in range(0,1101):
for i in range(419,420):
  velo_scan = data.get_velo(i)
  rgb = (data.get_rgb(i))[0]
  prepared = prepare_velo_points(velo_scan)
  ids = points_filter(prepared.transpose())
  selected = prepared[:,ids]
  #cloud = (selected.transpose())[:,[0,1,2]]
  #cloudi = (prepared.transpose())[:,[0,1,2]]
  projected = project_velo_points_in_img_567(selected)
  
  valid = 0
  c = 0
  for x in projected[0]:
    y = projected[1][c]
    if (x >= 0 and y >= 0 and x < 1226 and y < 370):
      valid += 1
    c += 1

  cloud_color = np.zeros((valid, 4), dtype=np.float32)
  valid = 0
  c = 0
  for x in projected[0]:
    y = projected[1][c]
    if (x >= 0 and y >= 0 and x < 1226 and y < 370):
      x = np.int(x)
      y = np.int(y)
      cloud_color[valid][0] = np.float32(selected[0][c])
      cloud_color[valid][1] = np.float32(selected[1][c])
      cloud_color[valid][2] = np.float32(selected[2][c])
      r, g, b = rgb.getpixel((x, y));
      cloud_color[valid][3] = np.int(r) << 16 | np.int(g) << 8 | np.int(b)
      valid += 1
      rgb.putpixel((x, y), (255,255,0))

      #ca += 1
    c += 1

  #p = pcl.PointCloud()
  #p.from_array(cloud)
  #pcl.save(p, "teste.pcd")
  p = pcl.PointCloud_PointXYZRGBA()
  p.from_array(cloud_color)
  #pcl.save(p, format(i, '06') + ".pcd")
  rgb.save("oie.png")
  print(i)
	

print("weeeee")
