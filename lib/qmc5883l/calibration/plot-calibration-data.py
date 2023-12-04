import numpy as np
import matplotlib.pyplot as plt


# Define calibration parameters
A = np.array(   [
    [0.001228,0.000015,0.000005],
    [0.000015,0.001351,-0.000004],
    [0.000005,-0.000004,0.000566]
])
b = np.array([146.439243, 239.251069, 112.504704])


# Read raw data and apply calibration
rawData = np.genfromtxt('./mag-readings.txt', delimiter='\t')  # Read raw measurements

calibData = np.genfromtxt('./mag-readings-actual.txt', delimiter='\t')

N = 0
if len(calibData) >  len(rawData):
    N = len(rawData)
else:
    N = len(calibData)

# Plot XY data
plt.figure()
plt.plot(rawData[:, 0], rawData[:, 1], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 0], calibData[:, 1], 'r*', label='Calibrated Meas.')
plt.title('XY Magnetometer Data')
plt.xlabel('X [uT]')
plt.ylabel('Y [uT]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot YZ data
plt.figure()
plt.plot(rawData[:, 1], rawData[:, 2], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 1], calibData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('YZ Magnetometer Data')
plt.xlabel('Y [uT]')
plt.ylabel('Z [uT]')
plt.legend()
plt.grid()
plt.axis('equal')

# Plot XZ data
plt.figure()
plt.plot(rawData[:, 0], rawData[:, 2], 'b*', label='Raw Meas.')
plt.plot(calibData[:, 0], calibData[:, 2], 'r*', label='Calibrated Meas.')
plt.title('XZ Magnetometer Data')
plt.xlabel('X [uT]')
plt.ylabel('Z [uT]')
plt.legend()
plt.grid()
plt.axis('equal')


# Plot 3D scatter
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(N):
    xraw = rawData[i, 0]
    yraw = rawData[i, 1]
    zraw = rawData[i, 2]

    xcalib = calibData[i, 0]
    ycalib = calibData[i, 1]
    zcalib = calibData[i, 2]
    ax.scatter(xraw, yraw, zraw, color='b', label='Raw Meas.')
    ax.scatter(xcalib, ycalib, zcalib, color='r', label='Calibrated Meas.')

ax.set_title('3D Scatter Plot of Magnetometer Data')
ax.set_xlabel('X [uT]')
ax.set_ylabel('Y [uT]')
ax.set_zlabel('Z [uT]')


# If you want to print out time series data of mag measurements

# magnetTestData = np.genfromtxt('./data.txt', delimiter='\t')
# xcalibmagnet = magnetTestData[:, 0]
# ycalibmagnet = magnetTestData[:, 1]
# zcalibmagnet = magnetTestData[:, 2]

# plt.figure()
# plt.plot(xcalibmagnet,label='ac_X')
# plt.plot(ycalibmagnet,label='ac_Y')
# plt.plot(zcalibmagnet,label='ac_Z')
# plt.legend(loc='upper left', ncol=3)
# plt.xlabel('Time')
# plt.ylabel("G's")
# plt.grid()


# testData = np.genfromtxt('./data.txt', delimiter='\t')

# pp = testData[:, 0]
# pi = testData[:, 1]
# pd = testData[:, 2]

# epitch = testData[:, 3]
# eyaw = testData[:, 4]
# etotal = testData[:, 5]

# acx = testData[:, 6]
# acy = testData[:, 7]
# acz = testData[:, 8]

# gcx = testData[:, 9]
# gcy = testData[:, 10]
# gcz = testData[:, 11]

# plt.figure()
# plt.plot(pp,label='P')
# plt.plot(pi,label='I')
# plt.plot(pd,label='D')
# plt.legend(loc='upper left', ncol=3)
# plt.xlabel('Time')
# plt.ylabel("G's")
# plt.grid()

# plt.figure()
# plt.plot(epitch,label='error_pitch')
# plt.plot(eyaw,label='error_yaw')
# plt.plot(etotal,label='error_total')
# plt.legend(loc='upper left', ncol=3)
# plt.xlabel('Time')
# plt.ylabel("G's")
# plt.grid()

# plt.figure()
# plt.plot(acx,label='accelerometer_X')
# plt.plot(acy,label='accelerometer_Y')
# plt.plot(acz,label='accelerometer_Z')
# plt.legend(loc='upper left', ncol=3)
# plt.xlabel('Time')
# plt.ylabel("G's")
# plt.grid()


# plt.figure()
# plt.plot(gcx,label='gyro_X')
# plt.plot(gcy,label='gyro_Y')
# plt.plot(gcz,label='gyro_Z')
# plt.legend(loc='upper left', ncol=3)
# plt.xlabel('Time')
# plt.ylabel("G's")
# plt.grid()








testData = np.genfromtxt('./data2.txt', delimiter='\t')

# pp = testData[:, 0]
# pi = testData[:, 1]
# pd = testData[:, 2]
# 
# epitch = testData[:, 3]
# eyaw = testData[:, 4]
# etotal = testData[:, 5]
# 
# acx = testData[:, 6]
# acy = testData[:, 7]
# acz = testData[:, 8]

gcx = testData[:, 0]
gcy = testData[:, 1]
gcz = testData[:, 2]

# plt.figure()
# plt.plot(pp,label='P')
# plt.plot(pi,label='I')
# plt.plot(pd,label='D')
# plt.legend(loc='upper left', ncol=3)
# plt.xlabel('Time')
# plt.ylabel("G's")
# plt.grid()

# plt.figure()
# plt.plot(epitch,label='error_pitch')
# plt.plot(eyaw,label='error_yaw')
# plt.plot(etotal,label='error_total')
# plt.legend(loc='upper left', ncol=3)
# plt.xlabel('Time')
# plt.ylabel("G's")
# plt.grid()

# plt.figure()
# plt.plot(acx,label='accelerometer_X')
# plt.plot(acy,label='accelerometer_Y')
# plt.plot(acz,label='accelerometer_Z')
# plt.legend(loc='upper left', ncol=3)
# plt.xlabel('Time')
# plt.ylabel("G's")
# plt.grid()


plt.figure()
plt.plot(gcx,label='gyro_X')
plt.plot(gcy,label='gyro_Y')
plt.plot(gcz,label='gyro_Z')
plt.legend(loc='upper left', ncol=3)
plt.xlabel('Time')
plt.ylabel("G's")
plt.grid()

plt.show()
