import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import math
b = bagreader('/home/arvinder/RSN/src/Gps_driver/Analysis/rosbag_data/Soccer_field_stationary.bag')
max_dist = 0

gpsdata= b.message_by_topic(topic="/gpstopic")
gpsdataread = pd.read_csv(gpsdata)
gpsdf = pd.DataFrame(gpsdataread, columns=[ "latitude.data", "longitude.data", "altitude.data", "utm_easting.data", "utm_northing.data", "fix_quality.data"]).astype(float)


easting_mean = float(gpsdf["utm_easting.data"].mean())
northing_mean = float(gpsdf["utm_northing.data"].mean())

for ind in gpsdf.index:
    distance = math.sqrt((easting_mean - float(gpsdf["utm_easting.data"][ind]))**2 + (northing_mean - float(gpsdf["utm_northing.data"][ind]))**2 )
    if distance > max_dist:
        max_dist = distance
    else:
        pass

print("Maximum distance from mean(Error):%fm" %max_dist)

plotgps = gpsdf.plot.scatter("utm_easting.data","utm_northing.data", label= "UTM(Easting,Northing) for solution status FIX")
plt.plot(easting_mean,northing_mean,label= "UTM(Average Easting,Average Northing)", marker="o", markersize=10,  markerfacecolor="red")
plt.legend(loc="upper left")
plt.xlabel("UTM Easting(m)")
plt.ylabel("UTM Northing(m)")
plt.grid(True)
plt.title("Easting vs Northing without obstructions(Stationary)")
plt.show()
