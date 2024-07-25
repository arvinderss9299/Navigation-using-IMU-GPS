import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import math
b = bagreader('/home/arvinder/RSN/src/Gps_driver/Analysis/rosbag_data/ISEC_stationary.bag')
max_dist_fix = 0
max_dist_float = 0
max_dist_single = 0


gpsdata= b.message_by_topic(topic="/gpstopic")
gpsdataread = pd.read_csv(gpsdata)
gpsdf = pd.DataFrame(gpsdataread, columns=[ "latitude.data", "longitude.data", "altitude.data", "utm_easting.data", "utm_northing.data", "fix_quality.data"]).astype(float)

easting_mean = float(gpsdf["utm_easting.data"].mean())
northing_mean = float(gpsdf["utm_northing.data"].mean())

df1 = pd.DataFrame(columns=["easting_fix","northing_fix"])
df2 = pd.DataFrame(columns=["easting_float","northing_float"])
df3 = pd.DataFrame(columns=["easting_single","northing_single"])


for ind in gpsdf.index:
    if gpsdf["fix_quality.data"][ind] == 4:
        df1.loc[ind] = [gpsdf["utm_easting.data"][ind], gpsdf["utm_northing.data"][ind]]

    elif gpsdf["fix_quality.data"][ind] == 5:
        df2.loc[ind] = [gpsdf["utm_easting.data"][ind], gpsdf["utm_northing.data"][ind]]
    
    elif gpsdf["fix_quality.data"][ind] == 1:  
        df3.loc[ind] = [gpsdf["utm_easting.data"][ind], gpsdf["utm_northing.data"][ind]]


easting_mean_fix = float(df1["easting_fix"].mean())
northing_mean_fix = float(df1["northing_fix"].mean())

easting_mean_float = float(df2["easting_float"].mean())
northing_mean_float = float(df2["northing_float"].mean())

easting_mean_single = float(df3["easting_single"].mean())
northing_mean_single = float(df3["northing_single"].mean())


for i in df1.index:
    distance_fix = math.sqrt((easting_mean_fix - float(df1["easting_fix"][i]))**2 + (northing_mean_fix - float(df1["northing_fix"][i]))**2 )
    if distance_fix > max_dist_fix:
        max_dist_fix = distance_fix
    else:
        pass

for j in df2.index:
    distance_float = math.sqrt((easting_mean_float - float(df2["easting_float"][j]))**2 + (northing_mean_float - float(df2["northing_float"][j]))**2 )
    if distance_float > max_dist_float:
        max_dist_float = distance_float
    else:
        pass

for k in df3.index:
    distance_single = math.sqrt((easting_mean_single - float(df3["easting_single"][k]))**2 + (northing_mean_single - float(df3["northing_single"][k]))**2 )
    if distance_single > max_dist_single:
        max_dist_single = distance_single
    else:
        pass



print("Maximum distance from mean for solution status fix(Error):%fm" %max_dist_fix)
print("Maximum distance from mean for solution status float(Error):%fm" %max_dist_float)

if max_dist_single != 0:
    print("Maximum distance from mean for single data (Error):%fm" %max_dist_single)
else:
    pass

fig, ax = plt.subplots()

ax.scatter(df2["easting_float"], df2["northing_float"],label= "UTM(Easting,Northing) for solution status FLOAT", c ='red')
ax.scatter(df1["easting_fix"], df1["northing_fix"],label= "UTM(Easting,Northing) for solution status FIX", c='green')
plt.legend(loc="upper left")
plt.xlabel("UTM Easting(m)")
plt.ylabel("UTM Northing(m)")
plt.grid(True)
plt.title("Easting vs Northing with obstructions(Stationary)")
plt.show()



