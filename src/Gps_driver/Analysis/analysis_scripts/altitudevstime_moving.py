import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
b = bagreader('/home/arvinder/RSN/src/Gps_driver/Analysis/rosbag_data/ISEC_moving.bag')

gpsdata= b.message_by_topic(topic="/gpstopic")
gpsdataread = pd.read_csv(gpsdata)
gpsdf = pd.DataFrame(gpsdataread, columns=[ "Time", "latitude.data", "longitude.data", "altitude.data", "utm_easting.data", "utm_northing.data", "fix_quality.data"]).astype(float)
gpsdf["Time"] = (gpsdf["Time"] - gpsdf.at[0,"Time"])/60
df1 = pd.DataFrame(columns=["altitude_fix","Time"])
df2 = pd.DataFrame(columns=["altitude_float","Time"])
df3 = pd.DataFrame(columns=["altitude_single","Time"])


for ind in gpsdf.index:
    if gpsdf["fix_quality.data"][ind] == 4:
        df1.loc[ind] = [gpsdf["altitude.data"][ind], gpsdf["Time"][ind]]

    elif gpsdf["fix_quality.data"][ind] == 5:
        df2.loc[ind] = [gpsdf["altitude.data"][ind], gpsdf["Time"][ind]]
    
    elif gpsdf["fix_quality.data"][ind] == 1:  
        df3.loc[ind] = [gpsdf["altitude.data"][ind], gpsdf["Time"][ind]]


fig, ax = plt.subplots()

ax.scatter(df2["altitude_float"], df2["Time"],label= "Altitude at time t for solution status FLOAT", c ='red')
ax.scatter(df3["altitude_single"], df3["Time"],label= "Altitude at time t for solution status Single", c='black')
ax.scatter(df1["altitude_fix"], df1["Time"],label= "Altitude at time t for solution status FIX", c='green')
plt.legend(loc="upper right")
plt.xlabel("Time(min)")
plt.ylabel("Altitude(m)")
plt.grid(True)
plt.title("Time vs Altitude with obstructions(Moving)") #use without obstructions accordingly with the data
plt.show()
