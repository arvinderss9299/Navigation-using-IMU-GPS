import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score

b = bagreader('/home/arvinder/RSN/src/Gps_driver/Analysis/rosbag_data/Soccer_field_moving.bag')

gpsdata= b.message_by_topic(topic="/gpstopic")
gpsdataread = pd.read_csv(gpsdata)
gpsdf = pd.DataFrame(gpsdataread, columns=[ "latitude.data", "longitude.data", "altitude.data", "utm_easting.data", "utm_northing.data", "fix_quality.data"]).astype(float)


plot_gps = plt.scatter(gpsdf["utm_easting.data"], gpsdf["utm_northing.data"], label= "UTM(X,Y)")
X1  = gpsdf["utm_easting.data"][:281].values.reshape(-1, 1)
Y1 = gpsdf["utm_northing.data"][:281].values.reshape(-1, 1)
linear_regressor1 = LinearRegression()
linear_regressor1.fit(X1, Y1)  
Y_pred1 = linear_regressor1.predict(X1)
plt.plot(X1, Y_pred1, color = "r", label= "Regression Line")
max_dist1 = abs(Y1 - Y_pred1).max()
mean_dist1 = abs(Y1 - Y_pred1).mean()
r1 = r2_score(Y1, Y_pred1)

X2  = gpsdf["utm_easting.data"][282:447].values.reshape(-1, 1)
Y2 = gpsdf["utm_northing.data"][282:447].values.reshape(-1, 1)
linear_regressor2 = LinearRegression()
linear_regressor2.fit(X2, Y2)  
Y_pred2 = linear_regressor2.predict(X2)
plt.plot(X2, Y_pred2, color = "r")
max_dist2 = abs(Y2 - Y_pred2).max()
mean_dist2 = abs(Y2 - Y_pred2).mean()
r2 = r2_score(Y2, Y_pred2)

X3  = gpsdf["utm_easting.data"][448:681].values.reshape(-1, 1)
Y3 = gpsdf["utm_northing.data"][448:681].values.reshape(-1, 1)
linear_regressor3 = LinearRegression()
linear_regressor3.fit(X3, Y3)  
Y_pred3 = linear_regressor3.predict(X3)
plt.plot(X3, Y_pred3, color = "r")
max_dist3 = abs(Y3 - Y_pred3).max()
mean_dist3 = abs(Y3 - Y_pred3).mean()
r3 = r2_score(Y3, Y_pred3)

X4  = gpsdf["utm_easting.data"][682:].values.reshape(-1, 1)
Y4 = gpsdf["utm_northing.data"][682:].values.reshape(-1, 1)
linear_regressor4 = LinearRegression()
linear_regressor4.fit(X4, Y4)  
Y_pred4 = linear_regressor4.predict(X4)
plt.plot(X4, Y_pred4, color = "r")
max_dist4 = abs(Y4 - Y_pred4).max()
mean_dist4 = abs(Y4 - Y_pred4).mean()
r4 = r2_score(Y4, Y_pred4)

print("Maximum Distance(error) from best fit line on corners: %fm" %max(max_dist1,max_dist2,max_dist3,max_dist4))
print("Maximum Distance(error) from best fit line while moving straight: %fm" %max(mean_dist1, mean_dist2, mean_dist3, mean_dist4))
print("Coefficient of determination: %f" %min(r1,r2,r3,r4))

plt.legend(loc="upper right")
plt.title("UTM Easting vs UTM Northing without obstructions(Moving)")
plt.xlabel("UTM Easting(m)")
plt.ylabel("UTM Northing(m)")
plt.grid(True)
plt.show()





